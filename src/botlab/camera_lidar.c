#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <inttypes.h>
#include <unistd.h>
#include <pthread.h>
#include <signal.h>
#include <lcm/lcm.h>
#include <math.h>

#include "vx/vx.h"
#include "vx/vxo_drawables.h"
#include "vx/vx_remote_display_source.h"

#include "common/config.h"
#include "common/getopt.h"
#include "common/timestamp.h"
#include "common/zarray.h"

#include "math/math_util.h"
#include "math/gsl_util_vector.h"
#include "math/gsl_util_matrix.h"
#include "math/ssc.h"
#include "math/so3.h"

#include "imagesource/image_util.h"
#include "imagesource/image_source.h"
#include "imagesource/image_convert.h"

#include "lcmtypes/maebot_diff_drive_t.h"
#include "lcmtypes/rplidar_laser_t.h"

#define JOYSTICK_REVERSE_SPEED1 -0.25f
#define JOYSTICK_FORWARD_SPEED1  0.35f

#define JOYSTICK_REVERSE_SPEED2 -0.35f
#define JOYSTICK_FORWARD_SPEED2  0.45f

// Camera calibration struct (Caltech model)
typedef struct calib calib_t;
struct calib 
{
    char *class;
    double skew;	// Camera skew
    double fc[2];	// Focal length(s)
    double cc[2];	// Principal point (x,y)
    int kc_len;		// Number of radial distortion coefficients
    double *kc;		// Radial Distortion coefficients
    int lc_len;		// Length of L Tangential parameters
    double *lc;		// Tangential Distortion coefficients
};

typedef struct state state_t;
struct state 
{
    bool running;

    getopt_t *gopt;
    char *url;
    image_source_t *isrc;
    int fidx;

    lcm_t *lcm;

    maebot_diff_drive_t cmd;
    pthread_t command_thread;
    pthread_t render_thread;

    vx_world_t *vw;
    vx_application_t app;
    vx_event_handler_t veh;
    zhash_t *layer_map; // <display, layer>

    config_t *config;
    calib_t *calib;
    gsl_matrix *K;
    gsl_matrix *H;
    gsl_matrix *cal_matrix;
    
    zarray_t *laser_points; // zarray of float[3] {x, y, z} elements

    
    pthread_mutex_t mutex;
};

static int verbose = 0;

// From http://www.cs.rit.edu/~ncs/color/t_convert.html
static uint32_t hsv2rgb (float h, float s, float v)
{
    float r, g, b;
    if (s == 0) {
        // achromatic (gray)
        r = g = b = v;
    }
    else {
        h /= 60.0; // sector 0 to 5
        int i = floor (h);
        float f = h - i; // factorial part of h
        float p = v * ( 1 - s );
        float q = v * ( 1 - s * f );
        float t = v * ( 1 - s * ( 1 - f ) );
        switch (i) {
            case 0:
                r = v;
                g = t;
                b = p;
                break;
            case 1:
                r = q;
                g = v;
                b = p;
                break;
            case 2:
                r = p;
                g = v;
                b = t;
                break;
            case 3:
                r = p;
                g = q;
                b = v;
                break;
            case 4:
                r = t;
                g = p;
                b = v;
                break;
            default: // case 5:
                r = v;
                g = p;
                b = q;
                break;
        }
    }

    uint32_t abgr = (int)(r * 0xFF) | (int)(g * 0xFF) << 8 | (int)(b * 0xFF) << 16 | 0xFF << 24;
    return abgr;
}

static void rplidar_handler (const lcm_recv_buf_t *rbuf, const char *channel, const rplidar_laser_t *msg, void *user)
{
    state_t *state = user;

    pthread_mutex_lock (&state->mutex);
    {
        zarray_clear (state->laser_points);

        for (int i=0; i < msg->nranges; i++) 
	{
            // CONVERT POINTS TO CAMERA FRAME AND STORE IN ZARRAY
            float p[3];
            zarray_add (state->laser_points, p);
        }
    }
    pthread_mutex_unlock (&state->mutex);
    printf ("msg->utime = %"PRId64"\n", msg->utime);
}

static void * command_thread (void *user)
{
    state_t *state = user;

    while (state->running) {
        pthread_mutex_lock (&state->mutex);
        {
            state->cmd.utime = utime_now ();
            maebot_diff_drive_t_publish (state->lcm,  "MAEBOT_DIFF_DRIVE", &state->cmd);
        }
        pthread_mutex_unlock (&state->mutex);

        // send at 20 hz
        const int hz = 20;
        usleep (1000000/hz);
    }
    return NULL;
}

/**
 * @brief Initialize calibration matrices
 */
void initialize_calibration(state_t *state)
{
    gsl_matrix *H = state->H;
    gsl_matrix *K = state->K;
    calib_t *cal = state->calib;
    
    // Set Intrinsics matrix
    gsl_matrix_set(K, 0, 0, cal->fc[0]); // Focal-x
    gsl_matrix_set(K, 1, 1, cal->fc[1]); // Focal-y
    gsl_matrix_set(K, 0, 1, cal->skew);	 // Skew
    gsl_matrix_set(K, 0, 2, cal->cc[0]); // Principal-x
    gsl_matrix_set(K, 1, 2, cal->cc[1]); // Principal-y
    gsl_matrix_set(K, 2, 2, 1);
    
    // Set Extrinsics matrix
    gsl_matrix_set(H, 0, 0, 1);		// No rotation R=I
    gsl_matrix_set(H, 1, 1, 1);		
    gsl_matrix_set(H, 2, 2, 1);		
    gsl_matrix_set(H, 2, 3, 0.088);	// Z-translation
    
    state->cal_matrix = gslu_blas_mm_alloc(K, H); // Store out combined cal matrix
}

/**
 * @brief Main rendering thread
 */
static void * render_thread (void *user)
{
    state_t *state = user;
    image_source_t *isrc = state->isrc;

    const int Hz = 5;

    if (verbose)
        printf ("Starting render_thread\n");

    vx_buffer_t *vb_text = vx_world_get_buffer (state->vw, "text");
    vx_buffer_t *vb_image = vx_world_get_buffer (state->vw, "image");
    
    // Render viewer text
    vx_object_t *vt = vxo_text_create (VXO_TEXT_ANCHOR_TOP_RIGHT, "<<right,#0000ff>>Robot viewer!\n");
    vx_buffer_add_back (vb_text, vxo_pix_coords (VX_ORIGIN_TOP_RIGHT, vt));
    vx_buffer_swap (vb_text);
    
    // Render loop
    while (state->running) 
    {
        int64_t t0 = utime_now ();
        printf ("t0 = %"PRId64"\n", t0);

        image_u32_t *im = NULL;
        if (isrc)
	{
            image_source_data_t isdata;
            int res = isrc->get_frame (isrc, &isdata);
            if (!res)
                im = image_convert_u32 (&isdata);
            else
                goto error;

            isrc->release_frame (isrc, &isdata);

            if (verbose)
                printf("Got frame %p\n", im);
        }

        if (im != NULL) 
	{
            pthread_mutex_lock (&state->mutex);
            {
                // Project lidar points into camera image
                if (state->calib) 
		{
                    const calib_t *calib = state->calib;
		    
		    gsl_matrix *cmat = state->cal_matrix;
		    //float lpts[2*zarray_size(state->laser_points)];
		    //int laserImgCount;
                    for (int i=0; i < zarray_size (state->laser_points); i++) 
		    {
                        // laser point in camera reference frame
                        float p_c[3];
                        zarray_get (state->laser_points, i, p_c); // TODO: Check z-correctness of laser
                        float X = p_c[0], Y = p_c[1], Z = p_c[2];
    
			gsl_vector *xyz_world = gsl_vector_alloc(4);
			gsl_vector_set(xyz_world, 0, X);
			gsl_vector_set(xyz_world, 0, Y);
			gsl_vector_set(xyz_world, 0, Z);
			gsl_vector_set(xyz_world, 3, 1);
			
			gsl_vector *xy_n = gslu_blas_mv_alloc(cmat, xyz_world); // Linear undistorted image coordinates
			
			// Pinhole model w/ distortion
                        int u_d = 0;
                        int v_d = 0;
                        if (0==strcmp (calib->class, "april.camera.models.CaltechCalibration")) 
			{
			    printf("Using Caltech Distortion/Calibration Model\n");
			    double x = gsl_vector_get(xy_n, 0);
			    double y = gsl_vector_get(xy_n, 1);
			    
			    double r2 = pow(x,2) + pow(y,2);
			    double r4 = pow(r2, 2);
			    double r6 = pow(r2, 3);
			    double l0 = state->calib->lc[0];
			    double l1 = state->calib->lc[1];
			    double k0 = state->calib->kc[0];
			    double k1 = state->calib->kc[1];
			    double k2 = state->calib->kc[2];
			    
			    // Compute Radial Distortion Scalar
			    double rd = 1 + k0*r2 + k1*r4 + k2*r6;
		
			    // Compute Tangential Distortion Vector
			    double dx0 = 2*l0*x*y + l1*(r2 + 2*pow(x,2));
			    double dx1 = l1*(r2 + 2*pow(y,2)) + 2*l1*x*y;
			    u_d = round(rd * x + dx0);
			    v_d = round(rd * y + dx1);
			}
                        else if (0==strcmp (calib->class, "april.camera.models.AngularPolynomialCalibration")) 
			{
			    printf("Using Angular Polynomial Distortion/Calibration Model\n");
                            // TODO: (Angular Polynomial Calibration?)
                        }
                        else {
                            printf ("error: unsupported distortion model: %s\n", calib->class);
                            exit (EXIT_FAILURE);
                        }

                        // Draw projected lidar points in image
                        if (u_d > 0 && u_d < im->width-1 && v_d > 0 && v_d < im->height-1) 
			{
                            const float depth = 256;
                            float hue = Z * depth;
                            if (hue > depth)
                                hue = depth;
                            uint32_t color = hsv2rgb(hue, 1.0, 1.0);
			    printf("LIDAR Point at X:%d, Y:%d\n", u_d, v_d);
			    float points[2] = {u_d, v_d};
                            vx_buffer_add_back(vb_image, vxo_chain(vxo_mat_translate2 (u_d, v_d), vxo_points( vx_resc_copyf (points, 2), 1, vxo_points_style(vx_red, 2.0f))));
			    
			    
			    //vx_buffer_add_back();
                        }
                        
                        gslu_vector_free(xyz_world);
			gslu_vector_free(xy_n);
                    }
                }
            }
            pthread_mutex_unlock (&state->mutex);

            double decimate = getopt_get_double (state->gopt, "decimate");
            if (decimate != 1.0) 
	    {
                image_u32_t *im2 = image_util_u32_decimate (im, decimate);
                image_u32_destroy (im);
                im = im2;
            }

            // Render downsampled image, but scale it so it appears the
            // same size as the original
            vx_object_t *vo = vxo_image_from_u32 (im, VXO_IMAGE_FLIPY, VX_TEX_MIN_FILTER);
            vx_buffer_add_back (vb_image, vxo_pix_coords (VX_ORIGIN_TOP_LEFT,
                                                          vxo_chain (vxo_mat_scale (decimate),
                                                                     vxo_mat_translate3 (0, -im->height, 0),
                                                                     vo)));
            vx_buffer_swap (vb_image);
            image_u32_destroy (im);
        }
        usleep (1000000/Hz - (utime_now () - t0));
    }

  error:
    isrc->stop (isrc);
    isrc->close (isrc);
    printf ("exiting\n");
    return NULL;
}

static int touch_event (vx_event_handler_t *vh, vx_layer_t *vl, vx_camera_pos_t *pos, vx_touch_event_t *mouse)
{
    return 0;
}

static int mouse_event (vx_event_handler_t *vh, vx_layer_t *vl, vx_camera_pos_t *pos, vx_mouse_event_t *mouse)
{
    return 0;
}

static int key_event (vx_event_handler_t *vh, vx_layer_t *vl, vx_key_event_t *key)
{
    state_t *state = vh->impl;

    static bool key_shift=0, key_up=0, key_down=0, key_left=0, key_right=0, manual_control;


    switch (key->key_code) 
    {
        case VX_KEY_SHIFT:
            key_shift = !key->released;
            break;
        case VX_KEY_UP:
            key_up = !key->released;
            break;
        case VX_KEY_DOWN:
            key_down = !key->released;
            break;
        case VX_KEY_LEFT:
            key_left = !key->released;
            break;
        case VX_KEY_RIGHT:
            key_right = !key->released;
            break;
        case VX_KEY_CTRL:
            manual_control = !key->released;
            if (key->released)
                state->cmd.motor_left_speed = state->cmd.motor_right_speed = 0.0;
        default:
            break;
    }

    if (manual_control) 
    {
        pthread_mutex_lock (&state->mutex);
        {
            // default to zero
            state->cmd.motor_left_speed = state->cmd.motor_right_speed = 0.0;

            float fwd_speed = JOYSTICK_FORWARD_SPEED1;
            float rev_speed = JOYSTICK_REVERSE_SPEED1;
            if (key_shift) { // speed boost
                fwd_speed = JOYSTICK_FORWARD_SPEED2;
                rev_speed = JOYSTICK_REVERSE_SPEED2;
            }

            if (key_up) { // forward
                state->cmd.motor_left_speed = fwd_speed;
                state->cmd.motor_right_speed = fwd_speed;
                if (key_left) {
                    state->cmd.motor_left_speed -= 0.1;
                    state->cmd.motor_right_speed += 0.1;
                }
                else if (key_right) {
                    state->cmd.motor_left_speed += 0.1;
                    state->cmd.motor_right_speed -= 0.1;
                }
            }
            else if (key_down) { // reverse
                state->cmd.motor_left_speed = rev_speed;
                state->cmd.motor_right_speed = rev_speed;
                if (key_left) {
                    state->cmd.motor_left_speed += 0.1;
                    state->cmd.motor_right_speed -= 0.1;
                }
                else if (key_right) {
                    state->cmd.motor_left_speed -= 0.1;
                    state->cmd.motor_right_speed += 0.1;
                }
            }
            else if (key_left) { // turn left
                state->cmd.motor_left_speed =  rev_speed;
                state->cmd.motor_right_speed = -rev_speed;
            }
            else if (key_right) { // turn right
                state->cmd.motor_left_speed = -rev_speed;
                state->cmd.motor_right_speed = rev_speed;
            }
        }
        pthread_mutex_unlock (&state->mutex);
    }

    return 0;
}

static void display_finished (vx_application_t *app, vx_display_t *disp)
{
    state_t *state = app->impl;
    pthread_mutex_lock (&state->mutex);
    {
        // retrieve reference to the world and layer that we associate with each vx_display_t
        vx_layer_t *layer = NULL;
        zhash_remove (state->layer_map, &disp, NULL, &layer);
        vx_layer_destroy (layer);
    }
    pthread_mutex_unlock (&state->mutex);
}

static void display_started (vx_application_t *app, vx_display_t *disp)
{
    state_t *state = app->impl;

    vx_layer_t *layer = vx_layer_create (state->vw);
    vx_layer_set_display (layer, disp);
    vx_layer_add_event_handler (layer, &state->veh);

    pthread_mutex_lock (&state->mutex);
    {
        // store a reference to the world and layer that we associate with each vx_display_t
        zhash_put (state->layer_map, &disp, &layer, NULL, NULL);
    }
    pthread_mutex_unlock (&state->mutex);
}


static void nodestroy (vx_event_handler_t *vh)
{
    // do nothing, since this event handler is statically allocated.
}

static state_t *global_state;

static void handler (int signum)
{
    if (!global_state->running)
        exit (EXIT_FAILURE);

    switch (signum) 
    {
        case SIGINT:
        case SIGQUIT:
            global_state->running = 0;
            break;
        default:
            break;
    }
}

calib_t * load_camera_calib (getopt_t *gopt)
{
    calib_t *calib = calloc (1, sizeof *calib);

    // Load camera calibration
    char *config_path = realpath (getopt_get_string (gopt, "config"), NULL);
    if (config_path == NULL) 
    {
        perror ("couldn't resolve path for config file");
        exit (EXIT_FAILURE);
    }

    FILE *config_file = fopen (config_path, "r");
    if (!config_file) 
    {
        perror ("couldn't open config file");
        exit (EXIT_FAILURE);
    }
    config_t *config = config_parse_file (config_file, config_path);

    calib->class = config_get_str_or_fail (config, "aprilCameraCalibration.camera0000.class");
    calib->skew = config_get_double_or_default (config, "aprilCameraCalibration.camera0000.intrinsics.skew", 0.0);
    config_get_double_array (config, "aprilCameraCalibration.camera0000.intrinsics.fc", calib->fc, 2);
    config_get_double_array (config, "aprilCameraCalibration.camera0000.intrinsics.cc", calib->cc, 2);

    calib->kc_len = config_get_array_len (config, "aprilCameraCalibration.camera0000.intrinsics.kc");
    if (calib->kc_len < 0) 
    {
        printf ("error: kc not found in config\n");
        exit (EXIT_FAILURE);
    }
    else 
    {
        calib->kc = calloc (calib->kc_len, sizeof *(calib->kc));
        config_get_double_array (config, "aprilCameraCalibration.camera0000.intrinsics.kc", calib->kc, calib->kc_len);
    }

    if (0==strcmp (calib->class, "april.camera.models.CaltechCalibration")) 
    {
        calib->lc_len = config_get_array_len (config, "aprilCameraCalibration.camera0000.intrinsics.lc");
        if (calib->lc_len > 0) 
	{
            calib->lc = calloc (calib->lc_len, sizeof *(calib->lc));
            config_get_double_array (config, "aprilCameraCalibration.camera0000.intrinsics.lc", calib->lc, calib->lc_len);
        }
    }

    // print calib to stdout
    printf ("Calibration config:\n");
    printf ("    class=%s\n", calib->class);
    printf ("    skew: %f\n", calib->skew);
    printf ("    fc: %f, %f\n", calib->fc[0], calib->fc[1]);
    printf ("    cc: %f, %f\n", calib->cc[0], calib->cc[1]);
    switch (calib->kc_len) 
    {
        case 3:
            printf ("    kc: %f, %f, %f\n", calib->kc[0], calib->kc[1], calib->kc[2]);
            break;
        case 4:
            printf ("    kc: %f, %f, %f, %f\n", calib->kc[0], calib->kc[1], calib->kc[2], calib->kc[3]);
            break;
        default:
            printf ("unhandled case kc_len=%d\n", calib->kc_len);
            exit (EXIT_FAILURE);
    }
    switch (calib->lc_len) 
    {
        case 0:
            break;
        case 2:
            printf ("    lc: %f, %f\n", calib->lc[0], calib->lc[1]);
            break;
        default:
            printf ("unhandled case lc_len=%d\n", calib->lc_len);
            exit (EXIT_FAILURE);
    }

    return calib;
}

state_t * state_create (void)
{
    state_t *state = calloc (1, sizeof(*state));

    state->gopt = getopt_create ();
    state->app.display_finished = display_finished;
    state->app.display_started = display_started;
    state->app.impl = state;
    state->veh.dispatch_order = -10;
    state->veh.touch_event = touch_event;
    state->veh.mouse_event = mouse_event;
    state->veh.key_event = key_event;
    state->veh.destroy = nodestroy;
    state->veh.impl = state;

    state->running = 1;
    state->lcm = lcm_create (NULL);
    state->vw = vx_world_create ();
    state->layer_map = zhash_create (sizeof(vx_display_t *), sizeof(vx_layer_t *), zhash_ptr_hash, zhash_ptr_equals);
    state->laser_points = zarray_create (sizeof(float[3]));

    // Camera linear calibration matrices
    state->H = gsl_matrix_calloc(3,4); // Camera extrinsics
    state->K = gsl_matrix_calloc(3,3); // Camera intrinsics
    
    // note, pg_sd() family of functions will trigger their own callback of my_param_changed(),
    // hence using a recursive mutex avoids deadlocking when using pg_sd() within my_param_changed()
    pthread_mutexattr_t attr;
    pthread_mutexattr_init (&attr);
    pthread_mutexattr_settype (&attr, PTHREAD_MUTEX_RECURSIVE);
    pthread_mutex_init (&state->mutex, &attr);

    return state;
}

int main (int argc, char *argv[])
{
    // so that redirected stdout won't be insanely buffered.
    setvbuf (stdout, (char *) NULL, _IONBF, 0);

    vx_global_init ();

    state_t *state = NULL;
    global_state = state = state_create ();

    signal (SIGINT, handler);

    getopt_add_bool (state->gopt, 'h', "help", 0, "Show this help");
    getopt_add_bool (state->gopt, 'v', "verbose", 0, "Show extra debugging output");
    getopt_add_int (state->gopt, 'l', "limitKBs", "-1", "Remote display bandwidth limit. < 0: unlimited.");
    getopt_add_int (state->gopt, 'd', "decimate", "1", "Decimate image by this amount before showing in vx");
    getopt_add_string (state->gopt, '\0', "url", "", "Camera URL");
    getopt_add_bool (state->gopt, '\0', "no-video", 0, "Disable video");
    getopt_add_string (state->gopt, '\0', "config", "../config/camera.config", "Camera calibration config");

    if (!getopt_parse (state->gopt, argc, argv, 0)) 
    {
        getopt_do_usage (state->gopt);
        exit (EXIT_FAILURE);
    }
    else if (getopt_get_bool (state->gopt,"help")) 
    {
        getopt_do_usage (state->gopt);
        exit (EXIT_SUCCESS);
    }

    verbose = getopt_get_bool (state->gopt, "verbose");


    if (!getopt_get_bool (state->gopt, "no-video")) 
    {
        // Set up the imagesource. This looks for a camera url specified on
        // the command line and, if none is found, enumerates a list of all
        // cameras imagesource can find and picks the first url it finds.
        if (strncmp (getopt_get_string (state->gopt, "url"), "", 1)) 
	{
            state->url = strdup (getopt_get_string (state->gopt, "url"));
            printf ("URL: %s\n", state->url);
        }
        else 
	{
            // No URL specified. Show all available and then use the first
            zarray_t *urls = image_source_enumerate ();
            printf ("Cameras:\n");
            for (int i = 0; i < zarray_size (urls); i++) 
	    {
                char *url;
                zarray_get (urls, i, &url);
                printf ("  %3d: %s\n", i, url);
            }

            if (0==zarray_size (urls)) 
	    {
                printf ("No cameras found.\n");
                exit (EXIT_FAILURE);
            }
            zarray_get (urls, 0, &state->url);
        }

        state->isrc = image_source_open (state->url);
        if (state->isrc == NULL) 
	{
            printf ("Unable to open device %s\n", state->url);
            exit (EXIT_FAILURE);
        }

        image_source_t *isrc = state->isrc;
        if (isrc->start (isrc))
            exit (EXIT_FAILURE);

        if (getopt_was_specified (state->gopt, "config"))
	{
            state->calib = load_camera_calib (state->gopt);
            if (!state->calib)
                exit (EXIT_FAILURE);
        }
        else
	{
            printf ("No Calibration Specified, using default\n");
	    state->calib = load_camera_calib(state->gopt);
	}
    }

    // Setup Vx remote display
    vx_remote_display_source_attr_t remote_attr;
    vx_remote_display_source_attr_init (&remote_attr);
    remote_attr.max_bandwidth_KBs = getopt_get_int (state->gopt, "limitKBs");
    remote_attr.advertise_name = "Maebot Teleop";
    vx_remote_display_source_t *remote = vx_remote_display_source_create_attr (&state->app, &remote_attr);

    // Launch worker threads
    pthread_create (&state->command_thread,  NULL, command_thread, state);
    pthread_create (&state->render_thread,  NULL, render_thread, state);

    // Subscribe to rplidar scans
    rplidar_laser_t_subscribe (state->lcm, "RPLIDAR_LASER", rplidar_handler, state);

    while (state->running)
        lcm_handle_timeout (state->lcm, 250);

    pthread_join (state->command_thread, NULL);
    pthread_join (state->render_thread, NULL);

    printf ("vx_remote_display_source_destroy...");
    vx_remote_display_source_destroy (remote);
    printf ("done\n");
}
