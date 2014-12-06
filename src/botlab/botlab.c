#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <inttypes.h>
#include <unistd.h>
#include <pthread.h>
#include <signal.h>
#include <math.h>
#include <lcm/lcm.h>

#include "vx/vx.h"
#include "vx/vxo_drawables.h"
#include "vx/vx_remote_display_source.h"

#include "common/getopt.h"
#include "common/timestamp.h"

#include "math/matd.h"
#include "math/math_util.h"
#include "math/gsl_util_vector.h"
#include "math/gsl_util_matrix.h"
#include "math/gsl_util_eigen.h"
#include "math/gsl_util_blas.h"
#include "math/homogenous.h"

#include "imagesource/image_util.h"
#include "imagesource/image_source.h"
#include "imagesource/image_convert.h"

#include "lcmtypes/maebot_diff_drive_t.h"
#include "lcmtypes/maebot_laser_t.h"
#include "lcmtypes/maebot_leds_t.h"
#include "lcmtypes/maebot_sensor_data_t.h"
#include "lcmtypes/maebot_motor_feedback_t.h"
#include "lcmtypes/pose_xyt_t.h"
#include "lcmtypes/rplidar_laser_t.h"

#include "xyt.h"

#define JOYSTICK_REVERSE_SPEED1 -0.25f
#define JOYSTICK_FORWARD_SPEED1  0.35f

#define JOYSTICK_REVERSE_SPEED2 -0.35f
#define JOYSTICK_FORWARD_SPEED2  0.45f

#define MAX_REVERSE_SPEED -0.35f
#define MAX_FORWARD_SPEED  0.35f

#define VXO_GRID_SIZE 0.25 // [m]

#define SLIDING_TIME_WINDOW 10000000 // 10 s

#define GOAL_RADIUS 0.10 // [m]

#define dmax(A,B) A < B ? B : A
#define dmin(A,B) A < B ? A : B

typedef struct state state_t;

struct state 
{
    bool running;
    getopt_t *gopt;
    char *url;
    image_source_t *isrc;
    int fidx;
    lcm_t *lcm;

    pose_xyt_t *pose;

    pthread_t command_thread;
    maebot_diff_drive_t cmd;
    bool manual_control;

    pthread_t render_thread;
    bool   have_goal;
    double goal[3];

    vx_world_t *vw;
    vx_application_t app;
    vx_event_handler_t veh;
    zhash_t *layer_map; // <display, layer>

    pthread_mutex_t mutex;
};


static void display_finished (vx_application_t *app, vx_display_t *disp)
{
    state_t *state = app->impl;

    pthread_mutex_lock (&state->mutex);
    {
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

    vx_layer_camera_op (layer, OP_PROJ_PERSPECTIVE);
    float eye[3]    = {  0,  0,  5};
    float lookat[3] = {  0,  0,  0 };
    float up[3]     = {  0,  1,  0 };
    vx_layer_camera_lookat (layer, eye, lookat, up, 1);

    pthread_mutex_lock (&state->mutex);
    {
        zhash_put (state->layer_map, &disp, &layer, NULL, NULL);
    }
    pthread_mutex_unlock (&state->mutex);
}

static int touch_event (vx_event_handler_t *vh, vx_layer_t *vl, vx_camera_pos_t *pos, vx_touch_event_t *mouse)
{
    return 0;
}

static int mouse_event (vx_event_handler_t *vh, vx_layer_t *vl, vx_camera_pos_t *pos, vx_mouse_event_t *mouse)
{
    state_t *state = vh->impl;

    // Button state
    bool m1 = mouse->button_mask & VX_BUTTON1_MASK;
    bool ctrl = mouse->modifiers & VX_CTRL_MASK;
    //bool alt = mouse->modifiers & VX_ALT_MASK;
    //bool shift = mouse->modifiers & VX_SHIFT_MASK;

    pthread_mutex_lock (&state->mutex);
    {
        if (m1 && ctrl) {
            // Ray cast to find click point
            vx_ray3_t ray;
            vx_camera_pos_compute_ray (pos, mouse->x, mouse->y, &ray);

            double ground[3];
            vx_ray3_intersect_xy (&ray, 0, ground);

            printf ("Mouse clicked at coords: [%8.3f, %8.3f]  Ground clicked at coords: [%6.3f, %6.3f]\n",
                    mouse->x, mouse->y, ground[0], ground[1]);

            state->goal[0] = ground[0];
            state->goal[1] = ground[1];
            state->have_goal = true;
        }
    }
    pthread_mutex_unlock (&state->mutex);

    return 0;
}

static int key_event (vx_event_handler_t *vh, vx_layer_t *vl, vx_key_event_t *key)
{
    state_t *state = vh->impl;

    static bool key_shift=0, key_up=0, key_down=0, key_left=0, key_right=0;

    switch (key->key_code) {
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
            state->manual_control = !key->released;
            if (key->released)
                state->cmd.motor_left_speed = state->cmd.motor_right_speed = 0.0;
            break;
        default:
            break;
    }

    if (state->manual_control) {
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

static void destroy (vx_event_handler_t *vh)
{
    // do nothing, since this event handler is statically allocated.
}

static state_t *global_state;
static void handler (int signum)
{
    switch (signum) {
        case SIGINT:
        case SIGQUIT:
            global_state->running = 0;
            break;
        default:
            break;
    }
}


// This thread continuously publishes command messages to the maebot
static void * command_thread (void *data)
{
    state_t *state = data;
    const uint32_t Hz = 20;
    const char *channel = getopt_get_string (state->gopt, "maebot-diff-drive-channel");

    while (state->running) {
        pthread_mutex_lock (&state->mutex);
        {
            if (!state->manual_control && state->have_goal) {
                // TODO: IMPLEMENT ME

            }
            // Publish
            state->cmd.utime = utime_now ();
            maebot_diff_drive_t_publish (state->lcm, channel, &(state->cmd));
        }
        pthread_mutex_unlock (&state->mutex);

        usleep (1000000/Hz);
    }

    return NULL;
}

// This thread continously renders updates from the robot
static void * render_thread (void *data)
{
    state_t *state = data;

    // Grid
    {
        vx_buffer_t *vb = vx_world_get_buffer (state->vw, "grid");
        vx_buffer_set_draw_order (vb, 0);
        vx_buffer_add_back (vb,
                            vxo_chain (vxo_mat_scale (VXO_GRID_SIZE),
                                       vxo_grid ()));
        vx_buffer_swap (vb);
    }

    // Axes
    {
        vx_buffer_t *vb = vx_world_get_buffer (state->vw, "axes");
        vx_buffer_set_draw_order (vb, 0);
        vx_buffer_add_back (vb,
                            vxo_chain (vxo_mat_scale3 (0.10, 0.10, 0.0),
                                       vxo_mat_translate3 (0.0, 0.0, -0.005),
                                       vxo_axes_styled (vxo_mesh_style (vx_red),
                                                        vxo_mesh_style (vx_green),
                                                        vxo_mesh_style (vx_black))));
        vx_buffer_swap (vb);
    }

    const int fps = 30;
    while (state->running) {
        pthread_mutex_lock (&state->mutex);
        {
            // Goal
            if (state->have_goal) {
                float color[4] = {0.0, 1.0, 0.0, 0.5};
                vx_buffer_t *vb = vx_world_get_buffer (state->vw, "goal");
                vx_buffer_set_draw_order (vb, -1);
                vx_buffer_add_back (vb,
                                    vxo_chain (vxo_mat_translate2 (state->goal[0], state->goal[1]),
                                               vxo_mat_scale (GOAL_RADIUS),
                                               vxo_circle (vxo_mesh_style (color))));
                vx_buffer_swap (vb);
            }

            // Robot
            {
                vx_buffer_t *vb = vx_world_get_buffer (state->vw, "robot");
                vx_buffer_set_draw_order (vb, 1);
                enum {ROBOT_TYPE_TRIANGLE, ROBOT_TYPE_DALEK};
                vx_object_t *robot = NULL;
                switch (ROBOT_TYPE_DALEK) {
                    case ROBOT_TYPE_DALEK: {
                        float line[6] = {0.0, 0.0, 0.151, 0.104, 0.0, 0.151};
                        robot = vxo_chain (vxo_lines (vx_resc_copyf (line, 6),
                                                      2,
                                                      GL_LINES,
                                                      vxo_lines_style (vx_red, 3.0f)),
                                           vxo_mat_scale3 (0.104, 0.104, 0.151),
                                           vxo_mat_translate3 (0.0, 0.0, 0.5),
                                           vxo_cylinder (vxo_mesh_style (vx_blue)));
                        break;
                    }
                    case ROBOT_TYPE_TRIANGLE:
                    default:
                        robot = vxo_chain (vxo_mat_scale (0.104),
                                           vxo_mat_scale3 (1, 0.5, 1),
                                           vxo_triangle (vxo_mesh_style (vx_blue)));
                        break;
                }

                if (state->pose)
                    vx_buffer_add_back (vb,
                                        vxo_chain (vxo_mat_from_xyt (state->pose->xyt),
                                                   robot));
                else
                    vx_buffer_add_back (vb, robot);

                vx_buffer_swap (vb);
            }

            // Robot Covariance
            // HINT: vxo_circle is what you want

            // Current Lidar Scan
            // HINT: vxo_points is what you want
        }
        pthread_mutex_unlock (&state->mutex);
        usleep (1000000/fps);
    }

    return NULL;
}

// === LCM Handlers =================
static void maebot_motor_feedback_handler (const lcm_recv_buf_t *rbuf, const char *channel, const maebot_motor_feedback_t *msg, void *user)
{
    state_t *state = user;

    pthread_mutex_lock (&state->mutex);
    {
        // TODO: IMPLEMENT ME
    }
    pthread_mutex_unlock (&state->mutex);
}

static void maebot_sensor_data_handler (const lcm_recv_buf_t *rbuf, const char *channel, const maebot_sensor_data_t *msg, void *user)
{
    state_t *state = user;

    pthread_mutex_lock (&state->mutex);
    {
        // TODO: IMPLEMENT ME
    }
    pthread_mutex_unlock (&state->mutex);
}

static void pose_xyt_handler (const lcm_recv_buf_t *rbuf, const char *channel, const pose_xyt_t *msg, void *user)
{
    state_t *state = user;

    pthread_mutex_lock (&state->mutex);
    {
        // TODO: IMPLEMENT ME
	gsl_matrix *sig = gsl_matrix_alloc(3,3);// Reconstruct covariance matrix
	memcpy(sig->data, msg->Sigma, 9*sizeof(double));
	gslu_eigen *sig_eigs = gslu_eigen_decomp_alloc (sig);// Compute Eigenvalues/vectors of covariance
    
	printf("Eigenstuff\n");
	gslu_vector_printf(sig_eigs->D,"Evals");
	gslu_matrix_printf(sig_eigs->V,"Evecs");
	gslu_matrix_free(sig);
    }
    pthread_mutex_unlock (&state->mutex);
}

static void rplidar_laser_handler (const lcm_recv_buf_t *rbuf, const char *channel, const rplidar_laser_t *msg, void *user)
{
    state_t *state = user;

    pthread_mutex_lock (&state->mutex);
    {
        // TODO: IMPLEMENT ME
    }
    pthread_mutex_unlock (&state->mutex);
}

state_t *state_create (void)
{
    state_t *state = calloc (1, sizeof (*state));

    state->running = 1;
    state->gopt = getopt_create ();
    state->lcm = lcm_create (NULL);

    state->have_goal = false;

    state->vw = vx_world_create ();
    state->app.display_finished = display_finished;
    state->app.display_started = display_started;
    state->app.impl = state;
    state->veh.dispatch_order = -10;
    state->veh.touch_event = touch_event;
    state->veh.mouse_event = mouse_event;
    state->veh.key_event = key_event;
    state->veh.destroy = destroy;
    state->veh.impl = state;
    state->layer_map = zhash_create (sizeof(vx_display_t*), sizeof(vx_layer_t*), zhash_ptr_hash, zhash_ptr_equals);

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

    // Clean up on Ctrl+C
    signal (SIGINT, handler);
    signal (SIGQUIT, handler);

    getopt_add_bool (state->gopt, 'h', "help", 0, "Show this help");
    getopt_add_int (state->gopt, 'l', "limitKBs", "-1", "Remote display bandwith limit in KBs. < 0: unlimited.");
    getopt_add_int (state->gopt, 'p', "port", "15151", "Vx display port");
    getopt_add_string (state->gopt, '\0', "maebot-motor-feedback-channel", "MAEBOT_MOTOR_FEEDBACK", "LCM channel name");
    getopt_add_string (state->gopt, '\0', "maebot-sensor-data-channel", "MAEBOT_SENSOR_DATA", "LCM channel name");
    getopt_add_string (state->gopt, '\0', "maebot-diff-drive-channel", "MAEBOT_DIFF_DRIVE", "LCM channel name");
    getopt_add_string (state->gopt, '\0', "odometry-channel", "BOTLAB_ODOMETRY", "LCM channel name");
    getopt_add_string (state->gopt, '\0', "rplidar-laser-channel", "RPLIDAR_LASER", "LCM channel name");


    if (!getopt_parse (state->gopt, argc, argv, 0)) {
        getopt_do_usage (state->gopt);
        exit (EXIT_FAILURE);
    }
    else if (getopt_get_bool (state->gopt,"help")) {
        getopt_do_usage (state->gopt);
        exit (EXIT_SUCCESS);
    }

    // Set up Vx remote display
    vx_remote_display_source_attr_t remote_attr;
    vx_remote_display_source_attr_init (&remote_attr);
    remote_attr.max_bandwidth_KBs = getopt_get_int (state->gopt, "limitKBs");
    remote_attr.advertise_name = "Maebot App";
    remote_attr.connection_port = getopt_get_int (state->gopt, "port");
    vx_remote_display_source_t *remote = vx_remote_display_source_create_attr (&state->app, &remote_attr);

    // Video stuff?

    // LCM subscriptions
    maebot_motor_feedback_t_subscribe (state->lcm,
                                       getopt_get_string (state->gopt, "maebot-motor-feedback-channel"),
                                       maebot_motor_feedback_handler, state);
    maebot_sensor_data_t_subscribe (state->lcm,
                                    getopt_get_string (state->gopt, "maebot-sensor-data-channel"),
                                    maebot_sensor_data_handler, state);
    pose_xyt_t_subscribe (state->lcm,
                          getopt_get_string (state->gopt, "odometry-channel"),
                          pose_xyt_handler, state);
    rplidar_laser_t_subscribe (state->lcm,
                               getopt_get_string (state->gopt, "rplidar-laser-channel"),
                               rplidar_laser_handler, state);

    // Launch worker threads
    pthread_create (&state->command_thread, NULL, command_thread, state);
    pthread_create (&state->render_thread, NULL, render_thread, state);

    // Loop forever
    while (state->running)
        lcm_handle_timeout (state->lcm, 500);

    pthread_join (state->command_thread, NULL);
    pthread_join (state->render_thread, NULL);

    printf ("waiting vx_remote_display_source_destroy...");
    vx_remote_display_source_destroy (remote);
    printf ("done\n");
}
