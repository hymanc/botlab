#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <unistd.h>
#include <pthread.h>
#include <signal.h>
#include <lcm/lcm.h>

#include "vx/vx.h"
#include "vx/vxo_drawables.h"
#include "vx/vx_remote_display_source.h"

#include "common/getopt.h"
#include "common/timestamp.h"
#include "imagesource/image_util.h"
#include "imagesource/image_source.h"
#include "imagesource/image_convert.h"

#include "lcmtypes/maebot_diff_drive_t.h"

#define REVERSE_SPEED1 -0.25f
#define FORWARD_SPEED1  0.35f

#define REVERSE_SPEED2 -0.35f
#define FORWARD_SPEED2  0.45f

typedef struct state state_t;
struct state {
    vx_application_t app;
    vx_event_handler_t veh;

    maebot_diff_drive_t cmd;
    pthread_t cmd_thread;

    int running;

    getopt_t *gopt;
    char *url;
    image_source_t *isrc;
    int fidx;

    lcm_t *lcm;

    vx_world_t *vw;
    zhash_t *layer_map; // <display, layer>

    pthread_mutex_t mutex;
};


static int verbose = 0;

static void
display_finished (vx_application_t *app, vx_display_t *disp)
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

static void
display_started (vx_application_t *app, vx_display_t *disp)
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


void *
run_camera (void *data)
{
    state_t *state = data;
    image_source_t *isrc = state->isrc;

    if (verbose)
        printf ("Starting run_camera\n");

    while (state->running) {
        image_u32_t *im = NULL;
        {
            image_source_data_t isdata;
            int res = isrc->get_frame (isrc, &isdata);
            if (!res)
                im = image_convert_u32 (&isdata);
            else
                goto error;

            isrc->release_frame (isrc, &isdata);
        }

        if (verbose)
            printf("Got frame %p\n", im);

        if (im != NULL) {
            double decimate = getopt_get_double (state->gopt, "decimate");
            if (decimate != 1.0) {
                image_u32_t *im2 = image_util_u32_decimate (im, decimate);
                image_u32_destroy (im);
                im = im2;
            }

            vx_object_t *vo = vxo_image_from_u32 (im, VXO_IMAGE_FLIPY, VX_TEX_MIN_FILTER);

            // show downsampled image, but scale it so it appears the
            // same size as the original
            vx_buffer_t *vb = vx_world_get_buffer (state->vw, "image");
            vx_buffer_add_back (vb, vxo_pix_coords (VX_ORIGIN_TOP_LEFT,
                                                    vxo_chain (vxo_mat_scale (decimate),
                                                               vxo_mat_translate3 (0, -im->height, 0),
                                                               vo)));
            vx_buffer_swap (vb);
            image_u32_destroy (im);
        }
    }

  error:
    isrc->stop (isrc);
    printf ("exiting\n");
    return NULL;

}

static int
touch_event (vx_event_handler_t *vh, vx_layer_t *vl, vx_camera_pos_t *pos, vx_touch_event_t *mouse)
{
    return 0;
}

static int
mouse_event (vx_event_handler_t *vh, vx_layer_t *vl, vx_camera_pos_t *pos, vx_mouse_event_t *mouse)
{
    return 0;
}

static int
key_event (vx_event_handler_t *vh, vx_layer_t *vl, vx_key_event_t *key)
{
    state_t *state = vh->impl;

    static float rev_speed = REVERSE_SPEED1;
    static float fwd_speed = FORWARD_SPEED1;
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
        default:
            ;// silently ignore
    }

    pthread_mutex_lock (&state->mutex);
    {
        // default to zero
        state->cmd.motor_left_speed = state->cmd.motor_right_speed = 0.0;

        if (key_shift) { // speed boost
            fwd_speed = FORWARD_SPEED2;
            rev_speed = REVERSE_SPEED2;
        }
        else {
            fwd_speed = FORWARD_SPEED1;
            rev_speed = REVERSE_SPEED1;
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

    return 0;
}

static void
nodestroy (vx_event_handler_t *vh)
{
    // do nothing, since this event handler is statically allocated.
}

static state_t *global_state;
static void
handler (int signum)
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

static void *
send_cmds (void *data)
{
    state_t *state = data;

    while (state->running) {
        pthread_mutex_lock (&state->mutex);
        {
            //state->cmd.timestamp = utime_now();
            maebot_diff_drive_t_publish (state->lcm,  "MAEBOT_DIFF_DRIVE", &state->cmd);
        }
        pthread_mutex_unlock (&state->mutex);

        // send at 20 hz
        const int hz = 20;
        usleep (1000000/hz);
    }
    return NULL;
}

int
main (int argc, char *argv[])
{
    // so that redirected stdout won't be insanely buffered.
    setvbuf (stdout, (char *) NULL, _IONBF, 0);

    vx_global_init ();

    state_t *state = calloc (1, sizeof(*state));
    global_state = state;
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
    pthread_mutex_init (&state->mutex, NULL);

    state->layer_map = zhash_create (sizeof(vx_display_t*), sizeof(vx_layer_t*), zhash_ptr_hash, zhash_ptr_equals);

    signal (SIGINT, handler);

    getopt_add_bool (state->gopt, 'h', "help", 0, "Show this help");
    getopt_add_bool (state->gopt, 'v', "verbose", 0, "Show extra debugging output");
    getopt_add_int (state->gopt, 'l', "limitKBs", "-1", "Remote display bandwidth limit. < 0: unlimited.");
    getopt_add_int (state->gopt, 'd', "decimate", "1", "Decimate image by this amount before showing in vx");
    getopt_add_string (state->gopt, '\0', "url", "", "Camera URL");
    getopt_add_bool (state->gopt, '\0', "no-video", 0, "Disable video");

    if (!getopt_parse (state->gopt, argc, argv, 0) || getopt_get_bool(state->gopt,"help")) {
        getopt_do_usage (state->gopt);
        exit (EXIT_FAILURE);
    }

    if (1) {
        vx_buffer_t *vb = vx_world_get_buffer (state->vw, "text");
        vx_object_t *vt = vxo_text_create (VXO_TEXT_ANCHOR_TOP_RIGHT, "<<right,#0000ff>>Robot viewer!\n");
        vx_buffer_add_back (vb, vxo_pix_coords (VX_ORIGIN_TOP_RIGHT, vt));
        vx_buffer_swap (vb);
    }

    verbose = getopt_get_bool (state->gopt, "verbose");

    vx_remote_display_source_attr_t remote_attr;
    vx_remote_display_source_attr_init (&remote_attr);
    remote_attr.max_bandwidth_KBs = getopt_get_int (state->gopt, "limitKBs");
    remote_attr.advertise_name = "Maebot Teleop";
    vx_remote_display_source_t *remote = vx_remote_display_source_create_attr (&state->app, &remote_attr);

    pthread_create (&state->cmd_thread,  NULL, send_cmds, state);

    if (!getopt_get_bool (state->gopt, "no-video")) {
        // Set up the imagesource. This looks for a camera url specified on
        // the command line and, if none is found, enumerates a list of all
        // cameras imagesource can find and picks the first url it finds.
        if (strncmp (getopt_get_string (state->gopt, "url"), "", 1)) {
            state->url = strdup (getopt_get_string (state->gopt, "url"));
            printf ("URL: %s\n", state->url);
        }
        else {
            // No URL specified. Show all available and then use the first
            zarray_t *urls = image_source_enumerate ();
            printf ("Cameras:\n");
            for (int i = 0; i < zarray_size (urls); i++) {
                char *url;
                zarray_get (urls, i, &url);
                printf ("  %3d: %s\n", i, url);
            }

            if (0==zarray_size (urls)) {
                printf ("No cameras found.\n");
                exit (EXIT_FAILURE);
            }
            zarray_get (urls, 0, &state->url);
        }

        state->isrc = image_source_open (state->url);
        if (state->isrc == NULL) {
            printf ("Unable to open device %s\n", state->url);
            exit (EXIT_FAILURE);
        }

        image_source_t *isrc = state->isrc;

        if (isrc->start (isrc))
            exit (EXIT_FAILURE);
        run_camera (state);

        isrc->close (isrc);
    }
    else {
        while (state->running) {
            sleep(1);
        }
    }

    vx_remote_display_source_destroy (remote);
}
