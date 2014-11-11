
#include <stdio.h>
#include <unistd.h>
#include <pthread.h>
#include <stdlib.h>
#include <lcm/lcm.h>
#include <signal.h>

#include "vx/vx.h"
#include "vx/vxo_drawables.h"
#include "vx/vx_remote_display_source.h"

#include "common/getopt.h"
#include "common/timestamp.h"
#include "imagesource/image_util.h"
#include "imagesource/image_source.h"
#include "imagesource/image_convert.h"

#include "lcmtypes/maebot_diff_drive_t.h"

#define MAX_REVERSE_SPEED -0.1f
#define MAX_FORWARD_SPEED 0.1f

typedef struct
{
    vx_application_t app;
    vx_event_handler_t veh;

    maebot_diff_drive_t cmd;
    pthread_mutex_t cmd_mutex;
    pthread_t cmd_thread;

    int running;

    getopt_t * gopt;
    char * url;
    image_source_t *isrc;
    int fidx;

    lcm_t * lcm;

    pthread_mutex_t layer_mutex;

    vx_world_t * vw;
    zhash_t *layer_map; // <display, layer>
} state_t;


static int verbose = 0;

static void display_finished(vx_application_t * app, vx_display_t * disp)
{
    state_t * state = app->impl;
    pthread_mutex_lock(&state->layer_mutex);

    vx_layer_t * layer = NULL;

    // store a reference to the world and layer that we associate with each vx_display_t
    zhash_remove(state->layer_map, &disp, NULL, &layer);

    vx_layer_destroy(layer);

    pthread_mutex_unlock(&state->layer_mutex);
}

static void display_started(vx_application_t * app, vx_display_t * disp)
{
    state_t * state = app->impl;

    vx_layer_t * layer = vx_layer_create(state->vw);
    vx_layer_set_display(layer, disp);
    vx_layer_add_event_handler(layer, &state->veh);

    pthread_mutex_lock(&state->layer_mutex);
    // store a reference to the world and layer that we associate with each vx_display_t
    zhash_put(state->layer_map, &disp, &layer, NULL, NULL);
    pthread_mutex_unlock(&state->layer_mutex);
}


void* run_camera(void * data)
{

    if (verbose) printf("Starting run_camera\n");

    state_t * state = data;
    image_source_t *isrc = state->isrc;

    while (state->running) {

        image_u32_t *im = NULL;

        {
            image_source_data_t isdata;

            int res = isrc->get_frame(isrc, &isdata);
            if (!res) {
                im = image_convert_u32(&isdata);
            }
            if (res)
                goto error;

            isrc->release_frame(isrc, &isdata);
        }

        if (verbose) printf("Got frame %p\n", im);
        if (im != NULL) {

            double decimate = getopt_get_double(state->gopt, "decimate");
            if (decimate != 1.0) {
                image_u32_t * im2 = image_util_u32_decimate(im, decimate);
                image_u32_destroy(im);
                im = im2;
            }

            vx_object_t * vo = vxo_image_from_u32(im, VXO_IMAGE_FLIPY, VX_TEX_MIN_FILTER);

            // show downsampled image, but scale it so it appears the
            // same size as the original
            vx_buffer_t *vb = vx_world_get_buffer(state->vw, "image");
            vx_buffer_add_back(vb, vxo_pix_coords(VX_ORIGIN_TOP_LEFT,
                                                  vxo_chain (vxo_mat_scale(decimate),
                                                             vxo_mat_translate3 (0, -im->height, 0),
                                                             vo)));
            vx_buffer_swap(vb);
        }

        image_u32_destroy(im);

    }

  error:
    isrc->stop(isrc);
    printf("exiting\n");
    return NULL;

}

static int touch_event (vx_event_handler_t * vh, vx_layer_t * vl, vx_camera_pos_t * pos, vx_touch_event_t * mouse)
{
    return 0;
}
static int mouse_event (vx_event_handler_t * vh, vx_layer_t * vl, vx_camera_pos_t * pos, vx_mouse_event_t * mouse)
{
    return 0;
}

static int key_event (vx_event_handler_t * vh, vx_layer_t * vl, vx_key_event_t * key)
{
    state_t *state = vh->impl;


    pthread_mutex_lock(&state->cmd_mutex);
    if (!key->released) {
        if (key->key_code == 'w' || key->key_code == 'W' || key->key_code == VX_KEY_UP) {
            // forward
            state->cmd.motor_left_speed = MAX_FORWARD_SPEED;
            state->cmd.motor_right_speed = MAX_FORWARD_SPEED;
        } else if (key->key_code == 'a' || key->key_code == 'A' || key->key_code == VX_KEY_LEFT) {
            // turn left
            state->cmd.motor_left_speed = MAX_REVERSE_SPEED;
            state->cmd.motor_right_speed = MAX_FORWARD_SPEED;

        } else if (key->key_code == 's' || key->key_code == 'S' || key->key_code == VX_KEY_DOWN) {
            // reverse
            state->cmd.motor_left_speed = MAX_REVERSE_SPEED;
            state->cmd.motor_right_speed = MAX_REVERSE_SPEED;
        } else if (key->key_code == 'd' || key->key_code == 'D' || key->key_code == VX_KEY_RIGHT) {
            // turn right
            state->cmd.motor_left_speed = MAX_REVERSE_SPEED;
            state->cmd.motor_right_speed = MAX_FORWARD_SPEED;
        }
    } else {
        // when key released, speeds default to 0
        state->cmd.motor_left_speed = 0;
        state->cmd.motor_right_speed = 0;
    }
    pthread_mutex_unlock(&state->cmd_mutex);

    return 0;
}

static void nodestroy (vx_event_handler_t * vh)
{
    // do nothing, since this event handler is statically allocated.
}

static state_t * global_state;
static void handler(int signum)
{
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

static void * send_cmds(void * data)
{
    state_t * state = data;

    while (state->running) {

        pthread_mutex_lock(&state->cmd_mutex);
        {
            //state->cmd.timestamp = utime_now();

            maebot_diff_drive_t_publish(state->lcm,  "MAEBOT_DIFF_DRIVE", &state->cmd);
        }
        pthread_mutex_unlock(&state->cmd_mutex);

        usleep(50000); // send at 20 hz
    }
    return NULL;
}

int main(int argc, char ** argv)
{
    vx_global_init();



    state_t * state = calloc(1, sizeof(state_t));
    global_state = state;
    state->gopt = getopt_create();
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
    state->lcm = lcm_create(NULL);
    state->vw = vx_world_create();
    pthread_mutex_init(&state->layer_mutex, NULL);
    pthread_mutex_init(&state->cmd_mutex, NULL);

    state->layer_map = zhash_create(sizeof(vx_display_t*), sizeof(vx_layer_t*), zhash_ptr_hash, zhash_ptr_equals);

    signal(SIGINT, handler);

    getopt_add_bool(state->gopt, 'h', "help", 0, "Show this help");
    getopt_add_bool(state->gopt, 'v', "verbose", 0, "Show extra debugging output");
    getopt_add_bool(state->gopt, '\0', "no-video", 0, "Disable video");
    getopt_add_int (state->gopt, 'l', "limitKBs", "-1", "Remote display bandwidth limit. < 0: unlimited.");
    getopt_add_int (state->gopt, 'd', "decimate", "1", "Decimate image by this amount before showing in vx");

    if (!getopt_parse(state->gopt, argc, argv, 0) ||
        getopt_get_bool(state->gopt,"help")) {
        getopt_do_usage(state->gopt);
        exit(-1);
    }

    if (0) {
        vx_object_t *vt = vxo_text_create(VXO_TEXT_ANCHOR_TOP_RIGHT, "<<right,#0000ff>>Robot viewer!\n");
        vx_buffer_t *vb = vx_world_get_buffer(state->vw, "text");
        vx_buffer_add_back(vb, vxo_pix_coords(VX_ORIGIN_TOP_RIGHT,vt));
        vx_buffer_swap(vb);
    }


    verbose = getopt_get_bool(state->gopt, "verbose");

    vx_remote_display_source_attr_t remote_attr;
    vx_remote_display_source_attr_init(&remote_attr);
    remote_attr.max_bandwidth_KBs = getopt_get_int(state->gopt, "limitKBs");
    remote_attr.advertise_name = "Maebot Teleop";

    vx_remote_display_source_t * remote = vx_remote_display_source_create_attr(&state->app, &remote_attr);


    pthread_create(&state->cmd_thread,  NULL, send_cmds, state);

    if (!getopt_get_bool(state->gopt, "no-video")) {
        const zarray_t *args = getopt_get_extra_args(state->gopt);
        if (zarray_size(args) > 0) {
            zarray_get(args, 0, &state->url);
        } else {
            zarray_t *urls = image_source_enumerate();

            printf("Cameras:\n");
            for (int i = 0; i < zarray_size(urls); i++) {
                char *url;
                zarray_get(urls, i, &url);
                printf("  %3d: %s\n", i, url);
            }

            if (zarray_size(urls) == 0) {
                printf("No cameras found.\n");
                exit(0);
            }
            zarray_get(urls, 0, &state->url);
        }



        state->isrc = image_source_open(state->url);
        if (state->isrc == NULL) {
            printf("Unable to open device %s\n", state->url);
            exit(-1);
        }

        image_source_t *isrc = state->isrc;

        if (isrc->start(isrc))
            exit(-1);
        run_camera(state);

        isrc->close(isrc);
    } else {
        while (1) sleep(1);
    }

    vx_remote_display_source_destroy(remote);
}
