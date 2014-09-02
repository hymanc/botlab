#include <stdio.h>
#include <unistd.h>
#include <pthread.h>
#include <stdlib.h>
#include <lcm/lcm.h>
#include <signal.h>
#include <math.h>

#include "vx/vx.h"
#include "vx/vxo_drawables.h"
#include "vx/vx_remote_display_source.h"

#include "common/getopt.h"
#include "common/timestamp.h"
#include "math/matd.h"
#include "math/math_util.h"
#include "imagesource/image_util.h"
#include "imagesource/image_source.h"
#include "imagesource/image_convert.h"

#include "lcmtypes/maebot_diff_drive_t.h"
#include "lcmtypes/maebot_laser_t.h"
#include "lcmtypes/maebot_leds_t.h"
#include "lcmtypes/maebot_sensor_data_t.h"
#include "lcmtypes/maebot_motor_feedback_t.h"

#define MAX_REVERSE_SPEED -1.0f
#define MAX_FORWARD_SPEED 1.0f

#define dmax(A,B) A < B ? B : A
#define dmin(A,B) A < B ? A : B

typedef struct state state_t;
struct state
{
    vx_application_t app;
    vx_event_handler_t veh;

    double joy_bounds;
    double last_click[3];

    maebot_diff_drive_t cmd;
    pthread_mutex_t cmd_mutex;
    pthread_t cmd_thread;
    pthread_mutex_t render_mutex;
    pthread_t render_thread;


    int running;

    getopt_t *gopt;
    char *url;
    image_source_t *isrc;
    int fidx;

    lcm_t *lcm;
    pthread_mutex_t lcm_mutex;

    pthread_mutex_t layer_mutex;

    vx_world_t *vw;
    zhash_t *layer_map; // <display, layer>
};

static int verbose = 0;

static void display_finished(vx_application_t *app, vx_display_t *disp)
{
    state_t *state = app->impl;
    pthread_mutex_lock(&state->layer_mutex);

    vx_layer_t *layer = NULL;

    zhash_remove(state->layer_map, &disp, NULL, &layer);

    vx_layer_destroy(layer);
    pthread_mutex_unlock(&state->layer_mutex);
}

static void display_started(vx_application_t *app, vx_display_t *disp)
{
    state_t *state = app->impl;

    vx_layer_t *layer = vx_layer_create(state->vw);
    vx_layer_set_display(layer, disp);
    vx_layer_add_event_handler(layer, &state->veh);

    pthread_mutex_lock(&state->layer_mutex);
    zhash_put(state->layer_map, &disp, &layer, NULL, NULL);
    pthread_mutex_unlock(&state->layer_mutex);
}

static int touch_event (vx_event_handler_t * vh, vx_layer_t * vl, vx_camera_pos_t * pos, vx_touch_event_t * mouse)
{
    return 0;
}
static int mouse_event (vx_event_handler_t * vh, vx_layer_t * vl, vx_camera_pos_t * pos, vx_mouse_event_t * mouse)
{
    state_t *state = vh->impl;

    // Button state
    int m1 = mouse->button_mask & VX_BUTTON1_MASK;
//    int alt = mouse->modifiers & VX_ALT_MASK;
    int ctrl = mouse->modifiers & VX_CTRL_MASK;
//    int shift = mouse->modifiers & VX_SHIFT_MASK;

    pthread_mutex_lock(&state->cmd_mutex);

    if (m1 && ctrl) {
        // Ray cast to find click point
        vx_ray3_t ray;
        vx_camera_pos_compute_ray(pos, mouse->x, mouse->y, &ray);
        vx_ray3_intersect_xy(&ray, 0.0, state->last_click);
    } else {
        state->last_click[0] = 0;
        state->last_click[1] = 0;
        state->last_click[2] = 0;
    }

    pthread_mutex_unlock(&state->cmd_mutex);

    return 0;
}

static int key_event (vx_event_handler_t * vh, vx_layer_t * vl, vx_key_event_t * key)
{
    return 0;
}

static void nodestroy (vx_event_handler_t * vh)
{
    // do nothing, since this event handler is statically allocated.
}

static state_t *global_state;
// XXX This is not working correctly right now
/*
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
*/

// This thread continuously publishes command messages to the maebot
static void* send_cmds(void *data)
{
    state_t *state = data;
    uint32_t Hz = 20;

    while (state->running) {
        pthread_mutex_lock(&state->cmd_mutex);
        matd_t *click = matd_create_data(3, 1, state->last_click);
        double mag = matd_vec_mag(click);
        matd_t *n = click;
        if (mag != 0) {
            n = matd_vec_normalize(click);  // Leaks memory
        }
        double len = dmin(mag, state->joy_bounds);

        // Map vector direction to motor command.
        state->cmd.utime = utime_now();

        int sign_x = matd_get(n, 0, 0) >= 0; // > 0 if positive
        int sign_y = matd_get(n, 1, 0) >= 0; // > 0 if positive
        float magx = fabs(matd_get(n, 0, 0));
        float magy = fabs(matd_get(n, 1, 0));
        float x2y = magx > 0 ? (magx-magy)/magx : 0.0f;
        float y2x = magy > 0 ? (magy-magx)/magy : 0.0f;
        float scale = 1.0f*len/state->joy_bounds;

        // Quadrant check
        if (sign_y && sign_x) {
            // Quad I
            state->cmd.motor_left_speed = MAX_FORWARD_SPEED*scale;
            if (magx > magy) {
                state->cmd.motor_right_speed = MAX_REVERSE_SPEED*scale*x2y;
            } else {
                state->cmd.motor_right_speed = MAX_FORWARD_SPEED*scale*y2x;
            }
        } else if (sign_y && !sign_x) {
            // Quad II
            state->cmd.motor_right_speed = MAX_FORWARD_SPEED*scale;
            if (magx > magy) {
                state->cmd.motor_left_speed = MAX_REVERSE_SPEED*scale*x2y;
            } else {
                state->cmd.motor_left_speed = MAX_FORWARD_SPEED*scale*y2x;
            }
        } else if (!sign_y && !sign_x) {
            // Quad III
            state->cmd.motor_left_speed = MAX_REVERSE_SPEED*scale;
            if (magx > magy) {
                state->cmd.motor_right_speed = MAX_FORWARD_SPEED*scale*x2y;
            } else {
                state->cmd.motor_right_speed = MAX_REVERSE_SPEED*scale*y2x;
            }
        } else {
            // Quad IV
            state->cmd.motor_right_speed = MAX_REVERSE_SPEED*scale;
            if (magx > magy) {
                state->cmd.motor_left_speed = MAX_FORWARD_SPEED*scale*x2y;
            } else {
                state->cmd.motor_left_speed = MAX_REVERSE_SPEED*scale*y2x;
            }
        }

        if (mag != 0) {
            matd_destroy(n);
        }
        matd_destroy(click);

        // Publish
        maebot_diff_drive_t_publish(state->lcm, "MAEBOT_DIFF_DRIVE", &(state->cmd));

        pthread_mutex_unlock(&state->cmd_mutex);

        usleep(1000000/Hz);
    }

    return NULL;
}

// This thread continously renders updates from the robot
static void* render_loop(void *data)
{
    state_t *state = data;

    int fps = 30;

    // Grid
    vx_buffer_add_back(vx_world_get_buffer(state->vw, "grid"),
                       vxo_grid());
    vx_buffer_swap(vx_world_get_buffer(state->vw, "grid"));

    // Joystick circle
    vx_buffer_add_back(vx_world_get_buffer(state->vw, "bounds"),
                       vxo_chain(vxo_mat_scale(state->joy_bounds),
                                 vxo_circle(vxo_lines_style(vx_blue, 2.0f))));
    vx_buffer_swap(vx_world_get_buffer(state->vw, "bounds"));

    while (state->running) {
        float line[6];
        line[0] = 0;
        line[1] = 0;
        line[2] = 0;

        // Scale click line
        pthread_mutex_lock(&state->cmd_mutex);

        matd_t *click = matd_create_data(3, 1, state->last_click);
        double mag = matd_vec_mag(click);
        matd_t *n = click;
        if (mag != 0) {
            n = matd_vec_normalize(click);
        }
        double len = dmin(mag, state->joy_bounds);

        line[3] = (float)len*matd_get(n, 0, 0);
        line[4] = (float)len*matd_get(n, 1, 0);
        line[5] = (float)len*matd_get(n, 2, 0);

        if (mag != 0) {
            matd_destroy(n);
        }
        matd_destroy(click);

        vx_buffer_add_back(vx_world_get_buffer(state->vw, "direction"),
                           vxo_lines(vx_resc_copyf(line, 6),
                                     2,
                                     GL_LINES,
                                     vxo_lines_style(vx_red, 3.0f)));
        vx_buffer_swap(vx_world_get_buffer(state->vw, "direction"));

        pthread_mutex_unlock(&state->cmd_mutex);

        usleep(1000000/fps);
    }

    return NULL;
}

// === LCM Handlers =================
static void motor_feedback_handler(const lcm_recv_buf_t *rbuf,
                                   const char* channel,
                                   const maebot_motor_feedback_t* msg,
                                   void* user)
{
}

static void sensor_data_handler(const lcm_recv_buf_t *rbuf,
                                const char* channel,
                                const maebot_sensor_data_t* msg,
                                void* user)
{
}


int main(int argc, char **argv)
{
    vx_global_init();

    // === State initialization ============================
    state_t *state = calloc(1, sizeof(state_t));
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
    state->last_click[0] = 0;
    state->last_click[1] = 0;
    state->last_click[2] = 0;
    state->joy_bounds = 10.0;

    state->running = 1;
    state->lcm = lcm_create(NULL);
    state->vw = vx_world_create();
    pthread_mutex_init(&state->layer_mutex, NULL);
    pthread_mutex_init(&state->cmd_mutex, NULL);
    pthread_mutex_init(&state->lcm_mutex, NULL);
    pthread_mutex_init(&state->render_mutex, NULL);

    state->layer_map = zhash_create(sizeof(vx_display_t*), sizeof(vx_layer_t*), zhash_ptr_hash, zhash_ptr_equals);
    // === End =============================================

    // Clean up on Ctrl+C
    //signal(SIGINT, handler);

    getopt_add_bool(state->gopt, 'h', "help", 0, "Show this help");
    getopt_add_bool(state->gopt, 'v', "verbose", 0, "Show extra debugging info");
    getopt_add_int(state->gopt, 'l', "limitKBs", "-1", "Remote display bandwith limit in KBs. < 0: unlimited.");
    getopt_add_int(state->gopt, 'p', "port", "15151", "Vx display port");

    if (!getopt_parse(state->gopt, argc, argv, 0) ||
        getopt_get_bool(state->gopt, "help"))
    {
        getopt_do_usage(state->gopt);
        exit(-1);
    }

    // Set up display
    verbose = getopt_get_bool(state->gopt, "verbose");

    vx_remote_display_source_attr_t remote_attr;
    vx_remote_display_source_attr_init(&remote_attr);
    remote_attr.max_bandwidth_KBs = getopt_get_int(state->gopt, "limitKBs");
    remote_attr.advertise_name = "Maebot Teleop";
    remote_attr.connection_port = getopt_get_int(state->gopt, "port");

    vx_remote_display_source_t *remote = vx_remote_display_source_create_attr(&state->app, &remote_attr);

    // Video stuff?

    // LCM subscriptions
    maebot_motor_feedback_t_subscribe(state->lcm,
                                        "MAEBOT_MOTOR_FEEDBACK",
                                        motor_feedback_handler,
                                        state);
    maebot_sensor_data_t_subscribe(state->lcm,
                                   "MAEBOT_SENSOR_DATA",
                                   sensor_data_handler,
                                   state);


    // Spin up thread(s)
    pthread_create(&state->cmd_thread, NULL, send_cmds, state);
    pthread_create(&state->render_thread, NULL, render_loop, state);

    // Loop forever
    while (1) lcm_handle(state->lcm);

    vx_remote_display_source_destroy(remote);
}
