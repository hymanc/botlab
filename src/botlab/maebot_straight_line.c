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
#include "imagesource/image_util.h"
#include "imagesource/image_source.h"
#include "imagesource/image_convert.h"

#include "lcmtypes/maebot_diff_drive_t.h"
#include "lcmtypes/maebot_laser_t.h"
#include "lcmtypes/maebot_leds_t.h"
#include "lcmtypes/maebot_sensor_data_t.h"
#include "lcmtypes/maebot_motor_feedback_t.h"

#define dmax(A,B) A < B ? B : A
#define dmin(A,B) A < B ? A : B

#define FORWARD_SPEED_RIGHT 0.25f
#define FORWARD_SPEED_LEFT  0.25f
#define TIMETOMOVE 1.0

#define MAX_REVERSE_SPEED -0.35f
#define MAX_FORWARD_SPEED  0.45f

typedef struct state state_t;
struct state {
    vx_application_t app;
    vx_event_handler_t veh;

    double joy_bounds;
    double last_click[3];

    maebot_diff_drive_t cmd;
    pthread_t cmd_thread;
    pthread_mutex_t cmd_mutex;

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

static void destroy (vx_event_handler_t *vh) {
    // do nothing, since this event handler is statically allocated.
}

static state_t *global_state;

// This thread continuously publishes command messages to the maebot
static void * send_cmds (void *data) {
    state_t *state = data;
    const uint32_t Hz = 20;

    while (state->running) {
        pthread_mutex_lock (&state->cmd_mutex);
        {
            matd_t *click = matd_create_data (3, 1, state->last_click);
            double mag = matd_vec_mag (click);
            matd_t *n = click;
            if (mag != 0)
                n = matd_vec_normalize (click);  // Leaks memory
            double len = dmin (mag, state->joy_bounds);

            // Map vector direction to motor command.
            state->cmd.utime = utime_now ();

            int sign_x = matd_get (n, 0, 0) >= 0; // > 0 if positive
            int sign_y = matd_get (n, 1, 0) >= 0; // > 0 if positive
            float magx = fabs (matd_get (n, 0, 0));
            float magy = fabs (matd_get (n, 1, 0));
            float x2y = magx > 0 ? (magx-magy)/magx : 0.0f;
            float y2x = magy > 0 ? (magy-magx)/magy : 0.0f;
            float scale = 1.0f*len/state->joy_bounds;

            // Quadrant check
            if (sign_y && sign_x) {
                // Quad I
                state->cmd.motor_left_speed = MAX_FORWARD_SPEED*scale;
                if (magx > magy)
                    state->cmd.motor_right_speed = MAX_REVERSE_SPEED*scale*x2y;
                else
                    state->cmd.motor_right_speed = MAX_FORWARD_SPEED*scale*y2x;
            }
            else if (sign_y && !sign_x) {
                // Quad II
                state->cmd.motor_right_speed = MAX_FORWARD_SPEED*scale;
                if (magx > magy)
                    state->cmd.motor_left_speed = MAX_REVERSE_SPEED*scale*x2y;
                else
                    state->cmd.motor_left_speed = MAX_FORWARD_SPEED*scale*y2x;
            }
            else if (!sign_y && !sign_x) {
                // Quad III
                state->cmd.motor_left_speed = MAX_REVERSE_SPEED*scale;
                if (magx > magy)
                    state->cmd.motor_right_speed = MAX_FORWARD_SPEED*scale*x2y;
                else
                    state->cmd.motor_right_speed = MAX_REVERSE_SPEED*scale*y2x;
            }
            else {
                // Quad IV
                state->cmd.motor_right_speed = MAX_REVERSE_SPEED*scale;
                if (magx > magy)
                    state->cmd.motor_left_speed = MAX_FORWARD_SPEED*scale*x2y;
                else
                    state->cmd.motor_left_speed = MAX_REVERSE_SPEED*scale*y2x;
            }
            if (mag != 0)
                matd_destroy (n);
            matd_destroy (click);

            // Publish
            maebot_diff_drive_t_publish (state->lcm, "MAEBOT_DIFF_DRIVE", &(state->cmd));
        }
        pthread_mutex_unlock (&state->cmd_mutex);
        usleep (1000000/Hz);
    }
    return NULL;
}
static void * demo_path(void *data) {
    state_t *state = data;

    float fwd_speed_l = FORWARD_SPEED_LEFT;
    float fwd_speed_r = FORWARD_SPEED_RIGHT;

    // send a drive forward command
    pthread_mutex_lock(  &state->cmd_mutex);
    state->cmd.motor_left_speed  = fwd_speed_l;
    state->cmd.motor_right_speed = fwd_speed_r;
    pthread_mutex_unlock(&state->cmd_mutex);
    printf("Moving forward for a while\n");

    // sleep for a while
    usleep(1000000*TIMETOMOVE);

    // stop the wheels
    pthread_mutex_lock(  &state->cmd_mutex);
    state->cmd.motor_left_speed  = 0.0;
    state->cmd.motor_right_speed = 0.0;
    pthread_mutex_unlock(&state->cmd_mutex);
    printf("Stopping the maebot\n");

    return NULL;
}

// === LCM Handlers =================
static void motor_feedback_handler (const lcm_recv_buf_t *rbuf, const char* channel,
                        const maebot_motor_feedback_t* msg, void* user) {
}

static void sensor_data_handler (const lcm_recv_buf_t *rbuf, const char* channel,
                     const maebot_sensor_data_t* msg, void* user) {
}


int main(int argc, char *argv[]) {
    // so that redirected stdout won't be insanely buffered.
    setvbuf (stdout, (char *) NULL, _IONBF, 0);

    // === State initialization ============================
    state_t *state              = calloc(1, sizeof (*state));
    global_state                = state;   // TODO can this not be global?
    state->gopt                 = getopt_create();
    state->app.impl             = state;
    state->veh.dispatch_order   = -10;
    state->veh.destroy          = destroy;
    state->veh.impl             = state;
    state->last_click[0]        = 0;
    state->last_click[1]        = 0;
    state->last_click[2]        = 0;
    state->joy_bounds           = 10.0;
    state->running              = 1;
    state->lcm                  = lcm_create (NULL);
    state->vw                   = vx_world_create ();
    pthread_mutex_init (&state->layer_mutex, NULL);
    pthread_mutex_init (&state->cmd_mutex, NULL);
    pthread_mutex_init (&state->lcm_mutex, NULL);
    pthread_mutex_init (&state->render_mutex, NULL);

    // === End =============================================

    // LCM subscriptions
    maebot_motor_feedback_t_subscribe(state->lcm, "MAEBOT_MOTOR_FEEDBACK",
                                       motor_feedback_handler, state);
    maebot_sensor_data_t_subscribe(state->lcm, "MAEBOT_SENSOR_DATA",
                                    sensor_data_handler, state);

    pthread_t demo_thread;
    // Spin up thread(s)
    pthread_create(&state->cmd_thread, NULL, send_cmds, state);
    //pthread_create(&demo_thread, NULL, demo_path, state);

    // Loop forever
    demo_path(state);

    //for(;;)
    //    lcm_handle (state->lcm);
    return 0;
}
