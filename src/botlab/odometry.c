#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include <math.h>

#include "common/getopt.h"
#include "math/gsl_util_matrix.h"
#include "math/gsl_util_blas.h"
#include "math/math_util.h"

#include "lcmtypes/maebot_motor_feedback_t.h"
#include "lcmtypes/maebot_sensor_data_t.h"
#include "lcmtypes/pose_xyt_t.h"

#include "xyt.h"

#define ALPHA_STRING          "1.0"  // longitudinal covariance scaling factor
#define BETA_STRING           "1.0"  // lateral side-slip covariance scaling factor
#define GYRO_RMS_STRING       "1.0"    // [deg/s]

typedef struct state state_t;
struct state 
{
    getopt_t *gopt;

    lcm_t *lcm;
    const char *odometry_channel;
    const char *feedback_channel;
    const char *sensor_channel;

    // odometry params
    double meters_per_tick; // conversion factor that translates encoder pulses into linear wheel displacement
    double alpha;
    double beta;
    double gyro_rms;

    int previous_left_encoder;
    int previous_right_encoder;
    
    bool use_gyro;
    int64_t dtheta_utime;
    double dtheta;
    double dtheta_sigma;

    double xyt[3]; // 3-dof pose
    double Sigma[3*3];
};

static void motor_feedback_handler (const lcm_recv_buf_t *rbuf, const char *channel, const maebot_motor_feedback_t *msg, void *user)
{
    state_t *state = user;
    
    // TODO: IMPLEMENT ME
    
    // Compute encoder differences (current "velocity")
    int l_diff = msg->encoder_left_ticks - state->previous_left_encoder;
    int r_diff = msg->encoder_right_ticks - state->previous_right_encoder;
    // Handle encoder wrap-around, probably not necessary?
    double dl = l_diff * state->meters_per_tick;
    double dr = r_diff * state->meters_per_tick;
    double ds = 0;

    // Current pose
    gsl_vector *p = gsl_vector_alloc(3);
    memcpy(p->data, state->xyt, 3*sizeof(double));
    
    // Compute new delta
    gsl_vector *delta = gsl_vector_alloc(3);
    gsl_vector_set(delta, 0, (dl + dr)/2);
    gsl_vector_set(delta, 1, ds);
    gsl_vector_set(delta, 2, (dr - dl)/2);
    
    // Next pose (pp = p (+) delta)
    gsl_vector *pp = gsl_vector_alloc(3);
    gsl_matrix *jplus = gsl_matrix_alloc(3,6);
    xyt_head2tail_gsl(pp, jplus, p, delta); // Compose delta onto p to get new estimate
   
    // Copy next pose into state
    memcpy(state->xyt, pp->data, 3*sizeof(double));
    
    // Update state covariance matrix 
    gsl_matrix *sig_p = gsl_matrix_alloc(3,3);
    gsl_matrix *sig_a = gsl_matrix_calloc(6,6);
    
    // Generate Sigma_delta matrix
    gsl_matrix *sig_delta = gsl_matrix_alloc(3,3);
    gsl_matrix_set(sig_delta, 0, 0, state->alpha * fabs(dl));
    gsl_matrix_set(sig_delta, 1, 1, state->alpha * fabs(dr));
    gsl_matrix_set(sig_delta, 2, 2, state->beta * fabs(dr + dl));
    
    gslu_matrix_set_submatrix(sig_a, 0, 0, sig_p);
    gslu_matrix_set_submatrix(sig_a, 3, 3, sig_delta);
    //J(+) * [SigP 0; 0 SigDelta] J(+)^T
    
    gsl_matrix * sig_temp = gslu_blas_mmT_alloc(sig_a,jplus);
    gsl_matrix * sig_pp = gslu_blas_mm_alloc(jplus, sig_temp);
    
    memcpy(state->sigma, sig_pp->data, sizeof(state->Sigma));
   
    // publish pose to LCM
    pose_xyt_t odo = { .utime = msg->utime };
    memcpy (odo.xyt, state->xyt, sizeof state->xyt);
    memcpy (odo.Sigma, state->Sigma, sizeof state->Sigma);
    pose_xyt_t_publish (state->lcm, state->odometry_channel, &odo);
    
    gslu_matrix_free(sig_p);
    gslu_matrix_free(sig_a);
    gslu_matrix_free(sig_delta);
    gslu_matrix_free(sig_temp);
    gslu_matrix_free(sig_pp);
    gslu_matrix_free(jplus);
    gslu_vector_free(p);
    gslu_vector_free(delta);
    gslu_vector_free(pp);
}

static void sensor_data_handler (const lcm_recv_buf_t *rbuf, const char *channel, const maebot_sensor_data_t *msg, void *user)
{
    state_t *state = user;

    if (!state->use_gyro)
        return;

    // TODO: IMPLEMENT ME
}

int main (int argc, char *argv[])
{
    // so that redirected stdout won't be insanely buffered.
    setvbuf (stdout, (char *) NULL, _IONBF, 0);

    state_t *state = calloc (1, sizeof *state);

    state->meters_per_tick = 2.0943951E-4;

    state->gopt = getopt_create ();
    getopt_add_bool   (state->gopt, 'h', "help", 0, "Show help");
    getopt_add_bool   (state->gopt, 'g', "use-gyro", 0, "Use gyro for heading instead of wheel encoders");
    getopt_add_string (state->gopt, '\0', "odometry-channel", "BOTLAB_ODOMETRY", "LCM channel name");
    getopt_add_string (state->gopt, '\0', "feedback-channel", "MAEBOT_MOTOR_FEEDBACK", "LCM channel name");
    getopt_add_string (state->gopt, '\0', "sensor-channel", "MAEBOT_SENSOR_DATA", "LCM channel name");
    getopt_add_double (state->gopt, '\0', "alpha", ALPHA_STRING, "Longitudinal covariance scaling factor");
    getopt_add_double (state->gopt, '\0', "beta", BETA_STRING, "Lateral side-slip covariance scaling factor");
    getopt_add_double (state->gopt, '\0', "gyro-rms", GYRO_RMS_STRING, "Gyro RMS deg/s");

    if (!getopt_parse (state->gopt, argc, argv, 1) || getopt_get_bool (state->gopt, "help")) {
        printf ("Usage: %s [--url=CAMERAURL] [other options]\n\n", argv[0]);
        getopt_do_usage (state->gopt);
        exit (EXIT_FAILURE);
    }

    state->use_gyro = getopt_get_bool (state->gopt, "use-gyro");
    state->odometry_channel = getopt_get_string (state->gopt, "odometry-channel");
    state->feedback_channel = getopt_get_string (state->gopt, "feedback-channel");
    state->sensor_channel = getopt_get_string (state->gopt, "sensor-channel");
    state->alpha = getopt_get_double (state->gopt, "alpha");
    state->beta = getopt_get_double (state->gopt, "beta");
    state->gyro_rms = getopt_get_double (state->gopt, "gyro-rms") * DTOR;

    // initialize LCM
    state->lcm = lcm_create (NULL);
    maebot_motor_feedback_t_subscribe (state->lcm, state->feedback_channel,
                                       motor_feedback_handler, state);
    maebot_sensor_data_t_subscribe (state->lcm, state->sensor_channel,
                                    sensor_data_handler, state);

    printf ("ticks per meter: %f\n", 1.0/state->meters_per_tick);

    while (1)
    {
        lcm_handle (state->lcm);
    }
}
