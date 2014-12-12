#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>
#include <math.h>

#include <pthread.h>
#include <unistd.h>

#include <lcm/lcm.h>
#include "common/timestamp.h"
#include "lcmtypes/maebot_sensor_data_t.h"
#include "lcmtypes/maebot_processed_sensor_data_t.h"
#include "lcmtypes/maebot_motor_feedback_t.h"
#include "lcmtypes/pose_xyt_t.h"

#define D_FORM "% 12.7lf"

typedef struct maebot_shared_state maebot_shared_state_t;
struct maebot_shared_state {
    int running;
    int valid;
    maebot_sensor_data_t            sData;  // sensor data
    maebot_motor_feedback_t         mData;  // motor data
    maebot_processed_sensor_data_t  pData;  // processed sensor data
    pose_xyt_t                      oData;  // odometry data

    // some variables needed to keep track of theta
    int64_t startup_int[3];
    int64_t startup_time_sama5;
    int64_t startup_time;
};

pthread_mutex_t sensor_data_mutex;

void usleepClock(uint64_t delay, const int64_t *clock, pthread_mutex_t *mutex);
void waitForSensorUpdate(const int64_t *clock, pthread_mutex_t *mutex) {

    int64_t utime1, utime2;
    pthread_mutex_lock(  mutex);
    utime1 = utime2 = *clock;
    pthread_mutex_unlock(mutex);
    do {
        usleepClock(100000, clock, mutex);
        pthread_mutex_lock(  mutex);
        utime2 = *clock;
        pthread_mutex_unlock(mutex);
    } while(utime1 == utime2);
    return;
}
static void motor_feedback_handler(const lcm_recv_buf_t *rbuf,
        const char *channel, const maebot_motor_feedback_t *msg, void *user) {
    maebot_shared_state_t * state = user;
    pthread_mutex_lock(  &sensor_data_mutex);
    state->mData = *msg;
    pthread_mutex_unlock(&sensor_data_mutex);
}
static void sensor_data_handler(const lcm_recv_buf_t *rbuf,
        const char *channel, const maebot_sensor_data_t *msg, void *user) {
    maebot_shared_state_t * state = user;
    pthread_mutex_lock(  &sensor_data_mutex);
    state->sData = *msg;
    pthread_mutex_unlock(&sensor_data_mutex);
}
static void processed_sensor_data_handler(const lcm_recv_buf_t *rbuf,
        const char *channel, const maebot_processed_sensor_data_t *msg,
        void *user) {
    maebot_shared_state_t * state = user;
    pthread_mutex_lock(  &sensor_data_mutex);
    state->pData = *msg;
    pthread_mutex_unlock(&sensor_data_mutex);
}
static void pose_xyt_handler(const lcm_recv_buf_t *rbuf,
        const char *channel, const pose_xyt_t *msg, void *user) {
    maebot_shared_state_t * state = user;
    pthread_mutex_lock(  &sensor_data_mutex);
    state->oData = *msg;
    pthread_mutex_unlock(&sensor_data_mutex);
}
void initState(maebot_shared_state_t *state) {
    state->running              = 0;
    state->valid                = 0;

    state->sData.utime          = 0;
    state->sData.utime_sama5    = 0;

    state->mData.utime          = 0;
    state->mData.utime_sama5    = 0;

    //state->pData.utime          = 0;
    state->pData.utime_sama5    = 0;

    state->oData.utime          = 0;
    for(int i=0; i<3; i++) {
        state->startup_int[i] = 0;
    }
}
double gyroConv(int64_t data) {
    return ((double)data*250.0/(INT16_MAX*1000000.0));
}
void * plot_handler(void *user) {
    int64_t lastTime = 0, startTime = 0;
    maebot_shared_state_t          * state = user;
    maebot_sensor_data_t           * sData = &state->sData;
    pose_xyt_t                     * oData = &state->oData;
    maebot_processed_sensor_data_t * pData = &state->pData;

    pthread_mutex_lock(  &sensor_data_mutex);
    startTime = pData->utime_sama5;
    pthread_mutex_unlock(&sensor_data_mutex);

    while(state->running) {
        pthread_mutex_lock(&sensor_data_mutex);
        if(pData->utime_sama5 != lastTime) {

            // utime
            double time = (double)(pData->utime_sama5 - startTime)/1000000.0;
            printf(D_FORM",", time);

            // gyro[0, 1, 2]: uncorrected rate
            printf(D_FORM",",     
                   //gyroConv(data->gyro[0]*1000000),
                   //gyroConv(data->gyro[1]*1000000),
                   gyroConv(sData->gyro[2]*1000000));

            // gyro_int[0, 1, 2]: uncorrected gyro integral
            printf(D_FORM",",     
                   //gyroConv(data->gyro_int[0] - state->startup_int[0]), 
                   //gyroConv(data->gyro_int[1] - state->startup_int[1]), 
                   gyroConv(sData->gyro_int[2] - state->startup_int[2]));

            // gyro_bias[0, 1, 2]:  measured bias
            printf(D_FORM",",     
                   //gyroConv(state->gyroBias[0]),
                   //gyroConv(state->gyroBias[1]),
                   gyroConv(pData->gyroBias[2]));

            // gyro_corr[0, 1, 2]:  processed gyro rate
            printf(D_FORM",",     
                   //procData->gyro[0], 
                   //procData->gyro[1], 
                   pData->gyro[2]);

            // gyro_int[0, 1, 2]:   processed gyro integral
            printf(D_FORM",",     
                   //procData->gyro_int[0], 
                   //procData->gyro_int[1], 
                   pData->gyro_int[2]);

            // pose[x, y, t]:   estimated pose from wheel encoders
            printf(D_FORM","D_FORM","D_FORM",", 
                   oData->xyt[0],
                   oData->xyt[1],
                   oData->xyt[2]);

            printf("\b \b\n");  // prints new line and gets rid of last comma

            lastTime = pData->utime_sama5;
        }
        pthread_mutex_unlock(&sensor_data_mutex);
    }
    return NULL;
}
void * print_handler(void *user) {
    int64_t lastTime = 0;
    maebot_shared_state_t          * state = user;
    maebot_sensor_data_t           * sData = &state->sData;
    maebot_processed_sensor_data_t * pData = &state->pData;
    pose_xyt_t                     * oData = &state->oData;

    while(state->running) {
        sleep(1);
        pthread_mutex_lock(&sensor_data_mutex);
        if(pData->utime_sama5 != lastTime) {
            printf("\n\n");
            int res = system("clear");
            if(res) printf("system clear failed\n");
            printf("\n\n");

            printf("utime: %"PRId64"\n", pData->utime_sama5);
            printf("gyro[0, 1, 2]:           % 15.6lf,% 15.6lf,% 15.6lf\n",
                   gyroConv(sData->gyro[0]*1000000),
                   gyroConv(sData->gyro[1]*1000000),
                   gyroConv(sData->gyro[2]*1000000));
            printf("gyro_corrected[0, 1, 2]: % 15.6lf,% 15.6lf,% 15.6lf\n",
                   pData->gyro[0], 
                   pData->gyro[1], 
                   pData->gyro[2]);
            printf("gyro_bias[0, 1, 2]:      % 15.6lf,% 15.6lf,% 15.6lf\n",
                   gyroConv(pData->gyroBias[0]), 
                   gyroConv(pData->gyroBias[1]), 
                   gyroConv(pData->gyroBias[2]));
            printf("gyro_int_uncorr[0, 1, 2]:% 15.6lf,% 15.6lf,% 15.6lf\n",
                   gyroConv(sData->gyro_int[0] - state->startup_int[0]),
                   gyroConv(sData->gyro_int[1] - state->startup_int[1]),
                   gyroConv(sData->gyro_int[2] - state->startup_int[2]));
            printf("gyro_int[0, 1, 2]:       % 15.6lf,% 15.6lf,% 15.6lf\n",
                   pData->gyro_int[0], 
                   pData->gyro_int[1], 
                   pData->gyro_int[2]);
            printf("pose[x, y, t]"D_FORM","D_FORM","D_FORM"\n", 
                   oData->xyt[0],
                   oData->xyt[1],
                   oData->xyt[2]);
                   
            lastTime = pData->utime_sama5;
        }
        pthread_mutex_unlock(&sensor_data_mutex);
    }
    return NULL;
}
void * lcm_handler (void *lcm) {
    for(;;) lcm_handle(lcm);
}
int main (int argc, char *argv[]) {
    // so that redirected stdout won't be insanely buffered.
    setvbuf (stdout, (char *) NULL, _IONBF, 0);

    maebot_shared_state_t sharedState;
    initState(&sharedState);
    sharedState.running = 1;

    lcm_t *lcm = lcm_create(NULL);
    if(!lcm) return EXIT_FAILURE;

    maebot_sensor_data_t_subscribe(lcm, "MAEBOT_SENSOR_DATA",
            sensor_data_handler, &sharedState);

    maebot_processed_sensor_data_t_subscribe(lcm, "MAEBOT_PROCESSED_SENSOR_DATA",
            processed_sensor_data_handler, &sharedState);

    maebot_motor_feedback_t_subscribe(lcm, "MAEBOT_MOTOR_FEEDBACK",
            motor_feedback_handler, &sharedState);

    pose_xyt_t_subscribe(lcm, "BOTLAB_ODOMETRY",
            pose_xyt_handler, &sharedState);

    pthread_t lcm_thread;
    pthread_create(&lcm_thread, NULL, lcm_handler, lcm);

    // Wait for first sensor data to come in
    //printf("Waiting for a sensor datum to come in ... ");
    waitForSensorUpdate(&sharedState.sData.utime_sama5, &sensor_data_mutex);
    //printf("Done\n");

    //printf("Waiting for a odometry datum to come in ... ");
    waitForSensorUpdate(&sharedState.oData.utime,       &sensor_data_mutex);
    //printf("Done\n");
    
   // printf("Waiting for a processed sensor datum to come in ... ");
    waitForSensorUpdate(&sharedState.pData.utime_sama5, &sensor_data_mutex);
    //printf("Done\n");

    pthread_mutex_lock(  &sensor_data_mutex);
    for(int i=0; i<3; i++) {
        sharedState.startup_int[i] = sharedState.sData.gyro_int[i];
    }
    pthread_mutex_unlock(&sensor_data_mutex);


    //pthread_t print_thread;
    //pthread_create(&print_thread, NULL, print_handler, &sharedState);

    pthread_t plot_thread;  // data for plots and graphs
    pthread_create(&plot_thread, NULL, plot_handler, &sharedState);

    for(;;){;}

    // will probably never get here, 
    // but if for some reason I do, close gracefully

    sharedState.running = 0;

    usleep(10000);   // a little time to let buffers clear
    return EXIT_SUCCESS;
}
void usleepClock(uint64_t delay, const int64_t *clock, pthread_mutex_t *mutex) {
    pthread_mutex_lock(  mutex);
    int64_t startTime = *clock;
    pthread_mutex_unlock(mutex);
    int64_t stopTime = startTime;
    while(stopTime - startTime < delay) {
        pthread_mutex_lock(  mutex);
        stopTime = *clock;
        pthread_mutex_unlock(mutex);
    }
}
