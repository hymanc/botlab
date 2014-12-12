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

typedef struct maebot_shared_state maebot_shared_state_t;
struct maebot_shared_state {
    int running;
    maebot_sensor_data_t sensorData;
    maebot_motor_feedback_t motorFeedback;
    maebot_processed_sensor_data_t processedSensorData;
};

pthread_mutex_t sensor_data_mutex;

static void motor_feedback_handler(const lcm_recv_buf_t *rbuf,
        const char *channel, const maebot_motor_feedback_t *msg, void *user) {

    maebot_shared_state_t * state = user;

    pthread_mutex_lock(  &sensor_data_mutex);
    state->motorFeedback = *msg;
    pthread_mutex_unlock(&sensor_data_mutex);
}
static void sensor_data_handler(const lcm_recv_buf_t *rbuf,
        const char *channel, const maebot_sensor_data_t *msg, void *user) {

    maebot_shared_state_t * state = user;

    pthread_mutex_lock(  &sensor_data_mutex);
    state->sensorData = *msg;
    state->processedSensorData.utime_sama5 = msg->utime_sama5;
    //for(int i=0; i<3; i++) state->processedSensorData.gyro[i] = 
    //    msg->gyro[i] - state->processedSensorData.gyroBias[i];
    pthread_mutex_unlock(&sensor_data_mutex);
}

static void processed_sensor_data_handler(const lcm_recv_buf_t *rbuf,
        const char *channel, const maebot_processed_sensor_data_t *msg, 
        void *user) {

    maebot_shared_state_t * state = user;

    pthread_mutex_lock(  &sensor_data_mutex);
    state->processedSensorData = *msg;
    pthread_mutex_unlock(&sensor_data_mutex);
}
void initState(maebot_shared_state_t *state) {
    state->running                         = 0;
    state->processedSensorData.utime_sama5 = 0;
    for(int i=0; i<3; i++) {
        //state->processedSensorData.gyroBias[i] = 0;
        state->processedSensorData.gyro[i]     = 0;
    }
    return;
}
double gyroConv(int64_t data) {
    return ((double)data*250.0/(INT16_MAX*1000000.0));
}
double getGyroDelta(const int64_t integral, const double time, 
        const int64_t bias) {
    return gyroConv((double)integral - ((double)bias * time));
}
void * lcm_handler (void *lcm) {
    for(;;) lcm_handle(lcm);
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
int main (int argc, char *argv[]) {
    // so that redirected stdout won't be insanely buffered.
    setvbuf (stdout, (char *) NULL, _IONBF, 0);

    maebot_shared_state_t sharedState;
    initState(&sharedState);
    sharedState.running = 1;

    lcm_t *lcm = lcm_create (NULL);
    if (!lcm) return EXIT_FAILURE;

    maebot_motor_feedback_t_subscribe(lcm, "MAEBOT_MOTOR_FEEDBACK",
            motor_feedback_handler, &sharedState);

    maebot_sensor_data_t_subscribe(lcm, "MAEBOT_SENSOR_DATA",
            sensor_data_handler, &sharedState);

    maebot_processed_sensor_data_t_subscribe(lcm, 
            "MAEBOT_PROCESSED_SENSOR_DATA",
            processed_sensor_data_handler, &sharedState);

    pthread_t lcm_thread;
    pthread_create(&lcm_thread, NULL, lcm_handler, lcm);
    

    //int16_t bias[3];
    //int64_t stopTime  = 0;
    //int64_t deltaTime = 0;


    // Wait for first sensor data to come in
    pthread_mutex_lock(  &sensor_data_mutex);
    int64_t utime = sharedState.processedSensorData.utime_sama5;
    pthread_mutex_unlock(&sensor_data_mutex);
    while(utime == 0) {
        usleepClock(100000, &sharedState.sensorData.utime_sama5, 
                &sensor_data_mutex); // also good for time warped loggs
        pthread_mutex_lock(  &sensor_data_mutex);
        utime = sharedState.processedSensorData.utime_sama5;
        pthread_mutex_unlock(&sensor_data_mutex);
    }
    printf("Processed data detected waiting for bias to correct\n");

    usleepClock(10000000, &sharedState.sensorData.utime_sama5, 
                &sensor_data_mutex); // also good for time warped loggs

#define WHEEL_CONTROL 0
#if WHEEL_CONTROL
    printf("Waiting for wheels to start moving\n");
    pthread_mutex_lock(  &sensor_data_mutex);
    int64_t startTicks = sharedState.motorFeedback.encoder_left_ticks;
    pthread_mutex_unlock(&sensor_data_mutex);

    int64_t stopTicks = startTicks;

    while(stopTicks == startTicks) {
        pthread_mutex_lock(  &sensor_data_mutex);
        stopTicks = sharedState.motorFeedback.encoder_left_ticks;
        pthread_mutex_unlock(&sensor_data_mutex);
    }

    printf("Starting run\n");

    pthread_mutex_lock(  &sensor_data_mutex);
    int64_t startIntegral[3], startBias[3];
    int64_t startTime = sharedState.sensorData.utime_sama5;
    for(int i=0; i<3; i++) {
        startIntegral[i] = sharedState.sensorData.gyro_int[i];
        startBias[i]     = sharedState.processedSensorData.gyroBias[i];
    }
    pthread_mutex_unlock(&sensor_data_mutex);

    sleep(1);

    while(stopTicks != startTicks) {
        startTicks = stopTicks;
        usleepClock(100000, &sharedState.sensorData.utime_sama5, 
                &sensor_data_mutex); // also good for time warped loggs
        pthread_mutex_lock(  &sensor_data_mutex);
        stopTicks = sharedState.motorFeedback.encoder_left_ticks;
        pthread_mutex_unlock(&sensor_data_mutex);
    }

    pthread_mutex_lock(  &sensor_data_mutex);
    int64_t stopIntegral[3], stopBias[3];
    int64_t stopTime = sharedState.sensorData.utime_sama5;
    for(int i=0; i<3; i++) {
        stopIntegral[i] = sharedState.sensorData.gyro_int[i];
        stopBias[i]     = sharedState.processedSensorData.gyroBias[i];
    }
    pthread_mutex_unlock(&sensor_data_mutex);
    printf("Stopping run\n");

    int64_t integral = stopIntegral[2] - startIntegral[2];
    int64_t bias = (stopBias[2] + startBias[2])/2;
    double deltaT = ((double)(stopTime - startTime))/1000000.0; // conv to seconds
    double theta = getGyroDelta(integral, deltaT, bias);
    printf("Here is the new angle     % 15.3lf\n",theta);
    printf("integral             =% 15lld\n", integral);
    printf("bias                 =% 15lld\n",bias);
    printf("bias*t               =% 15lld\n",(int64_t)(bias*deltaT));
    printf("deltaT               =% 15lf\n",deltaT);

#else

    pthread_mutex_lock(  &sensor_data_mutex);
    int64_t startIntegral[3], startBias[3];
    int64_t startTime = sharedState.sensorData.utime_sama5;
    for(int i=0; i<3; i++) {
        startIntegral[i] = sharedState.sensorData.gyro_int[i];
        //startBias[i]     = sharedState.processedSensorData.gyroBias[i];
    }
    pthread_mutex_unlock(&sensor_data_mutex);

    for(;;) {
        usleepClock(100000, &sharedState.sensorData.utime_sama5, 
                &sensor_data_mutex); // also good for time warped loggs

        pthread_mutex_lock(  &sensor_data_mutex);
        int64_t stopIntegral[3], stopBias[3];
        int64_t stopTime = sharedState.sensorData.utime_sama5;
        for(int i=0; i<3; i++) {
            stopIntegral[i] = sharedState.sensorData.gyro_int[i];
            //stopBias[i]     = sharedState.processedSensorData.gyroBias[i];
        }
        pthread_mutex_unlock(&sensor_data_mutex);

        int64_t integral = stopIntegral[2] - startIntegral[2];
        int64_t bias = (stopBias[2] + startBias[2])/2;
        double deltaT = ((double)(stopTime - startTime))/1000000.0; // conv to seconds
        double theta = getGyroDelta(integral, deltaT, bias);
        printf("Here is the new angle     % 15.3lf\n",theta);
        printf("integral             =% 15lld\n", integral);
        printf("bias                 =% 15lld\n",bias);
        printf("bias*t               =% 15lld\n",(int64_t)(bias*deltaT));
        printf("deltaT               =% 15lf\n",deltaT);
        printf("\n\n\n");
    }
#endif



    sharedState.running = 0;

    usleep(10000);   // a little time to let buffers clear
    return EXIT_SUCCESS;
}
