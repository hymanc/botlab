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
    for(int i=0; i<3; i++) state->processedSensorData.gyro[i] = 
        //msg->gyro[i] - state->processedSensorData.gyroBias[i];
        msg->gyro[i];   // Not corrected.
    pthread_mutex_unlock(&sensor_data_mutex);
}

void initState(maebot_shared_state_t *state) {
    state->running                = 0;

    state->sensorData.utime       = 0;
    state->sensorData.utime_sama5 = 0;
    state->sensorData.range       = 0;
    for(int i=0; i<3; i++) {
        state->sensorData.accel[i]        = 0;
        state->sensorData.gyro[i]         = 0;
        state->sensorData.gyro_int[i]     = 0;
        state->sensorData.line_sensors[i] = 0;
        state->processedSensorData.gyroBias[i] = 0;
        state->processedSensorData.gyro[i] = 0;
    }

    state->motorFeedback.utime               = 0;
    state->motorFeedback.utime_sama5         = 0;
    state->motorFeedback.encoder_left_ticks  = 0;
    state->motorFeedback.encoder_right_ticks = 0;
}
double gyroConv(int16_t data) {
    return 250.0*(double)data/INT16_MAX;
}
void * process_handler(void *user) {
	lcm_t *lcmP = lcm_create(NULL);
	if (!lcmP) exit (EXIT_FAILURE);

    int64_t lastTime = 0;
    maebot_shared_state_t * state = user;
    maebot_sensor_data_t * data = &state->sensorData;
    maebot_processed_sensor_data_t * procData = &state->processedSensorData;

    while(state->running) {
        pthread_mutex_lock(&sensor_data_mutex);

        if(data->utime != lastTime) {
		    maebot_processed_sensor_data_t_publish(lcmP, 
                    "MAEBOT_PROCESSED_SENSOR_DATA", procData);
            lastTime = data->utime;
        }
        pthread_mutex_unlock(&sensor_data_mutex);
    }
    return NULL;
}
void * print_handler(void *user) {
    int64_t lastTime = 0;
    maebot_shared_state_t * state = user;
    maebot_sensor_data_t * data = &state->sensorData;
    maebot_processed_sensor_data_t * procData = &state->processedSensorData;

    while(state->running) {
        pthread_mutex_lock(&sensor_data_mutex);

        if(data->utime != lastTime) {
            int res = system("clear");
            if (res) printf("system clear failed\n");

            printf("utime: %"PRId64"\n", data->utime_sama5);
            printf("gyro[0, 1, 2]:           % 15.6lf,% 15.6lf,% 15.6lf\n",
                   gyroConv(data->gyro[0]),
                   gyroConv(data->gyro[1]),
                   gyroConv(data->gyro[2]));
            printf("gyro_corrected[0, 1, 2]: % 15.6lf,% 15.6lf,% 15.6lf\n",
                   gyroConv(data->gyro[0] - procData->gyroBias[0]), 
                   gyroConv(data->gyro[1] - procData->gyroBias[1]), 
                   gyroConv(data->gyro[2] - procData->gyroBias[2]));
            printf("gyro_bias[0, 1, 2]:      % 15lld,% 15lld,% 15lld\n",
                   procData->gyroBias[0], 
                   procData->gyroBias[1], 
                   procData->gyroBias[2]);
            printf("encoder ticks [l r]:     % 15d,% 15d\n",
                    state->motorFeedback.encoder_left_ticks,
                    state->motorFeedback.encoder_right_ticks);
            lastTime = data->utime;
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

    lcm_t *lcm = lcm_create (NULL);
    if (!lcm) return EXIT_FAILURE;

    maebot_sensor_data_t_subscribe(lcm, "MAEBOT_SENSOR_DATA",
            sensor_data_handler, &sharedState);

    maebot_motor_feedback_t_subscribe(lcm, "MAEBOT_MOTOR_FEEDBACK",
            motor_feedback_handler, &sharedState);

    pthread_t lcm_thread;
    pthread_create(&lcm_thread, NULL, lcm_handler, lcm);
    
    pthread_t process_thread;
    pthread_create(&process_thread, NULL, process_handler, &sharedState);

    //pthread_t print_thread;
    //pthread_create(&print_thread, NULL, print_handler, &sharedState);

    int64_t bias[3];
    int64_t startTime = 0;
    int64_t stopTime  = 0;
    int64_t deltaTime = 0;
    int64_t biasArr[10][3];

    // clear array
    for(int i=0; i<10; i++) for(int j=0; j<3; j++) biasArr[i][j] = 0;

    for(;;) {
        // Wait for first sensor data to come in
        pthread_mutex_lock(  &sensor_data_mutex);
        int64_t utime = sharedState.sensorData.utime_sama5;
        pthread_mutex_unlock(&sensor_data_mutex);
        while(utime == 0) {
            usleepClock(100000, &sharedState.sensorData.utime_sama5, 
                    &sensor_data_mutex);
            pthread_mutex_lock(  &sensor_data_mutex);
            utime = sharedState.sensorData.utime_sama5;
            pthread_mutex_unlock(&sensor_data_mutex);
        }

        // Get start state for static calibration
        pthread_mutex_lock(  &sensor_data_mutex);
        maebot_sensor_data_t    startSensorState   = sharedState.sensorData;
        maebot_motor_feedback_t startMotorFeedback = sharedState.motorFeedback;
        pthread_mutex_unlock(&sensor_data_mutex);

        //usleep(1000000);   // sleep for a while to gather gyro data
        usleepClock(1000000, &sharedState.sensorData.utime_sama5, 
                &sensor_data_mutex);

        // Get end state for static calibration
        pthread_mutex_lock(  &sensor_data_mutex);
        maebot_sensor_data_t    stopSensorState   = sharedState.sensorData;
        maebot_motor_feedback_t stopMotorFeedback = sharedState.motorFeedback;
        utime = sharedState.sensorData.utime_sama5;
        pthread_mutex_unlock(&sensor_data_mutex);

        startTime = startSensorState.utime_sama5;
        stopTime  = stopSensorState.utime_sama5;
        deltaTime = stopTime - startTime;
        if(deltaTime < 1000) {  // do not accept chunks of time less than 100ms
            //printf("Error: toss this calibration\n");
            continue; // Time roll over: ignore
        }
        if(stopMotorFeedback.encoder_left_ticks != 
                startMotorFeedback.encoder_left_ticks) {
            //printf("Error: toss this calibration\n");
            continue;
        }
        if(stopMotorFeedback.encoder_right_ticks != 
                startMotorFeedback.encoder_right_ticks) {
            //printf("Error: toss this calibration\n");
            continue;
        }
        //printf("Updating calibration Array\n");
        
        // If I get here, then the calibration succeded.
        double ddeltaTime = (double)deltaTime/1000000.0;    // conv to seconds
        for(int i = 0; i<3; i++) {
            int64_t startInt = startSensorState.gyro_int[i];
            int64_t stopInt  = stopSensorState.gyro_int[i];
            bias[i] = (int64_t)(((double)(stopInt - startInt) / ddeltaTime ));
            //bias[i] = stopInt - startInt;
        }
        //bias[0] = 1000000000;
        //bias[1] = -1000000000;
        //bias[2] = INT16_MAX+1;

        /*
        printf("\n");
        printf("start time = % lld\n", startSensorState.utime_sama5);
        printf("stop time  = % lld\n",  stopSensorState.utime_sama5);
        printf("delta time = % lld\n", deltaTime);
        for(int i = 0; i<3; i++) printf("bias[%2d]   = % d\n", i, bias[i]);
        */

        // shift the data down array
        // its only 27 elements so dynamic allocaton seems unessisary
        int64_t sum[3] = {0, 0, 0};
        for(int i=10-1; i>0; i--) {
            for(int j=0; j<3; j++) {
                sum[j] += biasArr[i][j] = biasArr[i-1][j];
            }
        }
        for(int j=0; j<3; j++) {
            sum[j] += biasArr[0][j] = bias[j];
            bias[j] = sum[j]/10;
        }

        pthread_mutex_lock(  &sensor_data_mutex);
        sharedState.processedSensorData.gyroBias[0] = bias[0];
        sharedState.processedSensorData.gyroBias[1] = bias[1];
        sharedState.processedSensorData.gyroBias[2] = bias[2];
        pthread_mutex_unlock(&sensor_data_mutex);
    }

    // will probably never get here, 
    // but if for some reason I do, close gracefully

    sharedState.running = 0;

    //pthread_join(print_thread, NULL);
    usleep(10000);   // a little time to let buffers clear
    return EXIT_SUCCESS;
}
