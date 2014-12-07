#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>
#include <math.h>

#include <pthread.h>
#include <unistd.h>

#include <lcm/lcm.h>
#include "common/timestamp.h"
#include "lcmtypes/maebot_sensor_data_t.h"

typedef struct maebot_shared_state maebot_shared_state_t;
struct maebot_shared_state {
    int running;
    maebot_sensor_data_t sensorData;
    int gyroBias[3];
};

pthread_mutex_t sensor_data_mutex;

static void sensor_data_handler (const lcm_recv_buf_t *rbuf,
        const char *channel, const maebot_sensor_data_t *msg, void *user) {

    maebot_shared_state_t * state = user;

    pthread_mutex_lock(  &sensor_data_mutex);
    state->sensorData = *msg;
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
        state->gyroBias[i] = 0;
    }
}
double gyroConv(int16_t data) {
    return 250.0*(double)data/INT16_MAX;
}
void * print_handler (void *user) {
    int64_t lastTime = 0;
    maebot_shared_state_t * state = user;
    maebot_sensor_data_t * data = &state->sensorData;

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
                   gyroConv(data->gyro[0]-state->gyroBias[0]), 
                   gyroConv(data->gyro[1]-state->gyroBias[1]), 
                   gyroConv(data->gyro[2]-state->gyroBias[2]));
            printf("gyro_bias[0, 1, 2]:      % 15d,% 15d,% 15d\n",
                   state->gyroBias[0], 
                   state->gyroBias[1], 
                   state->gyroBias[2]);
            lastTime = data->utime;
        }
        pthread_mutex_unlock(&sensor_data_mutex);
    }
    return NULL;
}
void * lcm_handler (void *lcm) {
    for(;;) lcm_handle(lcm);
}
void getGyroBias(uint64_t delay) {


    return;
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

    pthread_t lcm_thread;
    pthread_create(&lcm_thread, NULL, lcm_handler, lcm);
    
    pthread_t print_thread;
    pthread_create(&print_thread, NULL, print_handler, &sharedState);

    int16_t bias[3];
    int64_t startTime = 0;
    int64_t stopTime  = 0;
    int64_t deltaTime = 0;
    int16_t biasArr[10][3];

    // clear array
    for(int i=0; i<10; i++) for(int j=0; j<3; j++) biasArr[i][j] = 0;

    for(;;) {
        // Wait for first sensor data to come in
        pthread_mutex_lock(  &sensor_data_mutex);
        int64_t utime = sharedState.sensorData.utime_sama5;
        pthread_mutex_unlock(&sensor_data_mutex);
        while(utime == 0) {
            pthread_mutex_lock(  &sensor_data_mutex);
            utime = sharedState.sensorData.utime_sama5;
            pthread_mutex_unlock(&sensor_data_mutex);
        }

        // Get start state for static calibration
        pthread_mutex_lock(  &sensor_data_mutex);
        maebot_sensor_data_t startSensorState = sharedState.sensorData;
        pthread_mutex_unlock(&sensor_data_mutex);

        usleep(1000000);   // sleep for a while to gather gyro data

        // Get end state for static calibration
        pthread_mutex_lock(  &sensor_data_mutex);
        maebot_sensor_data_t stopSensorState = sharedState.sensorData;
        pthread_mutex_unlock(&sensor_data_mutex);

        pthread_mutex_lock(  &sensor_data_mutex);
        utime = sharedState.sensorData.utime_sama5;
        pthread_mutex_unlock(&sensor_data_mutex);

        bias[3];
        startTime = startSensorState.utime_sama5;
        stopTime  = stopSensorState.utime_sama5;
        deltaTime = stopTime - startTime;
        if(deltaTime < 0) continue; // Time roll over: ignore
        
        // If I get here, then the calibration succeded.
        for(int i = 0; i<3; i++) {
            int64_t startInt = startSensorState.gyro_int[i];
            int64_t stopInt  = stopSensorState.gyro_int[i];
            bias[i] = (int)(round((double)(stopInt - startInt) / (double)deltaTime));
        }

        /*
        printf("\n");
        printf("start time = % lld\n", startSensorState.utime_sama5);
        printf("stop time  = % lld\n",  stopSensorState.utime_sama5);
        printf("delta time = % lld\n", deltaTime);
        for(int i = 0; i<3; i++) printf("bias[%2d]   = % d\n", i, bias[i]);
        */

        // shift the data down array
        // its only 27 elements so dynamic allocaton seems unessisary
        int sum[3] = {0, 0, 0};
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
        sharedState.gyroBias[0] = bias[0];
        sharedState.gyroBias[1] = bias[1];
        sharedState.gyroBias[2] = bias[2];
        pthread_mutex_unlock(&sensor_data_mutex);
    }

    // will probably never get here, 
    // but if for some reason I do, close gracefully

    sharedState.running = 0;

    pthread_join(print_thread, NULL);
    usleep(10000);   // a little time to let buffers clear
    return EXIT_SUCCESS;
}
