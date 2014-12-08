#include <lcm/lcm.h>
#include <pthread.h>
#include <unistd.h>

#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>

#include "common/timestamp.h"
#include "lcmtypes/maebot_diff_drive_t.h"
#include "lcmtypes/maebot_motor_feedback_t.h"

#define CMD_PRD 50000 //us  -> 20Hz
#define MTR_SPD 0.4f
#define MTR_STOP 0.0f

maebot_diff_drive_t msg;
pthread_mutex_t msg_mutex;

void * lcm_handler (void *lcm) {
    for(;;) lcm_handle(lcm);
}
void * diff_drive_thread (void *arg) {
    lcm_t *lcm = lcm_create (NULL);

    uint64_t utime_start;
    while(1) {
        utime_start = utime_now ();

        pthread_mutex_lock (&msg_mutex);
        {
            msg.utime = utime_now ();
            maebot_diff_drive_t_publish (lcm, "MAEBOT_DIFF_DRIVE", &msg);
        }
        pthread_mutex_unlock (&msg_mutex);

        usleep (CMD_PRD - (utime_now() - utime_start));
    }

    return NULL;
}
static void motor_feedback_handler (const lcm_recv_buf_t *rbuf, 
        const char *channel, const maebot_motor_feedback_t *msg, void *user) {
    static int64_t lastTime = 0;
    static int lastLeftTicks  = 0;
    static int lastRightTicks = 0;
    uint64_t dt_us = msg->utime - lastTime;
    int dlt = msg->encoder_left_ticks  - lastLeftTicks;
    int drt = msg->encoder_right_ticks - lastRightTicks;
    float dt_f = dt_us/1000000.0;

    float leftRate  = dlt/(dt_f*4800);   // Datasheet says 48 ticks/cm
    float rightRate = drt/(dt_f*4800);   // Datasheet says 48 ticks/cm
    lastTime = msg->utime;
    lastLeftTicks = msg->encoder_left_ticks;
    lastRightTicks = msg->encoder_right_ticks;

    printf("encoder_[left, right]_ticks:\t\t%d,\t%d\n",
            msg->encoder_left_ticks, msg->encoder_right_ticks);
    printf("delta_[left, right]_ticks:\t\t%d,\t%d\n", dlt, drt);
    printf("motor_[left, right]_commanded_speed:\t%f,\t%f\n",
            msg->motor_left_commanded_speed, msg->motor_right_commanded_speed);
    printf("motor_[left, right]_actual_speed:\t%f,\t%f\n\n\n",
            leftRate, rightRate);
}

int main (int argc, char *argv[]) {
    // so that redirected stdout won't be insanely buffered.
    setvbuf (stdout, (char *) NULL, _IONBF, 0);

    if (pthread_mutex_init (&msg_mutex, NULL)) {
        printf ("mutex init failed\n");
        exit (EXIT_FAILURE);
    }

	lcm_t *lcm = lcm_create (NULL);
	if(!lcm) return 1;
    maebot_motor_feedback_t_subscribe(lcm, "MAEBOT_MOTOR_FEEDBACK",
                                       motor_feedback_handler, NULL);

    pthread_t lcm_thread;
    pthread_create (&lcm_thread, NULL, lcm_handler, lcm);


    // Init msg
    // no need for mutex here, as command thread hasn't started yet.
    pthread_mutex_lock (&msg_mutex);
    msg.motor_left_speed = MTR_STOP;
    msg.motor_right_speed = MTR_STOP;
    pthread_mutex_unlock (&msg_mutex);


    // Start sending motor commands
    pthread_t diff_drive_thread_pid;
    pthread_create (&diff_drive_thread_pid, NULL, diff_drive_thread, NULL);

    sleep(10);

    // forward
    pthread_mutex_lock (&msg_mutex);
    msg.motor_left_speed  = MTR_SPD*1.15;
    msg.motor_right_speed = MTR_SPD;
    pthread_mutex_unlock (&msg_mutex);

    usleep(2000000);

    pthread_mutex_lock (&msg_mutex);
    msg.motor_left_speed  = MTR_STOP;
    msg.motor_right_speed = MTR_STOP;
    pthread_mutex_unlock (&msg_mutex);

    usleep (200000);

    return EXIT_SUCCESS;
}
