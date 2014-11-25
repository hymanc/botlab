#include <lcm/lcm.h>
#include <pthread.h>
#include <unistd.h>

#include "common/timestamp.h"
#include "lcmtypes/maebot_diff_drive_t.h"

#define CMD_PRD 50000 //us  -> 20Hz
#define MTR_SPD 0.25f
#define MTR_STOP 0.0f

maebot_diff_drive_t msg;
pthread_mutex_t msg_mutex;

void *
diff_drive_thread (void *arg)
{
    lcm_t *lcm = lcm_create (NULL);

    uint64_t utime_start;
    while(1) {
        utime_start = utime_now ();

        pthread_mutex_lock (&msg_mutex);
        maebot_diff_drive_t_publish (lcm, "MAEBOT_DIFF_DRIVE", &msg);
        pthread_mutex_unlock (&msg_mutex);

        usleep (CMD_PRD - (utime_now() - utime_start));
    }

    return NULL;
}

int
main (int argc, char *argv[])
{
    // so that redirected stdout won't be insanely buffered.
    setvbuf (stdout, (char *) NULL, _IONBF, 0);

    if (pthread_mutex_init (&msg_mutex, NULL)) {
        printf ("mutex init failed\n");
        exit (EXIT_FAILURE);
    }

    // Init msg
    // no need for mutex here, as command thread hasn't started yet.
    msg.motor_left_speed = MTR_STOP;
    msg.motor_right_speed = MTR_STOP;

    // Start sending motor commands
    pthread_t diff_drive_thread_pid;
    pthread_create (&diff_drive_thread_pid, NULL, diff_drive_thread, NULL);

    // forward
    pthread_mutex_lock (&msg_mutex);
    msg.motor_left_speed  = MTR_SPD;
    msg.motor_right_speed = MTR_SPD;
    pthread_mutex_unlock (&msg_mutex);

    usleep (500000);

    // reverse
    pthread_mutex_lock (&msg_mutex);
    msg.motor_left_speed  = -MTR_SPD;
    msg.motor_right_speed = -MTR_SPD;
    pthread_mutex_unlock (&msg_mutex);

    usleep (500000);

    // left turn
    pthread_mutex_lock (&msg_mutex);
    msg.motor_left_speed  = -MTR_SPD;
    msg.motor_right_speed = MTR_SPD;
    pthread_mutex_unlock (&msg_mutex);

    usleep (500000);

    // right turn
    pthread_mutex_lock (&msg_mutex);
    msg.motor_left_speed  = MTR_SPD;
    msg.motor_right_speed = -MTR_SPD;
    pthread_mutex_unlock (&msg_mutex);

    usleep (500000);

    // stop
    pthread_mutex_lock (&msg_mutex);
    msg.motor_left_speed  = MTR_STOP;
    msg.motor_right_speed = MTR_STOP;
    pthread_mutex_unlock (&msg_mutex);

    usleep (100000);

    return EXIT_SUCCESS;
}
