#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>

#include <lcm/lcm.h>
#include "lcmtypes/maebot_motor_feedback_t.h"

static void
motor_feedback_handler (const lcm_recv_buf_t *rbuf, const char *channel,
                        const maebot_motor_feedback_t *msg, void *user)
{
    int res = system ("clear");
    if (res)
        printf ("system clear failed\n");

    printf ("Subscribed to channel: %s\n", channel);
    printf ("utime: %"PRId64"\n", msg->utime);
    printf ("encoder_[left, right]_ticks:\t\t%d,\t%d\n",
            msg->encoder_left_ticks, msg->encoder_right_ticks);
    printf ("motor_current[left, right]:\t\t%d,\t%d\n",
            msg->motor_current_left, msg->motor_current_right);
    printf ("motor_[left, right]_commanded_speed:\t%f,\t%f\n",
            msg->motor_left_commanded_speed, msg->motor_right_commanded_speed);
    printf ("motor_[left, right]_actual_speed:\t%f,\t%f\n",
            msg->motor_left_commanded_speed, msg->motor_right_commanded_speed);
}

int
main (int argc, char *argv[])
{
	lcm_t *lcm = lcm_create (NULL);
	if(!lcm)
		return 1;

    printf ("utime,\t\tleft_ticks,\tright_ticks\n");

    maebot_motor_feedback_t_subscribe (lcm,
                                       "MAEBOT_MOTOR_FEEDBACK",
                                       motor_feedback_handler,
                                       NULL);

    while (1)
        lcm_handle (lcm);

    return 0;
}
