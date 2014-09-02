#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>

#include <lcm/lcm.h>
#include "lcmtypes/maebot_sensor_data_t.h"


static void
sensor_data_handler (const lcm_recv_buf_t *rbuf, const char *channel,
                     const maebot_sensor_data_t *msg, void *user)
{
    int res = system ("clear");
    if (res)
        printf ("system clear failed\n");

    printf ("Subscribed to channel: MAEBOT_SENSOR_DATA\n");
    printf ("utime: %"PRId64"\n", msg->utime);
    printf ("accel[0, 1, 2]:        %d,\t%d,\t%d\n",
            msg->accel[0], msg->accel[1], msg->accel[2]);
    printf ("gyro[0, 1, 2]:         %d,\t%d,\t%d\n",
            msg->gyro[0], msg->gyro[1], msg->gyro[2]);
    printf ("gyro_int[0, 1, 2]:     %"PRId64",\t%"PRId64",\t%"PRId64"\n",
            msg->gyro_int[0], msg->gyro_int[1], msg->gyro_int[2]);
    printf ("line_sensors[0, 1, 2]: %d,\t%d,\t%d\n",
            msg->line_sensors[0], msg->line_sensors[1], msg->line_sensors[2]);
    printf ("range: %d\n", msg->range);
    printf ("user_button_pressed: %s\n", msg->user_button_pressed ? "true" : "false");
    printf ("power_button_pressed: %s\n", msg->power_button_pressed ? "true" : "false");

}

int
main (int argc, char *argv[])
{
    lcm_t *lcm = lcm_create (NULL);
    if(!lcm)
        return 1;

    maebot_sensor_data_t_subscribe (lcm,
                                    "MAEBOT_SENSOR_DATA",
                                    sensor_data_handler,
                                    NULL);

    while (1)
        lcm_handle (lcm);

    return 0;

}
