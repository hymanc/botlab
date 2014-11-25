#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include "lcmtypes/maebot_laser_t.h"

#define NUM_BLINKS 3

int
main (int argc, char *argv[])
{
    // so that redirected stdout won't be insanely buffered.
    setvbuf (stdout, (char *) NULL, _IONBF, 0);

    lcm_t *lcm = lcm_create (NULL);
    maebot_laser_t msg;

    msg.laser_power = 1;
    maebot_laser_t_publish (lcm, "MAEBOT_LASER", &msg);
    usleep (250000);

    for (int i = 1; i < NUM_BLINKS; i++) {
        msg.laser_power = 0;
        maebot_laser_t_publish (lcm, "MAEBOT_LASER", &msg);
        usleep (250000);

        msg.laser_power = 1;
        maebot_laser_t_publish (lcm, "MAEBOT_LASER", &msg);
        usleep (250000);
    }

    msg.laser_power = 0;
    maebot_laser_t_publish (lcm, "MAEBOT_LASER", &msg);

    return 0;
}
