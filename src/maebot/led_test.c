#include <stdint.h>
#include <unistd.h>
#include <lcm/lcm.h>

#include "common/timestamp.h"

#include "lcmtypes/maebot_leds_t.h"

int
main (int argc, char *argv[])
{
    // so that redirected stdout won't be insanely buffered.
    setvbuf (stdout, (char *) NULL, _IONBF, 0);

    lcm_t *lcm = lcm_create (NULL);
    maebot_leds_t msg;

    msg.bottom_led_left = 0;
    msg.bottom_led_middle = 0;
    msg.bottom_led_right = 0;
    msg.line_sensor_leds = 1;

    // Red and Left Bottom
    msg.top_rgb_led_left = 0x100000;
    msg.top_rgb_led_right = 0x100000;
    msg.bottom_led_left = 1;

    msg.utime = utime_now ();
    maebot_leds_t_publish (lcm, "MAEBOT_LEDS", &msg);

    usleep (1000000);

    // Green and Middle Bottom
    msg.top_rgb_led_left = 0x1000;
    msg.top_rgb_led_right = 0x1000;
    msg.bottom_led_left = 0;
    msg.bottom_led_middle = 1;

    msg.utime = utime_now ();
    maebot_leds_t_publish (lcm, "MAEBOT_LEDS", &msg);

    usleep (1000000);

    // Blue and Right Bottom
    msg.top_rgb_led_left = 0x10;
    msg.top_rgb_led_right = 0x10;
    msg.bottom_led_middle = 0;
    msg.bottom_led_right = 1;

    msg.utime = utime_now ();
    maebot_leds_t_publish (lcm, "MAEBOT_LEDS", &msg);

    usleep (1000000);

    // White and All Bottom
    msg.top_rgb_led_left = 0x101010;
    msg.top_rgb_led_right = 0x101010;
    msg.bottom_led_left = 1;
    msg.bottom_led_middle = 1;
    msg.bottom_led_right = 1;

    msg.utime = utime_now ();
    maebot_leds_t_publish (lcm, "MAEBOT_LEDS", &msg);

    usleep (1000000);

    // Off
    msg.top_rgb_led_left = 0x0;
    msg.top_rgb_led_right = 0x0;
    msg.bottom_led_left = 0;
    msg.bottom_led_middle = 0;
    msg.bottom_led_right = 0;
    msg.line_sensor_leds = 0;

    msg.utime = utime_now ();
    maebot_leds_t_publish (lcm, "MAEBOT_LEDS", &msg);

    usleep (1000000);

    // Blue
    msg.top_rgb_led_left = 0x10;
    msg.top_rgb_led_right = 0x10;
    msg.line_sensor_leds = 1;

    msg.utime = utime_now ();
    maebot_leds_t_publish (lcm, "MAEBOT_LEDS", &msg);

    return 0;
}
