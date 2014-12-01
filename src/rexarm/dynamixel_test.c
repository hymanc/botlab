#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>

#include "common/getopt.h"
#include "math/math_util.h"

#include "dynamixel_serial_bus.h"
#include "dynamixel_device.h"

int
main(int argc, char *argv[])
{
    // so that redirected stdout won't be insanely buffered.
    setvbuf (stdout, (char *) NULL, _IONBF, 0);

    getopt_t *gopt = getopt_create();
    getopt_add_bool(gopt, 'h', "help", 0, "Show this help screen");
    getopt_add_string(gopt, 'd', "device", "/dev/ttyUSB0", "USB Dynamixel device path");
    getopt_add_int(gopt, '\0', "baud", "1000000", "Set the servo's baud rate");
    getopt_add_bool(gopt, '\0', "home-all", 0, "Command all servos on bus to zero degrees");
    getopt_add_bool(gopt, '\0', "idle-all", 0, "Command all servos on bus to idle");

    getopt_add_spacer(gopt, "---------------");
    getopt_add_int(gopt, 'i', "id", "-1", "Execute a command for a specific servo");
    getopt_add_bool(gopt, '\0', "home", 0, "Command the servo to zero degrees");
    getopt_add_bool(gopt, '\0', "idle", 0, "Command the servo to idle");
    getopt_add_int(gopt, '\0', "set-id", "0", "Set the servo's ID");
    getopt_add_int(gopt, '\0', "set-baud", "0", "Set the servo's baud rate");
    getopt_add_double(gopt, '\0', "set-degrees", "0", "Set the servo's position");

    getopt_add_spacer(gopt, "----------------");
    getopt_add_double(gopt, '\0', "speed", ".5", "Speed setting for interactive mode");
    getopt_add_double(gopt, '\0', "torque", ".5", "Torque setting for interactive mode");
    getopt_add_int(gopt, '\0', "max-id", "6", "Maximum servo ID to poll over");

    if (!getopt_parse(gopt, argc, argv, 1) || getopt_get_bool(gopt, "help")) {
        getopt_do_usage(gopt);
        exit(-1);
    }

    // By default, just pings all the servos
    dynamixel_bus_t *bus;
    const char *devname = getopt_get_string(gopt, "device");
    int baud = getopt_get_int(gopt, "baud");

    // Only supports serial bus for now.
    printf("Opening device %s with baud %d\n", devname, baud);
    bus = serial_bus_create(devname, baud);

    if (getopt_get_bool(gopt, "home-all")) {
        for (int id = 0; id < getopt_get_int(gopt, "max-id"); id++) {
            dynamixel_device_t *servo = bus->get_servo(bus, id);
            if (servo == NULL)
                continue;
            servo->set_goal(servo,
                            0,
                            getopt_get_double(gopt, "speed"),
                            getopt_get_double(gopt, "torque"));
            servo->destroy(servo);
        }
    }
    else if (getopt_get_bool(gopt, "idle-all")) {
        for (int id = 0; id < getopt_get_int(gopt, "max-id"); id++) {
            dynamixel_device_t *servo = bus->get_servo(bus, id);
            if (servo == NULL)
                continue;
            servo->idle(servo);
            servo->destroy(servo);
        }
    }
    else if (getopt_was_specified(gopt, "id")) {
        int id = getopt_get_int(gopt, "id");
        dynamixel_device_t *servo = bus->get_servo(bus, id);

        if (servo == NULL) {
            printf("Couldn't find a servo at id %d\n", id);
            return 0;
        }

        if (getopt_get_bool(gopt, "home")) {
            printf("Commanding servo %d home\n", id);
            servo->set_goal(servo,
                            0,
                            getopt_get_double(gopt, "speed"),
                            getopt_get_double(gopt, "torque"));
        }

        if (getopt_get_bool(gopt, "idle")) {
            printf("Commanding servo %d idle\n", id);
            servo->idle(servo);
        }

        if (getopt_was_specified(gopt, "set-id")) {
            printf("Changing servo id %d to %d\n", id, getopt_get_int(gopt, "set-id"));
            servo->set_id(servo, getopt_get_int(gopt, "set-id"));
        }

        if (getopt_was_specified(gopt, "set-baud")) {
            printf("Changing servo id %d baud-rate to %d\n", id, getopt_get_int(gopt, "set-baud"));
            servo->set_baud(servo, getopt_get_int(gopt, "set-baud"));
        }

        if (getopt_was_specified(gopt, "set-degrees")) {
            printf("Seting servo id %d to %f degrees\n", id, getopt_get_double(gopt, "set-degrees"));
            servo->set_goal(servo,
                            to_radians(getopt_get_double(gopt, "set-degrees")),
                            .5,
                            .5);
        }

        servo->destroy(servo);
    }

    // Begin interactive mode
    // You can type '1' through '9' (followed by enter) to set position
    if (1) {
        double speed = getopt_get_double(gopt, "speed");
        double torque = getopt_get_double(gopt, "torque");

        int id0, id1;
        if (getopt_was_specified(gopt, "id")) {
            id0 = getopt_get_int(gopt, "id");
            id1 = id0;
        }
        else {
            id0 = 0;
            id1 = getopt_get_int(gopt, "max-id");
        }

        while (1) {
            if (id0 != id1)
                printf("\n");
            for (int id = id0; id <= id1; id++) {
                printf("%d...\r", id);
                fflush(stdout);

                dynamixel_device_t *servo = bus->get_servo(bus, id);
                if (servo == NULL)
                    continue;

                dynamixel_device_status_t *status = servo->get_status(servo);
                char buf[255];
                status->to_string(status, buf);

                printf("id %3d : %s v %-3d [ %s ]\n",
                       id,
                       servo->get_name(servo),
                       servo->get_firmware_version(servo),
                       buf);
                dynamixel_device_status_destroy(status);

                // Read from stdin
                char c[2];
                int res = read(STDIN_FILENO, c, 2);   // Will block until \n received
                assert (res <= 2);
                if (c[0] >= '1' && c[0] <= '9') {
                    double v = 1.0 * (c[0] - '1') / ('9' - '1');
                    double rad = to_radians(359.99999)*v - M_PI;
                    servo->set_goal(servo,
                                    rad,
                                    speed,
                                    torque);

                    double diff0=0.0, diff1=0.0;
                    int wait = 1;
                    dynamixel_device_status_t *status = NULL;
                    do {
                        status = servo->get_status(servo);
                        diff1 = diff0;
                        diff0 = fabs(status->position_radians - rad);
                        printf("deg=%.3f\trad=%.3f\tdiff0=%.3f\r", rad*180.0/M_PI, rad, diff0);
                        fflush(stdout);
                        if (diff0 < 2.0*M_PI/180.0) {
                            printf("\ngoal reached\n");
                            wait = 0;
                        }
                        else if (fabs(diff0-diff1) < 0.0005) {
                            printf("\ntimeout\n");
                            wait = 0;
                        }
                        usleep(250e3);
                        dynamixel_device_status_destroy(status);
                    }
                    while (wait);
                    usleep(0.5e6);

                    printf("\n");

                    char buf[255];
                    status = servo->get_status(servo);
                    status->to_string(status, buf);
                    printf("[ %s ]\n\n",
                           buf);
                    dynamixel_device_status_destroy(status);
                }

                // Sets servo ID to 0 for 'a' through 26 for 'z'
                if (c[0] >= 'a' && c[0] <= 'z') {
                    id = c[0] - 'a';
                    id0 = id;
                    id1 = id;
                }

                servo->destroy(servo);
            }
        }
    }

    // Find all of our servos
    /*for (int id = 0; id < getopt_get_int(gopt, "max-id"); id++) {
        printf("Searching for servo %d...\n", id);
        dynamixel_device_t *servo = bus->get_servo(bus, id);
        if (servo == NULL)
            continue;

        printf("Pinging servo %d...\n", id);
        int res = servo->ping(servo);
        if (res > 0) {
            printf("Successfully pinged servo %d!\n", id);
        } else {
            printf("WRN: Unable to ping servo %d.\n", id);
        }
        servo->destroy(servo);
    }*/

    bus->destroy(bus);
    getopt_destroy(gopt);

    return 0;
}
