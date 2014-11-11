#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <pthread.h>
#include <signal.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <sys/time.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>


#include <lcm/lcm.h>
#include "lcmtypes/maebot_command_t.h"
#include "lcmtypes/maebot_state_t.h"
#include "lcmtypes/maebot_diff_drive_t.h"
#include "lcmtypes/maebot_motor_feedback_t.h"
#include "lcmtypes/maebot_sensor_data_t.h"
#include "lcmtypes/maebot_leds_t.h"
#include "lcmtypes/maebot_laser_t.h"

#include "types.h"

#define I2C_DEVICE_PATH "/dev/i2c-3"
#define LED_ADDRESS 0x4D


#ifndef max
	#define max( a, b ) ( ((a) > (b)) ? (a) : (b) )
#endif

#ifndef min
	#define min( a, b ) ( ((a) < (b)) ? (a) : (b) )
#endif

const uint32_t HEADER_BYTES = 12;
const uint32_t UART_MAGIC_NUMBER = 0xFDFDFDFD;  // Marks Beginning of Message

/* Reads the value of the gpio value file and returns a boolean 0 or 1 */
static unsigned char _read_gpio(int fd)
{
	char c;
	lseek(fd, 0, SEEK_SET);
	if (read(fd, &c, 1) != 1) {
	  printf("error reading gpio\r\n");
	  return 0;
	};
	return (c == '1');
}


typedef struct maebot_shared_state {
    maebot_diff_drive_t diff_drive;
    maebot_motor_feedback_t motor_feedback;
    maebot_sensor_data_t sensor_data;
    maebot_leds_t leds;
    maebot_laser_t laser;

} maebot_shared_state_t;

pthread_mutex_t statelock;

maebot_shared_state_t shared_state;

pthread_cond_t leds_write_cond;
pthread_mutex_t leds_write_cond_mutex;

lcm_t* lcm;
int port;
int i2c_leds_fd;

int
writen (int fd, const void *buf, size_t count);

void
clamp(float *val, float min, float max)
{
    *val = min (*val, max);
    *val = max (*val, min);
}

void *
leds_write_thread (void *);

int
send_command (command_t command, int port)
{
    const uint32_t msg_sz = HEADER_BYTES + COMMAND_T_BUFFER_BYTES + 1;
	uint8_t buf[msg_sz];

	uint32_t *buf32 = (uint32_t *)buf;

	buf32[0] = UART_MAGIC_NUMBER;
	buf32[1] = COMMAND_T_BUFFER_BYTES;//msg_sz;
	buf32[2] = COMMAND_TYPE;

	serialize_command (&command, (void *)(buf + HEADER_BYTES));

	//int i;
    //printf("buf: ");
    //for(i = 0; i < COMMAND_T_BUFFER_BYTES; i++)
    //{
    //    printf("%d ", (int) (((uint8_t*)buf + HEADER_BYTES)[i]));
    //}

    buf[msg_sz - 1] = calc_checksum (buf + HEADER_BYTES, COMMAND_T_BUFFER_BYTES);

    writen (port, buf, msg_sz);

    return 0;
}

int
open_port (void)
{
    //attempt to open port
    int fd;

    if ((fd = open("/dev/ttyO1", O_RDWR | O_NOCTTY | O_NONBLOCK)) == -1)
	    printf ("error opening /dev/ttyO1\r\n");
    else
	    fcntl (fd, F_SETFL, 0);

    return fd;
}

int
configure_port (int fd)
{
  struct termios old_settings;
    if (tcgetattr (fd, &old_settings) != 0) {
	    printf ("error reading port config\r\n");
	    return -1;
    }

    cfmakeraw (&old_settings);
    cfsetspeed (&old_settings, B115200);

    if (tcsetattr (fd, TCSANOW, &old_settings)) {
	    printf ("error setting port config\r\n");
	    return -1;
    }

    return fd;
}

int
readn (int fd, void *buf, unsigned int count)
{
	int ret;
	int sofar = 0;
	while (sofar < count) {
		ret = read (fd, ((char *)buf) + sofar, count - sofar);
		if (ret <= 0) {
			printf ("Error or end of file. %d\n", ret);
			return 1;
		}
		sofar += ret;
	}
	return sofar;
}

int
writen (int fd, const void* buf, size_t count)
{
	int ret;
	int i = 0;
	while(i < count) {
		ret = write (fd, ((char *)buf) + i, count - i);
		if(ret <= 0) {
			printf ("Error writing to file descriptor.\n");
			return 1;
		}
		i += ret;
	}
	return i;
}

state_t
get_state (int port)
{
	state_t state;

	uint8_t magic_check;
	uint8_t num_magic;

	uint8_t done = 0;
	while (!done) {

        magic_check = 0;
        num_magic = 0;

        while (num_magic != 4) {
            readn (port, (void *)&magic_check, 1);
            if (magic_check == 0xFD)
                num_magic++;
        }

        uint32_t size = 0;
        readn (port, (void *)&size, 4);

        //printf("Message size: %d\n", size);

        uint32_t type = 0;
        readn (port, (void *)&type, 4);

        if (type == STATE_TYPE)	{
       		if (size != STATE_T_BUFFER_BYTES) {
                printf ("Bad packet: expected size=%d, found size=%d\r\n",
                        STATE_T_BUFFER_BYTES, size);
                continue;
            }
	        uint8_t buf[STATE_T_BUFFER_BYTES];
	        readn (port, buf, STATE_T_BUFFER_BYTES);

            uint8_t checksum;
            readn (port, (void *)&checksum, 1);

            if (checksum != calc_checksum (buf, STATE_T_BUFFER_BYTES)) {
                printf ("Bad packet: checksum failed\r\n");
                continue;
            }

	        deserialize_state (buf, &state);
            done = 1;
        }
        else {
            printf ("Unrecognized type: %d\r\n", type);
            continue;
        }
	}

	return state;
}

void *
sama5_state_thread (void *arg)
{
	state_t state;

        int user_button = open("/sys/class/gpio/gpio174/value", O_RDONLY);
	if (user_button < 0) {
		printf("Error opening file: %m\n");
	}

	while(1) {
		// Telemetry handling
		state = get_state (port);

		pthread_mutex_lock (&statelock);

        shared_state.motor_feedback.utime = state.utime;
        shared_state.sensor_data.utime = state.utime;

		// Copy motor feedback
		shared_state.motor_feedback.encoder_left_ticks = state.encoder_left_ticks;
		shared_state.motor_feedback.encoder_right_ticks = state.encoder_right_ticks;

		shared_state.motor_feedback.motor_left_commanded_speed =
            (float)state.motor_left_speed_cmd / UINT16_MAX;
		if (state.flags & flags_motor_left_reverse_cmd_mask)
            shared_state.motor_feedback.motor_left_commanded_speed *= -1.0;

		shared_state.motor_feedback.motor_left_commanded_speed =
            (float)state.motor_right_speed_cmd / UINT16_MAX;
		if (state.flags & flags_motor_right_reverse_cmd_mask)
            shared_state.motor_feedback.motor_right_commanded_speed *= -1.0;

		// Actual same as commanded for now. No slewing logic yet.
		shared_state.motor_feedback.motor_left_actual_speed =
            shared_state.motor_feedback.motor_left_commanded_speed;
		shared_state.motor_feedback.motor_right_actual_speed =
            shared_state.motor_feedback.motor_right_commanded_speed;

		// Copy sensor data
		shared_state.sensor_data.accel[0] = state.accel[0];
		shared_state.sensor_data.accel[1] = state.accel[1];
		shared_state.sensor_data.accel[2] = state.accel[2];
		shared_state.sensor_data.gyro[0] = state.gyro[0];
		shared_state.sensor_data.gyro[1] = state.gyro[1];
		shared_state.sensor_data.gyro[2] = state.gyro[2];
        shared_state.sensor_data.gyro_int[0] = state.gyro_int[0];
        shared_state.sensor_data.gyro_int[1] = state.gyro_int[1];
        shared_state.sensor_data.gyro_int[2] = state.gyro_int[2];
		shared_state.sensor_data.line_sensors[0] = state.line_sensors[0];
		shared_state.sensor_data.line_sensors[1] = state.line_sensors[1];
		shared_state.sensor_data.line_sensors[2] = state.line_sensors[2];
		shared_state.sensor_data.range = state.range;
		shared_state.motor_feedback.motor_current_left = state.motor_current_left;
		shared_state.motor_feedback.motor_current_right = state.motor_current_right;
		shared_state.sensor_data.power_button_pressed = state.flags & flags_power_button_mask;

		shared_state.sensor_data.user_button_pressed = _read_gpio(user_button);

		maebot_motor_feedback_t_publish (lcm, "MAEBOT_MOTOR_FEEDBACK", &shared_state.motor_feedback);
		maebot_sensor_data_t_publish (lcm, "MAEBOT_SENSOR_DATA", &shared_state.sensor_data);

		pthread_mutex_unlock (&statelock);
	}

	return 0;
}

static uint8_t pwm_prea;
static uint8_t pwm_diva;
static uint16_t pwm_prd;

void
sama5_send_command (void)
{
    command_t command;

    pthread_mutex_lock (&statelock);

    command.motor_left_speed = fabs (shared_state.diff_drive.motor_left_speed) * UINT16_MAX;
    command.motor_right_speed = fabs (shared_state.diff_drive.motor_right_speed) * UINT16_MAX;

    command.pwm_prea = pwm_prea;
    command.pwm_diva = pwm_diva;
    command.pwm_prd = pwm_prd;

    command.flags = 0;
    if (shared_state.diff_drive.motor_left_speed < 0)
        command.flags |= flags_motor_left_reverse_mask;
    if (shared_state.diff_drive.motor_right_speed < 0)
        command.flags |= flags_motor_right_reverse_mask;
    if (shared_state.leds.bottom_led_left)
        command.flags |= flags_led_left_power_mask;
    if (shared_state.leds.bottom_led_middle)
        command.flags |= flags_led_middle_power_mask;
    if (shared_state.leds.bottom_led_right)
        command.flags |= flags_led_right_power_mask;
    if (shared_state.leds.line_sensor_leds)
        command.flags |= flags_line_sensor_led_power_mask;

    send_command (command, port);

    pthread_mutex_unlock (&statelock);
}

void
maebot_shared_state_init (maebot_shared_state_t *state)
{
    // diff drive
    state->diff_drive.motor_left_speed = 0.0;
    state->diff_drive.motor_right_speed = 0.0;

    // motor_feedback
    state->motor_feedback.encoder_left_ticks = 0;
    state->motor_feedback.encoder_left_ticks = 0;
    state->motor_feedback.motor_current_left = 0;
    state->motor_feedback.motor_current_right = 0;
    state->motor_feedback.motor_left_commanded_speed = 0.0;
    state->motor_feedback.motor_right_commanded_speed = 0.0;
    state->motor_feedback.motor_left_actual_speed = 0.0;
    state->motor_feedback.motor_right_actual_speed = 0.0;

    // sensor data
    state->sensor_data.accel[0] = 0;
    state->sensor_data.accel[1] = 0;
    state->sensor_data.accel[2] = 0;
    state->sensor_data.gyro[0] = 0;
    state->sensor_data.gyro[1] = 0;
    state->sensor_data.gyro[2] = 0;
    state->sensor_data.line_sensors[0] = 0;
    state->sensor_data.line_sensors[1] = 0;
    state->sensor_data.line_sensors[2] = 0;
    state->sensor_data.range = 0;
    state->sensor_data.user_button_pressed = 0;
    state->sensor_data.power_button_pressed = 0;

    // leds
    state->leds.top_rgb_led_left = 0x02;
    state->leds.top_rgb_led_right = 0x02;
    state->leds.bottom_led_left = 0;
    state->leds.bottom_led_middle = 0;
    state->leds.bottom_led_right = 0;
    state->leds.line_sensor_leds = 1; //default on

    // laser
    state->laser.laser_power = 0;
}



//////////////////////
//                  //
// LCM Handlers     //
//                  //
//////////////////////


static void
diff_drive_handler (const lcm_recv_buf_t *rbuf, const char *channel,
                    const maebot_diff_drive_t *msg, void *user)
{
    //printf ("recieved msg on channel DIFF_DRIVE\n");

    pthread_mutex_lock (&statelock);

    // copy into shared state;
    shared_state.diff_drive = *msg;
    clamp(&shared_state.diff_drive.motor_left_speed, -1.0, 1.0);
    clamp(&shared_state.diff_drive.motor_right_speed, -1.0, 1.0);

    pthread_mutex_unlock (&statelock);

    sama5_send_command ();
}

static void
laser_handler (const lcm_recv_buf_t *rbuf, const char *channel,
               const maebot_laser_t *msg, void *user)
{
    pthread_mutex_lock (&statelock);

    //printf ("recieved msg on channel LASER\n");

    // copy into shared state;
    shared_state.laser = *msg;

    pthread_mutex_unlock (&statelock);

    if (msg->laser_power) {
        if (system ("echo 1 > /sys/class/gpio/gpio172/value"))
            printf ("Error writing to laser pin");
    }
    else {
        if (system ("echo 0 > /sys/class/gpio/gpio172/value"))
            printf ("Error writing to laser pin");
    }
}


static void
leds_handler (const lcm_recv_buf_t *rbuf, const char *channel,
              const maebot_leds_t *msg, void *user)
{
    pthread_mutex_lock (&statelock);
    shared_state.leds = *msg;
    pthread_mutex_unlock (&statelock);

    pthread_mutex_lock (&leds_write_cond_mutex);
    pthread_cond_broadcast (&leds_write_cond);
    pthread_mutex_unlock (&leds_write_cond_mutex);
}


void
leds_write (void)
{
    sama5_send_command ();

    pthread_mutex_lock (&statelock);
    uint8_t cmd[6];
    cmd[0] = (1 << 5) | ((shared_state.leds.top_rgb_led_right >> (16 + 3)) & 0x1F);
    cmd[1] = (2 << 5) | ((shared_state.leds.top_rgb_led_right >> ( 8 + 3)) & 0x1F);
    cmd[2] = (3 << 5) | ((shared_state.leds.top_rgb_led_right >> ( 0 + 3)) & 0x1F);
    cmd[3] = (4 << 5) | ((shared_state.leds.top_rgb_led_left  >> (16 + 3)) & 0x1F);
    cmd[4] = (5 << 5) | ((shared_state.leds.top_rgb_led_left  >> ( 8 + 3)) & 0x1F);
    cmd[5] = (6 << 5) | ((shared_state.leds.top_rgb_led_left  >> ( 0 + 3)) & 0x1F);
    pthread_mutex_unlock (&statelock);

    int fd = i2c_leds_fd;
    //fd = open(I2C_DEVICE_PATH, O_RDWR);
    if (ioctl (fd, I2C_SLAVE, LED_ADDRESS) < 0)
        printf ("Failed to set slave address: %m\n");

    for (int i = 0; i < 6; i++) {
        if (i2c_smbus_write_byte (fd, cmd[i]) < 0)
            printf ("Failed to write to I2C device: %m\n");
    }

    //close(fd);
}

void *
leds_write_thread (void *arg)
{
    while(1) {
        pthread_mutex_lock (&leds_write_cond_mutex);
        pthread_cond_wait (&leds_write_cond, &leds_write_cond_mutex);
        pthread_mutex_unlock (&leds_write_cond_mutex);
        leds_write ();
    }

    return NULL;
}

int
main (int argc, char *argv[])
{
    maebot_shared_state_init (&shared_state);

	lcm = lcm_create ("udpm://239.255.76.67:7667?ttl=0");
	if (!lcm)
		return 1;

    if (pthread_mutex_init (&statelock, NULL)) {
        printf ("mutex initialization failed\n");
        return 1;
    }

	port = open_port ();
	if (port == -1) {
		printf ("error opening port\n");
		return 1;
	}

	port = configure_port (port);

    i2c_leds_fd = open (I2C_DEVICE_PATH, O_RDWR);

    if (pthread_cond_init (&leds_write_cond, NULL)) {
        printf ("condition variable init failed\n");
        return 1;
    }

    if (pthread_mutex_init (&leds_write_cond_mutex, NULL)) {
        printf ("mutex initialization failed\n");
        return 1;
    }
    pthread_t leds_write_thread_pid;
    pthread_create (&leds_write_thread_pid, NULL, leds_write_thread, NULL);

    // Subscribe to LCM Channels
    maebot_diff_drive_t_subscribe (lcm, "MAEBOT_DIFF_DRIVE", &diff_drive_handler, NULL);
    printf ("Listening on channel MAEBOT_DIFF_DRIVE\n");

    maebot_leds_t_subscribe (lcm, "MAEBOT_LEDS", &leds_handler, NULL);
    printf ("Listening on channel MAEBOT_LEDS\n");

    maebot_laser_t_subscribe (lcm, "MAEBOT_LASER", &laser_handler, NULL);
    printf ("Listening on channel MAEBOT_LASER\n");

    // Read state from sama5
	pthread_t sama5_state_thread_pid;
	pthread_create (&sama5_state_thread_pid, NULL, sama5_state_thread, NULL);

	while (1)
		lcm_handle(lcm);
}
