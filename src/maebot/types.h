#ifndef __TYPES_H__
#define __TYPES_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Message types
#define STATE_TYPE 1
#define COMMAND_TYPE 2

/* Structure to hold robot state for transmission to variscite */
typedef struct state{
    uint64_t utime;

    int32_t encoder_left_ticks;
    int32_t encoder_right_ticks;

    uint16_t motor_left_speed_cmd;
    uint16_t motor_right_speed_cmd;

    int16_t accel[3]; // X, Y, Z
    int16_t gyro[3];  // r, p, y ?

    int64_t gyro_int[3];

    uint16_t line_sensors[3]; // 0 - Left, 1 - Center, 2 - Right
    uint16_t range;

    uint16_t motor_current_left;
    uint16_t motor_current_right;

    uint8_t pwm_prea; // pwm_freq = 128MHz / ((2 ^ prea) * diva * prd);
    uint8_t pwm_diva;
    uint16_t pwm_prd;

    uint8_t flags;

/*
  unsigned int power_button_pressed    : 1;
  unsigned int motor_left_reverse_cmd  : 1;
  unsigned int motor_right_reverse_cmd : 1;
  unsigned int motor_left_coast_cmd    : 1;
  unsigned int motor_right_coast_cmd   : 1;
  unsigned int extra                   : 3;
*/
} state_t;

#define flags_power_button_mask            (1 << 0)
#define flags_motor_left_reverse_cmd_mask  (1 << 1)
#define flags_motor_right_reverse_cmd_mask (1 << 2)
#define flags_motor_left_coast_cmd_mask    (1 << 3)
#define flags_motor_right_coast_cmd_mask   (1 << 4)

void
serialize_state (state_t *state, void *buf);
void
deserialize_state (void *buf, state_t *state);
#define STATE_T_BUFFER_BYTES 73


/* Structure to hold commands received by the variscite */
typedef struct command
{
    uint16_t motor_left_speed;
    uint16_t motor_right_speed;

    uint8_t pwm_prea; // pwm_freq = 128MHz / ((2 ^ prea) * diva * prd);
    uint8_t pwm_diva;
    uint16_t pwm_prd;

    uint8_t flags; // see below
/*
    unsigned int motor_left_reverse    : 1;
    unsigned int motor_right_reverse   : 1;
    unsigned int led_left_power        : 1;
    unsigned int led_middle_power      : 1;
    unsigned int led_right_power       : 1;
    unsigned int line_sensor_led_power : 1;
    unsigned int motor_left_coast      : 1;
    unsigned int motor_right_coast     : 1;
*/
} command_t;

// Masks for flags field in command_t
#define flags_motor_left_reverse_mask    (1 << 0)
#define flags_motor_right_reverse_mask   (1 << 1)
#define flags_led_left_power_mask        (1 << 2)
#define flags_led_middle_power_mask      (1 << 3)
#define flags_led_right_power_mask       (1 << 4)
#define flags_line_sensor_led_power_mask (1 << 5)
#define flags_motor_left_coast_mask      (1 << 6)
#define flags_motor_right_coast_mask     (1 << 7)

void
serialize_command (command_t *command, void *buf);
void
deserialize_command (void *buf, command_t *command);
#define COMMAND_T_BUFFER_BYTES 9


uint8_t
calc_checksum (uint8_t *buf, uint32_t len);

#ifdef __cplusplus
}
#endif

#endif //__TYPES_H__
