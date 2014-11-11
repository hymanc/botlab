#ifndef _TYPES_H
#define _TYPES_H

#include <stdint.h>

// Message types
#define STATE_TYPE 1
#define COMMAND_TYPE 2

/* Structure to hold robot state for transmission to variscite */
typedef struct state
{
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

void serialize_state(state_t* state, void* buf);
void deserialize_state(void* buf, state_t* state);
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

void serialize_command(command_t* command, void* buf);
void deserialize_command(void* buf, command_t* command);
#define COMMAND_T_BUFFER_BYTES 9

uint8_t calc_checksum(uint8_t* buf, uint32_t len);

static inline uint8_t readu8(uint8_t* buf);
static inline uint16_t readu16(uint8_t* buf);
static inline int16_t read16(uint8_t* buf);
static inline int32_t read32(uint8_t* buf);
static inline int64_t read64(uint8_t* buf);

static inline void writeu8(uint8_t* buf, uint8_t v);
static inline void writeu16(uint8_t* buf, uint16_t v);
static inline void write16(uint8_t* buf, int16_t v);
static inline void write32(uint8_t* buf, int32_t v);
static inline void write64(uint8_t* buf, int64_t v);

static inline uint8_t readu8(uint8_t* buf)
{
    return (uint8_t)buf[0];
}

static inline uint16_t readu16(uint8_t* buf)
{
    uint16_t v = 0;
    v |= buf[1] << 8;
    v |= buf[0];

    return v;
}

static inline int16_t read16(uint8_t* buf)
{
    int16_t v = 0;
    v |= buf[1] << 8;
    v |= buf[0];

    return v;
}

static inline int32_t read32(uint8_t* buf)
{
    int32_t v = 0;
    v |= buf[3] << 24;
    v |= buf[2] << 16;
    v |= buf[1] << 8;
    v |= buf[0];

    return v;
}

static inline int64_t read64(uint8_t* buf)
{
    int64_t v = 0;
    v |= (int64_t)buf[7] << 56;
    v |= (int64_t)buf[6] << 48;
    v |= (int64_t)buf[5] << 40;
    v |= (int64_t)buf[4] << 32;
    v |= (int64_t)buf[3] << 24;
    v |= (int64_t)buf[2] << 16;
    v |= (int64_t)buf[1] << 8;
    v |= (int64_t)buf[0];

    return v;
}

static inline void writeu8(uint8_t* buf, uint8_t i)
{
    buf[0] = i;
}

static inline void writeu16(uint8_t* buf, uint16_t i)
{
    buf[1] = i >> 8;
    buf[0] = i;
}

static inline void write16(uint8_t* buf, int16_t i)
{
    buf[1] = i >> 8;
    buf[0] = i;
}

static inline void write32(uint8_t* buf, int32_t i)
{
    buf[3] = i >> 24;
    buf[2] = i >> 16;
    buf[1] = i >> 8;
    buf[0] = i;

}

static inline void write64(uint8_t* buf, int64_t i)
{
    buf[7] = i >> 56;
    buf[6] = i >> 48;
    buf[5] = i >> 40;
    buf[4] = i >> 32;
    buf[3] = i >> 24;
    buf[2] = i >> 16;
    buf[1] = i >> 8;
    buf[0] = i;
}

#endif
