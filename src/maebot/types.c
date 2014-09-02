#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

#include "types.h"

void
serialize_state(state_t *state, void *buf)
{
    int64_t  *b64  = buf;
    int32_t  *b32  = buf;
    int16_t  *b16  = buf;
    uint16_t *bu16 = buf;
    uint8_t  *bu8  = buf;

    b64[0] = state->utime;

    b32[2] = state->encoder_left_ticks;
    b32[3] = state->encoder_right_ticks;

    b16[8] = state->motor_left_speed_cmd;
    b16[9] = state->motor_right_speed_cmd;

    b16[10] = state->accel[0];
    b16[11] = state->accel[1];
    b16[12] = state->accel[2];
    b16[13] = state->gyro[0];
    b16[14] = state->gyro[1];
    b16[15] = state->gyro[2];

    b64[4] = state->gyro_int[0];
    b64[5] = state->gyro_int[1];
    b64[6] = state->gyro_int[2];

    bu16[28] = state->line_sensors[0];
    bu16[29] = state->line_sensors[1];
    bu16[30] = state->line_sensors[2];

    bu16[31] = state->range;

    bu16[32] = state->motor_current_left;
    bu16[33] = state->motor_current_right;

    bu8[68] = state->pwm_prea;
    bu8[69] = state->pwm_diva;
    bu16[35] = state->pwm_prd;

    bu8[72] = state->flags;

    return;
}

void
deserialize_state (void *buf, state_t *state)
{
    int64_t  *b64  = buf;
    int32_t  *b32  = buf;
    int16_t  *b16  = buf;
    uint16_t *bu16 = buf;
    uint8_t  *bu8  = buf;

    state->utime = b64[0];

    state->encoder_left_ticks = b32[2];
    state->encoder_right_ticks = b32[3];

    state->motor_left_speed_cmd  = b16[8];
    state->motor_right_speed_cmd = b16[9];

    state->accel[0] = b16[10];
    state->accel[1] = b16[11];
    state->accel[2] = b16[12];
    state->gyro[0]  = b16[13];
    state->gyro[1]  = b16[14];
    state->gyro[2]  = b16[15];

    state->gyro_int[0] = b64[4];
    state->gyro_int[1] = b64[5];
    state->gyro_int[2] = b64[6];

    state->line_sensors[0] = bu16[28];
    state->line_sensors[1] = bu16[29];
    state->line_sensors[2] = bu16[30];

    state->range = bu16[31];

    state->motor_current_left = bu16[32];
    state->motor_current_right = bu16[33];

    state->pwm_prea = bu8[68];
    state->pwm_diva = bu8[69];
    state->pwm_prd = bu16[35];

    state->flags = bu8[72];

    return;
}

void
serialize_command (command_t *command, void *buf)
{
    int16_t *bu16 = buf;
    uint8_t *bu8  = buf;

    bu16[0] = command->motor_left_speed;
    bu16[1] = command->motor_right_speed;

    bu8[4] = command->pwm_prea;
    bu8[5] = command->pwm_diva;
    bu16[3] = command->pwm_prd;

    bu8[8] = command->flags;

    return;
}

void
deserialize_command (void *buf, command_t *command)
{
    int16_t *bu16 = buf;
    uint8_t *bu8  = buf;

    command->motor_left_speed = bu16[0];
    command->motor_right_speed = bu16[1];

    command->pwm_prea = bu8[36];
    command->pwm_diva = bu8[37];
    command->pwm_prd = bu16[19];

    command->flags = bu8[8];

    return;
}


uint8_t
calc_checksum (uint8_t *buf, uint32_t len)
{
    if (len <= 0)
        return 0;

    uint8_t checksum = buf[0];
    for (uint32_t i = 1; i < len; i++)
        checksum = checksum ^ buf[i];

    return checksum;
}
