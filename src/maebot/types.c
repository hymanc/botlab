
#include "types.h"

void serialize_state(state_t* state, void* vbuf)
{
    uint8_t* buf = (uint8_t*)vbuf;

    write64(buf +  0, state->utime);

    write32(buf +  8, state->encoder_left_ticks);
    write32(buf + 12, state->encoder_right_ticks);

    write16(buf + 16, state->motor_left_speed_cmd);
    write16(buf + 18, state->motor_right_speed_cmd);

    write16(buf + 20, state->accel[0]);
    write16(buf + 22, state->accel[1]);
    write16(buf + 24, state->accel[2]);
    write16(buf + 26, state->gyro[0]);
    write16(buf + 28, state->gyro[1]);
    write16(buf + 30, state->gyro[2]);

    write64(buf + 32, state->gyro_int[0]);
    write64(buf + 40, state->gyro_int[1]);
    write64(buf + 48, state->gyro_int[2]);

    writeu16(buf + 56, state->line_sensors[0]);
    writeu16(buf + 58, state->line_sensors[1]);
    writeu16(buf + 60, state->line_sensors[2]);

    writeu16(buf + 62, state->range);

    writeu16(buf + 64, state->motor_current_left);
    writeu16(buf + 66, state->motor_current_right);

    writeu8(buf + 68, state->pwm_prea);
    writeu8(buf + 69, state->pwm_diva);
    writeu16(buf + 70, state->pwm_prd);

    writeu8(buf + 72, state->flags);

    return;
}

void deserialize_state(void* vbuf, state_t* state)
{
    uint8_t* buf = (uint8_t*)vbuf;

    state->utime = read64(buf + 0);

    state->encoder_left_ticks = read32(buf + 8);
    state->encoder_right_ticks = read32(buf + 12);

    state->motor_left_speed_cmd  = read16(buf + 16);
    state->motor_right_speed_cmd = read16(buf + 18);

    state->accel[0] = read16(buf + 20);
    state->accel[1] = read16(buf + 22);
    state->accel[2] = read16(buf + 24);
    state->gyro[0]  = read16(buf + 26);
    state->gyro[1]  = read16(buf + 28);
    state->gyro[2]  = read16(buf + 30);

    state->gyro_int[0] = read64(buf + 32);
    state->gyro_int[1] = read64(buf + 40);
    state->gyro_int[2] = read64(buf + 48);

    state->line_sensors[0] = readu16(buf + 56);
    state->line_sensors[1] = readu16(buf + 58);
    state->line_sensors[2] = readu16(buf + 60);

    state->range = readu16(buf + 62);

    state->motor_current_left = readu16(buf + 64);
    state->motor_current_right = readu16(buf + 66);

    state->pwm_prea = readu8(buf + 68);
    state->pwm_diva = readu8(buf + 69);
    state->pwm_prd = readu16(buf + 70);

    state->flags = readu8(buf + 72);

    return;
}

void serialize_command(command_t* command, void* vbuf)
{
    uint8_t*  buf  = (uint8_t*)vbuf;

    writeu16(buf + 0, command->motor_left_speed);
    writeu16(buf + 2, command->motor_right_speed);

    writeu8(buf + 4, command->pwm_prea);
    writeu8(buf + 5, command->pwm_diva);
    writeu16(buf + 6, command->pwm_prd);

    writeu8(buf + 8,  command->flags);

    return;
}

void deserialize_command(void* vbuf, command_t* command)
{
    uint8_t* buf = (uint8_t*)vbuf;

    command->motor_left_speed = readu16(buf + 0);
    command->motor_right_speed = readu16(buf + 2);

    command->pwm_prea = readu8(buf + 4);
    command->pwm_diva = readu8(buf + 5);
    command->pwm_prd = readu16(buf + 6);

    command->flags = readu8(buf + 8);

    return;
}


uint8_t calc_checksum(uint8_t* buf, uint32_t len)
{
    if(len <= 0) return 0;
    uint8_t checksum = buf[0];
    uint32_t i;
    for(i = 1; i < len; i++)
    {
        checksum = checksum ^ buf[i];
    }
    return checksum;
}
