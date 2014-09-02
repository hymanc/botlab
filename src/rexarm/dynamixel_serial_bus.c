#include <assert.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>

#include "common/serial.h"
#include "common/ioutils.h"

#include "dynamixel_serial_bus.h"
#include "dynamixel_device.h"

#define TIMEOUT_MS 50
#define VERBOSE 0

// === Bus specific implementation ===================
// Send an instruction with the specified parameters. The error code,
// body, and checksum of the response are returned (while the initial 4
// bytes of the header are removed)
static dynamixel_msg_t *
send_command_raw(dynamixel_bus_t *bus,
                 uint8_t id,
                 int instruction,
                 dynamixel_msg_t *params)
{
    dynamixel_serial_bus_impl_t *impl = (bus->impl);

    // Missing synchronization
    int parameterlen = (params == NULL) ? 0 : params->len;
    uint8_t *cmd = malloc((6+parameterlen)*sizeof(uint8_t));
    cmd[0] = 255;   // MAGIC
    cmd[1] = 255;   // MAGIC
    cmd[2] = id;    // servo id
    cmd[3] = (uint8_t)(parameterlen+2) & 0xff;  // Length
    cmd[4] = (uint8_t)(instruction & 0xff);

    if (params != NULL) {
        for (int i = 0; i < params->len; i++)
            cmd[5+i] = params->buf[i];
    }

    int checksum = 0;
    for (int i = 2; i < parameterlen+6-1; i++) {
        checksum += (cmd[i] & 0xff);
    }
    cmd[5+parameterlen] = (uint8_t)((checksum ^ 0xff) & 0xff);

    int res = write(impl->fd, cmd, 6+parameterlen);
    if (VERBOSE) {
        // XXX Dump cmd...which isn't a msg
    }
    free(cmd);

    // Read response. The header is really 5 bytes, but we put the
    // error code in the body so that the caller knows what went wrong
    // if something bad happens. Synchronize on the first two 0xffff
    // characters.
    dynamixel_msg_t *header = dynamixel_msg_create(4);
    int header_have = 0;
    while (header_have < 4) {
        res = read_fully_timeout(impl->fd,
                                 (header->buf)+header_have,
                                 4 - header_have,
                                 TIMEOUT_MS);

        if (VERBOSE) {
            printf("READ:  res = %d : ", res);
            dynamixel_msg_dump(header);
        }

        if (res < 1)
            return NULL;

        assert (res <= (4 - header_have));
        assert (res + header_have == 4);

        // If the first two bytes are the sync bytes, we're done
        if ((header->buf[0] & 0xff) == 0xff && (header->buf[1] & 0xff) == 0xff)
            break;

        // Shift buffer, read one more character
        header_have = 3;
        for (int i = 0; i < 3; i++)
            header->buf[i] = header->buf[i+1];
    }

    if ((header->buf[2] & 0xff) != id) {
        printf("serial_bus: Received response for wrong servo %d\n",
               header->buf[2] & 0xff);
        return NULL;
    }

    //int thisid = header->buf[2] & 0xff;
    int length = header->buf[3] & 0xff;

    if (length < 2)
        return NULL;

    dynamixel_msg_t *body = dynamixel_msg_create(length);
    res = read_fully_timeout(impl->fd,
                             body->buf,
                             body->len,
                             TIMEOUT_MS);

    if (VERBOSE) {
        printf("READ:  res = %d : ", res);
        dynamixel_msg_dump(body);
    }

    if (1) {
        int checksum = 0;
        for (int i = 2; i < header->len; i++)
            checksum += (header->buf[i] & 0xff);
        for (int i = 0; i < body->len-1; i++)
            checksum += (body->buf[i] & 0xff);
        checksum = (checksum & 0xff) ^ 0xff;
        if ((body->buf[body->len - 1] & 0xff) != checksum) {
            printf("serial_bus: Bad checksum %02x %02x\n",
                   body->buf[body->len - 1] & 0xff,
                   checksum);
            return NULL;
        }
    }

    dynamixel_msg_destroy(header);
    return body;
}

dynamixel_msg_t *
serial_bus_send_command(dynamixel_bus_t *bus,
                        int id,
                        int instruction,
                        dynamixel_msg_t *params,
                        int retry)
{
    do {
        dynamixel_msg_t *resp = send_command_raw(bus,
                                                 id,
                                                 instruction,
                                                 params);

        if (resp == NULL || resp->len < 1) {
            if (VERBOSE) {
                printf("serial_bus id=%d error: short response.\n", id);
            }
            continue;
        }

        // Something went wrong!
        if (resp->buf[0] != 0) {
            int code = (resp->buf[0]) & 0xff;

            int errormask = ERROR_ANGLE_LIMIT |
                            ERROR_VOLTAGE     |
                            ERROR_OVERLOAD;

            if ((code & (~errormask)) != 0)
                continue;
        }

        return resp;
    } while (retry && bus->retry_enable);

    return NULL;
}

// === Bus creation and destruction ==================
dynamixel_bus_t *
serial_bus_create(const char *device, int baud)
{
    dynamixel_bus_t *bus = dynamixel_bus_create();

    // Implementation stuff
    dynamixel_serial_bus_impl_t *impl = malloc(sizeof(*impl));
    impl->baud = baud;
    int fd = serial_open(device, baud, 1);

    // Check to see if we opened a file
    if (fd == -1) {
        printf("ERR: could not open serial port at %s\n", device);
        exit(-1);
    }
    impl->fd = fd;

    bus->impl = impl;

    // Fill in functions
    bus->send_command = serial_bus_send_command;
    bus->destroy = serial_bus_destroy;

    return bus;
}

void
serial_bus_destroy(dynamixel_bus_t *bus)
{
    free(bus->impl);
    dynamixel_bus_destroy(bus);
}
