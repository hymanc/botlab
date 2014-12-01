/** \file
   SPI driver for the CMUcam5 Pixy on Beaglebone Black.
*/

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>

enum {
    TYPE_NORMAL,
    TYPE_COLOR_CODE,
};

typedef struct {
    uint8_t type;
    uint16_t checksum;
    uint16_t signature;
    uint16_t x;
    uint16_t y;
    uint16_t width;
    uint16_t height;
    int16_t angle;
} pixy_t;

int debug = 0;
uint8_t mode = 0; //SPI_NO_CS; - this causes an error when trying to set mode
uint8_t bits = 8;           // bits per data word
uint32_t speed = 1000000;   // in Hz

/** Perform a bidirectional SPI transfer of len bytes. The bytes in txbuf are
    sent to the slave device while the bytes received are placed in rxbuf */
void spi_transfer(int fd, uint8_t *txbuf, uint8_t *rxbuf, int len)
{
    struct spi_ioc_transfer tfer = {
        .tx_buf = (uintptr_t)txbuf,
        .rx_buf = (uintptr_t)rxbuf,
        .len = len,
        .delay_usecs = 0,
        .speed_hz = speed,
        .bits_per_word = bits,
    };

    int ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tfer);
    if (ret < 1) {
        perror("spi transfer failed");
        abort();
    }
}

/*
void spi_read_buffered(int fd, uint8_t *dst, int len)
{
    static uint8_t buf[100];
    static const int bufsize = 100;
    static int bufidx = 100;

    // Fill buffer
    while (len) {
	// Fill buffer if necessary
	if (bufidx >= bufsize) {
	    spi_transfer(fd, NULL, buf, bufsize);
	    bufidx = 0;
	}

	// Copy n bytes to dst
	int n = (len < (bufsize-bufidx)) ? len : bufsize-bufidx;
	while (n--) {
	    *dst++ = buf[bufidx++];
	    len--;
	}
    }
}
*/

/** Read a 16-bit int in big-endian byte order */
uint16_t readu16(int fd)
{
    uint8_t rx[2];
    spi_transfer(fd, NULL, rx, 2);
    return (rx[1] | rx[0] << 8);
}

/** Read a 8-bit int */
uint8_t readu8(int fd)
{
    uint8_t rx[1];
    spi_transfer(fd, NULL, rx, 1);
    return rx[0];
}

int main(int argc, char *argv[])
{
    // so that redirected stdout won't be insanely buffered.
    setvbuf (stdout, (char *) NULL, _IONBF, 0);

    int opt;
    while ((opt = getopt(argc, argv, "dh")) != -1) {
        switch (opt) {
        case 'd':
            printf("Debug messages on\n");
            debug = 1;
            break;

        case 'h':
            printf("Usage: pixy_driver [options] [/path/to/spidev]\n");
            printf("-d\tTurn on debug messages\n");
            printf("-h\tPrint this help and exit\n");
            exit(1);
        }
    }

    char *port = "/dev/spidev1.0";
    if (argc > optind)
        port = argv[optind];

    printf("Opening device %s\n", port);

    // Open SPI port and set mode
    int fd = open(port, O_RDWR);
    if (fd == -1) {
        perror("can't open SPI device");
        abort();
    }

    if (ioctl(fd, SPI_IOC_WR_MODE, &mode) == -1)
        perror("can't set spi mode");
    if (ioctl(fd, SPI_IOC_RD_MODE, &mode) == -1)
        perror("can't read spi mode");

    if (ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits) == -1)
        perror("can't set bits per word");
    if (ioctl(fd, SPI_IOC_RD_BITS_PER_WORD, &bits) == -1)
        perror("can't read bits per word");

    if (ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed) == -1)
        perror("can't set max speed");
    if (ioctl(fd, SPI_IOC_RD_MAX_SPEED_HZ, &speed) == -1)
        perror("can't read max speed");

    printf("SPI config: mode %d, %d bits per word, %d Hz max\n",
           mode, bits, speed);

    int valid_blocks = 0;
    int checksum_errors = 0;

    while (1) {
        int i;

        // IMPORTANT: data appears to be big-endian, despite the documentation
        // Wait for sync pattern 0xaa55
        while ((i = readu8(fd)) != 0xaa) {
            //printf("%02x ", i);
            usleep(2000);
        }
        if ((i = readu8(fd)) == 0x55) {
            // Read blocks until zero byte is encountered
            // Blocks will begin with pattern 0xaa55 or 0xaa56
            while ((i = readu8(fd)) == 0xaa) {
                if (debug) printf("%02x ", i);

                // Decode packet
                pixy_t msg = {};
                int i = readu8(fd);
                if (debug) printf("%02x ", i);
                if (i == 0x55)
                    msg.type = TYPE_NORMAL;
                else if (i == 0x56)
                    msg.type = TYPE_COLOR_CODE;
                else
                    break;

                msg.checksum = readu16(fd);
                // Stupid packet format is ambiguous: pixy will send 0xAA55
                // BEFORE the block header to indicate the start of a frame.
                // Handle this case here:
                if (msg.checksum == 0xAA55) {
                    msg.type = TYPE_NORMAL;
                    msg.checksum = readu16(fd);
                } else if (msg.checksum == 0xAA56) {
                    msg.type = TYPE_COLOR_CODE;
                    msg.checksum = readu16(fd);
                }

                msg.signature = readu16(fd);
                msg.x = readu16(fd);
                msg.y = readu16(fd);
                msg.width = readu16(fd);
                msg.height = readu16(fd);
                if (msg.type == TYPE_COLOR_CODE)
                    msg.angle = readu16(fd);
                else
                    msg.angle = 0;

                if (debug)
                    printf("%d %d %d %d %d %d %d\n",
                           msg.checksum, msg.signature,
                           msg.x, msg.y, msg.width, msg.height, msg.angle);

                // Verify checksum
                uint16_t checksum = msg.signature + msg.x + msg.y + msg.width +
                    msg.height + msg.angle;
                if (msg.checksum != checksum) {
                    printf("failed checksum: received %x, computed %x\n",
			   msg.checksum, checksum);
                    checksum_errors += 1;
                    continue;
                }

                // Valid packet received
                // TODO: Publish to LCM
                valid_blocks += 1;
                if (msg.type == TYPE_COLOR_CODE) {
                    printf("Color code %d (octal %o) at (%d, %d) size (%d, %d)"
                           " angle %d\n", msg.signature, msg.signature,
                           msg.x, msg.y, msg.width, msg.height, msg.angle);
                } else {
                    printf("Signature %d at (%d, %d) size (%d, %d)\n",
                           msg.signature, msg.x, msg.y, msg.width, msg.height);
                }
            }

            if (debug) printf("%d valid blocks read, %d failed checksum\n",
                              valid_blocks, checksum_errors);
        }
    }

    printf("Exiting\n");
    close(fd);
}
