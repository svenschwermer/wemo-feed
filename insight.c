#include <assert.h>
#include <errno.h>
#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <linux/serial.h>
#include <sys/ioctl.h>
#include "ring_buffer.h"

static const char *SERIAL_DEVICE = "/dev/ttyS1";
static const speed_t BAUDRATE = B9600;
static const uint8_t START_BYTE = 0xAE;
static const uint8_t MESSAGE_LENGTH = 30;

enum fsm_state
{
    FIND_START,
    READ_LENGTH,
    READ_MESSAGE,
    VERIFY_CHECKSUM,
};

struct report
{
    float int_temperature;
    float ext_temperature;
    float rms_voltage;
    float rms_current;
    float active_power;
    float average_power;
    float power_factor;
    float line_frequency;
    float active_energy;
};

static void fsm(int fd);
static void consume_message(struct ring_buffer *rb);
static void print_report(const struct report *report);
static float get_word(struct ring_buffer *rb, float factor);
static int open_serial_port(void);

int main()
{
    int fd = open_serial_port();
    assert(fd > 0);
    fsm(fd);
    close(fd);
    return 0;
}

static void fsm(int fd)
{
    int c;
    uint8_t length, i, checksum;
    enum fsm_state state = FIND_START;
    struct ring_buffer *rb = rb_new();
    assert(rb != NULL);

    while (1)
    {
        c = rb_getc(rb, fd);
        if (c < 0)
        {
            fprintf(stderr, "Failed to read byte: %s\n", strerror(-c));
            break;
        }
        checksum += c;

        switch (state)
        {
        case FIND_START:
            if (c == START_BYTE)
            {
                checksum = c;
                state = READ_LENGTH;
            }
            else
            {
                fprintf(stderr, "Discarding non-start byte (0x%02x)\n", c);
                rb_pop(rb);
            }
            break;

        case READ_LENGTH:
            length = c;
            if (length == MESSAGE_LENGTH)
            {
                i = 2;
                state = READ_MESSAGE;
            }
            else
            {
                fprintf(stderr, "Unexpected length (%d)\n", length);
                rb_pop(rb);
                rb_reset_iterator(rb);
                state = FIND_START;
            }
            break;

        case READ_MESSAGE:
            if (++i >= length - 1)
                state = VERIFY_CHECKSUM;
            break;

        case VERIFY_CHECKSUM:
            if (checksum == 0)
                consume_message(rb);
            else
            {
                fprintf(stderr, "Wrong checksum (0x%02x)\n", checksum);
                rb_pop(rb);
            }
            rb_reset_iterator(rb);
            state = FIND_START;
            break;
        }
    }

    rb_free(rb);
    fputs("FSM terminating\n", stderr);
}

static void consume_message(struct ring_buffer *rb)
{
    struct report r;
    rb_pop(rb); // start byte
    rb_pop(rb); // length byte
    r.int_temperature = get_word(rb, 0.001);
    r.ext_temperature = get_word(rb, 0.001);
    r.rms_voltage = get_word(rb, 0.001);
    r.rms_current = get_word(rb, 0.001 / 128);
    r.active_power = get_word(rb, 0.005);
    r.average_power = get_word(rb, 0.005);
    r.power_factor = get_word(rb, 0.001);
    r.line_frequency = get_word(rb, 0.001);
    r.active_energy = get_word(rb, 1); // factor unknown
    rb_pop(rb);                        // checksum

    print_report(&r);
}

static void print_report(const struct report *r)
{
    printf(
        "Int. temperature: % 8.3f°C\n"
        "Ext. temperature: % 8.3f°C\n"
        "Line voltage:     % 8.3f V\n"
        "Current (RMS):    % 8.3f A\n"
        "Active power:     % 8.3f W\n"
        "Average power:    % 8.3f W\n"
        "Power factor:     % 8.3f\n"
        "Line frequency:   % 8.3f Hz\n"
        "Active energy:    % 8.3f\n"
        "-----------------------------\n",
        r->int_temperature, r->ext_temperature,
        r->rms_voltage, r->rms_current,
        r->active_power, r->average_power,
        r->power_factor, r->line_frequency,
        r->active_energy);
}

// Consumes 3 bytes from the ring buffer and assembles them to a little endian
// word. The word multiplied by the given factor is returned. Panics if not
// enough data is available in the ring buffer.
static float get_word(struct ring_buffer *rb, float factor)
{
    int32_t word = 0;
    for (int i = 0; i < 3; ++i)
    {
        int c = rb_pop(rb);
        assert(c >= 0);
        word |= (uint8_t)c << 8 * i;
    }
    if (word & 0x00800000)
        word |= 0xff000000;
    return word * factor;
}

static int open_serial_port(void)
{
    int fd = open(SERIAL_DEVICE, O_RDONLY | O_NOCTTY);
    if (fd < 0)
    {
        fprintf(stderr, "Failed to open serial port %s: %s\n", SERIAL_DEVICE, strerror(errno));
        return -1;
    }

    struct termios termios;
    if (tcgetattr(fd, &termios) < 0)
    {
        fprintf(stderr, "Failed to get terminal attributes: %s\n", strerror(errno));
        goto error;
    }

    termios.c_oflag &= ~OPOST;
    termios.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // raw mode
    termios.c_iflag &= ~(IXON | IXOFF | IXANY);         // disable software flow control
    termios.c_iflag |= IGNBRK;                          // ignore break
    termios.c_iflag &= ~ISTRIP;                         // do not strip high bit
    termios.c_iflag &= ~(INLCR | ICRNL);                // do not modify CR/NL
    termios.c_cflag |= (CLOCAL | CREAD);                // enable the receiver and set local mode
    termios.c_cflag &= ~CRTSCTS;                        // disable hardware flow control
    termios.c_cflag &= ~CSIZE;                          // Mask the character size bits
    termios.c_cflag |= CS8;                             // 8 data bits
    termios.c_cflag &= ~PARENB;                         // no parity
    termios.c_cflag &= ~CSTOPB;                         // 1 stop bit

    if (cfsetspeed(&termios, BAUDRATE) < 0)
    {
        fprintf(stderr, "Setting baudrate failed: %s\n", strerror(errno));
        goto error;
    }

    if (tcsetattr(fd, TCSANOW, &termios) < 0)
    {
        fprintf(stderr, "Failed to set terminal attributes: %s\n", strerror(errno));
        goto error;
    }

    struct serial_struct ser;
    if (ioctl(fd, TIOCGSERIAL, &ser) != 0)
    {
        fprintf(stderr, "Failed to get serial line configuration: %s\n", strerror(errno));
        goto error;
    }

    ser.flags &= ~ASYNC_SPD_MASK;
    ser.flags |= ASYNC_SPD_CUST;
    ser.custom_divisor = ser.baud_base / 38400;

    if (ioctl(fd, TIOCSSERIAL, &ser) != 0)
    {
        fprintf(stderr, "Failed to set serial line configuration: %s\n", strerror(errno));
        goto error;
    }

    return fd;

error:
    close(fd);
    return -1;
}
