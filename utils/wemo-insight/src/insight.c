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

#include <libubox/ulog.h>
#include <libubox/ustream.h>

#include "insight.h"
#include "ring_buffer.h"

enum fsm_state
{
    FIND_START,
    READ_LENGTH,
    READ_MESSAGE,
    VERIFY_CHECKSUM,
};

struct insight_data
{
    struct ustream_fd s;
    struct ring_buffer *rb;
    enum fsm_state state;
    uint8_t length, i, checksum;
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

static void read_cb(struct ustream *s, int bytes_new);
static void state_cb(struct ustream *s);
static void consume_message(struct ring_buffer *rb);
static void print_report(const struct report *report);
static float get_word(struct ring_buffer *rb, float factor);
static int open_serial_port(void);

static const speed_t BAUDRATE = B9600;
static const uint8_t START_BYTE = 0xAE;
static const uint8_t MESSAGE_LENGTH = 30;

struct insight_data *insight_open(const char *dev)
{
    struct insight_data *data = malloc(sizeof(struct insight_data));
    if (data == NULL)
    {
        ULOG_ERR("Failed to allocate insight data: %s\n", strerror(errno));
        goto err_exit;
    }
    data->rb = rb_new();
    if (data->rb == NULL)
    {
        ULOG_ERR("Failed to allocate ring buffer: %s\n", strerror(errno));
        goto free_data;
    }
    data->state = FIND_START;

    int fd = open(dev, O_RDONLY | O_NOCTTY);
    if (fd < 0)
    {
        ULOG_ERR("Failed to open serial port %s: %s\n", dev, strerror(errno));
        goto free_data;
    }

    struct termios tio;
    if (tcgetattr(fd, &tio) < 0)
    {
        ULOG_ERR("Failed to get terminal attributes: %s\n", strerror(errno));
        goto close_fd;
    }

    tio.c_oflag &= ~OPOST;
    tio.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // raw mode
    tio.c_iflag &= ~(IXON | IXOFF | IXANY);         // disable software flow control
    tio.c_iflag |= IGNBRK;                          // ignore break
    tio.c_iflag &= ~ISTRIP;                         // do not strip high bit
    tio.c_iflag &= ~(INLCR | ICRNL);                // do not modify CR/NL
    tio.c_cflag |= (CLOCAL | CREAD);                // enable the receiver and set local mode
    tio.c_cflag &= ~CRTSCTS;                        // disable hardware flow control
    tio.c_cflag &= ~CSIZE;                          // Mask the character size bits
    tio.c_cflag |= CS8;                             // 8 data bits
    tio.c_cflag &= ~PARENB;                         // no parity
    tio.c_cflag &= ~CSTOPB;                         // 1 stop bit

    if (cfsetspeed(&tio, BAUDRATE) < 0)
    {
        ULOG_ERR("Setting baudrate failed: %s\n", strerror(errno));
        goto close_fd;
    }

    if (tcsetattr(fd, TCSANOW, &tio) < 0)
    {
        ULOG_ERR("Failed to set terminal attributes: %s\n", strerror(errno));
        goto close_fd;
    }

    struct serial_struct ser;
    if (ioctl(fd, TIOCGSERIAL, &ser) != 0)
    {
        ULOG_ERR("Failed to get serial line configuration: %s\n", strerror(errno));
        goto close_fd;
    }

    ser.flags &= ~ASYNC_SPD_MASK;
    ser.flags |= ASYNC_SPD_CUST;
    ser.custom_divisor = ser.baud_base / 38400;

    if (ioctl(fd, TIOCSSERIAL, &ser) != 0)
    {
        ULOG_ERR("Failed to set serial line configuration: %s\n", strerror(errno));
        goto close_fd;
    }

    data->s.stream.string_data = true;
    data->s.stream.notify_read = read_cb;
    data->s.stream.notify_state = state_cb;
    ustream_fd_init(&data->s, fd);

    tcflush(fd, TCIFLUSH);

    return data;

close_fd:
    close(fd);
free_data:
    if (data->rb != NULL)
        rb_free(data->rb);
    free(data);
err_exit:
    return NULL;
}

int insight_free(struct insight_data *h)
{
    close(h->s.fd.fd);
    ustream_free(&h->s.stream);
    rb_free(h->rb);
    free(h);
    return 0;
}

static void read_cb(struct ustream *s, int bytes_new)
{
    struct ustream_fd *s_fd = container_of(s, struct ustream_fd, stream);
    struct insight_data *data = container_of(s_fd, struct insight_data, s);

    while (bytes_new-- > 0)
    {
        int c = rb_getc(data->rb, s);
        if (c < 0)
        {
            ULOG_ERR("Failed to read byte: %s\n", strerror(-c));
            break;
        }
        data->checksum += c;

        switch (data->state)
        {
        case FIND_START:
            if (c == START_BYTE)
            {
                data->checksum = c;
                data->state = READ_LENGTH;
            }
            else
            {
                ULOG_WARN("Discarding non-start byte (0x%02x)\n", c);
                rb_pop(data->rb);
            }
            break;

        case READ_LENGTH:
            data->length = c;
            if (data->length == MESSAGE_LENGTH)
            {
                data->i = 2;
                data->state = READ_MESSAGE;
            }
            else
            {
                ULOG_ERR("Unexpected length (%d)\n", data->length);
                rb_pop(data->rb);
                rb_reset_iterator(data->rb);
                data->state = FIND_START;
            }
            break;

        case READ_MESSAGE:
            if (++data->i >= data->length - 1)
                data->state = VERIFY_CHECKSUM;
            break;

        case VERIFY_CHECKSUM:
            if (data->checksum == 0)
                consume_message(data->rb);
            else
            {
                ULOG_ERR("Wrong checksum (0x%02x)\n", data->checksum);
                rb_pop(data->rb);
            }
            rb_reset_iterator(data->rb);
            data->state = FIND_START;
            break;
        }
    }
}

static void state_cb(struct ustream *s)
{
    if (s->eof)
    {
        ULOG_ERR("tty error, shutting down\n");
        // TODO: is there a better way to handle this?
        exit(-1);
    }
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
