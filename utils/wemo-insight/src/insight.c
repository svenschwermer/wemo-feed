#include <assert.h>
#include <errno.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <pthread.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>
#include <linux/serial.h>
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

struct insight_state
{
    struct ustream_fd s_fd;
    struct ring_buffer *rb;
    enum fsm_state state;
    uint8_t length, i, checksum;
    pthread_rwlock_t data_lock;
    struct insight_data data;
};

static void read_cb(struct ustream *s, int bytes_new);
static void state_cb(struct ustream *s);
static void consume_message(struct insight_state *s);
static float get_word(struct ring_buffer *rb, float factor);

static const speed_t BAUDRATE = B9600;
static const uint8_t START_BYTE = 0xAE;
static const uint8_t MESSAGE_LENGTH = 30;

struct insight_state *insight_open(const char *dev)
{
    struct insight_state *s = malloc(sizeof(struct insight_state));
    if (s == NULL)
    {
        ULOG_ERR("Failed to allocate insight state: %s\n", strerror(errno));
        goto err_exit;
    }
    memset(s, 0, sizeof(*s));
    s->rb = rb_new();
    if (s->rb == NULL)
    {
        ULOG_ERR("Failed to allocate ring buffer: %s\n", strerror(errno));
        goto free_data;
    }
    s->state = FIND_START;

    int ret = pthread_rwlock_init(&s->data_lock, NULL);
    if (ret)
    {
        ULOG_ERR("Failed to init data lock: %s\n", strerror(ret));
        goto free_data;
    }

    int fd = open(dev, O_RDONLY | O_NOCTTY);
    if (fd < 0)
    {
        ULOG_ERR("Failed to open serial port %s: %s\n", dev, strerror(errno));
        goto destroy_lock;
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

    s->s_fd.stream.string_data = true;
    s->s_fd.stream.notify_read = read_cb;
    s->s_fd.stream.notify_state = state_cb;
    ustream_fd_init(&s->s_fd, fd);

    tcflush(fd, TCIFLUSH);

    return s;

close_fd:
    close(fd);
destroy_lock:
    pthread_rwlock_destroy(&s->data_lock);
free_data:
    if (s->rb != NULL)
        rb_free(s->rb);
    free(s);
err_exit:
    return NULL;
}

int insight_free(struct insight_state *s)
{
    close(s->s_fd.fd.fd);
    pthread_rwlock_destroy(&s->data_lock);
    ustream_free(&s->s_fd.stream);
    rb_free(s->rb);
    free(s);
    return 0;
}

const struct insight_data *insight_borrow_data(struct insight_state *s)
{
    int ret = pthread_rwlock_rdlock(&s->data_lock);
    assert(ret == 0);
    return &s->data;
}

void insight_return_data(struct insight_state *s)
{
    int ret = pthread_rwlock_unlock(&s->data_lock);
    assert(ret == 0);
}

static void read_cb(struct ustream *stream, int bytes_new)
{
    struct ustream_fd *s_fd = container_of(stream, struct ustream_fd, stream);
    struct insight_state *s = container_of(s_fd, struct insight_state, s_fd);

    while (bytes_new-- > 0)
    {
        int c = rb_getc(s->rb, stream);
        if (c < 0)
        {
            ULOG_ERR("Failed to read byte: %s\n", strerror(-c));
            break;
        }
        s->checksum += c;

        switch (s->state)
        {
        case FIND_START:
            if (c == START_BYTE)
            {
                s->checksum = c;
                s->state = READ_LENGTH;
            }
            else
            {
                ULOG_WARN("Discarding non-start byte (0x%02x)\n", c);
                rb_pop(s->rb);
            }
            break;

        case READ_LENGTH:
            s->length = c;
            if (s->length == MESSAGE_LENGTH)
            {
                s->i = 2;
                s->state = READ_MESSAGE;
            }
            else
            {
                ULOG_ERR("Unexpected length (%d)\n", s->length);
                rb_pop(s->rb);
                rb_reset_iterator(s->rb);
                s->state = FIND_START;
            }
            break;

        case READ_MESSAGE:
            if (++s->i >= s->length - 1)
                s->state = VERIFY_CHECKSUM;
            break;

        case VERIFY_CHECKSUM:
            if (s->checksum == 0)
                consume_message(s);
            else
            {
                ULOG_ERR("Wrong checksum (0x%02x)\n", s->checksum);
                rb_pop(s->rb);
            }
            rb_reset_iterator(s->rb);
            s->state = FIND_START;
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

static void consume_message(struct insight_state *s)
{
    rb_pop(s->rb); // start byte
    rb_pop(s->rb); // length byte

    int ret = pthread_rwlock_wrlock(&s->data_lock);
    assert(ret == 0);

    s->data.int_temperature = get_word(s->rb, 0.001);
    s->data.ext_temperature = get_word(s->rb, 0.001);
    s->data.rms_voltage = get_word(s->rb, 0.001);
    s->data.rms_current = get_word(s->rb, 0.001 / 128);
    s->data.active_power = get_word(s->rb, 0.005);
    s->data.average_power = get_word(s->rb, 0.005);
    s->data.power_factor = get_word(s->rb, 0.001);
    s->data.line_frequency = get_word(s->rb, 0.001);
    s->data.active_energy = get_word(s->rb, 1); // factor unknown
    
    ret = pthread_rwlock_unlock(&s->data_lock);
    assert(ret == 0);

    rb_pop(s->rb); // checksum
}

void print_data(const struct insight_state *s)
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
        s->data.int_temperature, s->data.ext_temperature,
        s->data.rms_voltage, s->data.rms_current,
        s->data.active_power, s->data.average_power,
        s->data.power_factor, s->data.line_frequency,
        s->data.active_energy);
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
