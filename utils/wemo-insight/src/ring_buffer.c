#include "ring_buffer.h"
#include <errno.h>
#include <stdlib.h>
#include <unistd.h>

struct ring_buffer
{
    int read;  // index where we read next
    int write; // index where we write next
    int it;    // iterator
    uint8_t buffer[64];
};

struct ring_buffer *rb_new(void)
{
    struct ring_buffer *rb = malloc(sizeof(struct ring_buffer));
    if (rb != NULL)
    {
        rb->read = 0;
        rb->write = 0;
        rb->it = 0;
    }
    return rb;
}

void rb_free(struct ring_buffer *rb)
{
    if (rb != NULL)
        free(rb);
}

// Pushes a byte to the ring buffer. Returns 1 if the ring buffer is full.
// Otherwise, 0 is returned.
int rb_push(struct ring_buffer *rb, uint8_t c)
{
    if ((rb->read - rb->write) % sizeof(rb->buffer) == 1)
        return 1;

    rb->buffer[rb->write] = c;
    rb->write = (rb->write + 1) % sizeof(rb->buffer);
    return 0;
}

// Pops a byte off the ring buffer and returns it. Returns -1 if the ring buffer
// is empty.
int rb_pop(struct ring_buffer *rb)
{
    if (rb->read == rb->write)
        return -1;

    uint8_t c = rb->buffer[rb->read];
    rb->read = (rb->read + 1) % sizeof(rb->buffer);
    return c;
}

// Gets a byte from the ring buffer without consuming it, i.e. advancing the
// read index. If no byte is available, new data will be read from fd. If that
// read fails or the ring buffer is full, -1 will be returned. Otherwise, 0 is
// returned.
int rb_getc(struct ring_buffer *rb, int fd)
{
    uint8_t c;
    if (rb->it == rb->write)
    {
        // TODO: Read as many bytes as there is space in the buffer
        ssize_t result = read(fd, &c, 1);
        if (result < 0)
            return -errno;
        if (result == 0)
            return -EIO;
        if (rb_push(rb, c))
            return -ENOMEM;
    }

    c = rb->buffer[rb->it];
    rb->it = (rb->it + 1) % sizeof(rb->buffer);
    return c;
}

// Resets the iterator by putting it back to the read index.
void rb_reset_iterator(struct ring_buffer *rb)
{
    rb->it = rb->read;
}
