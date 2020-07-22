#ifndef RING_BUFFER_H_
#define RING_BUFFER_H_

#include <stdint.h>
#include <libubox/ustream.h>

struct ring_buffer;

struct ring_buffer *rb_new(void);
void rb_free(struct ring_buffer *rb);

int rb_push(struct ring_buffer *rb, uint8_t c);
int rb_pop(struct ring_buffer *rb);
int rb_getc(struct ring_buffer *rb, struct ustream *s);
void rb_reset_iterator(struct ring_buffer *rb);

#endif
