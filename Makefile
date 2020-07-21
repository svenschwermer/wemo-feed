SRC := ring_buffer.c insight.c
OBJ := $(SRC:%.c=%.o)
CFLAGS := -Wall -Wextra -pedantic
LDFLAGS := -s

insight: $(OBJ)
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $^

%.o: %.c
	$(CC) $(CFLAGS) -c -o $@ $<

clean:
	rm -f insight $(OBJ)

.PHONY: clean
