TARGET := wemo-insight
SRC := ring_buffer.c insight.c
OBJ := $(SRC:%.c=%.o)
LIBS :=

$(TARGET): $(OBJ)
	$(CC) $(LDFLAGS) -o $@ $^ $(LIBS)

%.o: %.c
	$(CC) $(CFLAGS) -c -o $@ $<

clean:
	rm -f $(TARGET) $(OBJ)

.PHONY: clean
