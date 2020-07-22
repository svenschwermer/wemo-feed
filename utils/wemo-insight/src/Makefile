TARGET := wemo-insight
SRC := main.c insight.c ring_buffer.c
OBJ := $(SRC:%.c=%.o)
LIBS := -lubox -lubus

$(TARGET): $(OBJ)
	$(CC) $(LDFLAGS) -o $@ $^ $(LIBS)

%.o: %.c
	$(CC) $(CFLAGS) -c -o $@ $<

clean:
	rm -f $(TARGET) $(OBJ)

.PHONY: clean
