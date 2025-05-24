VTRLIB := vtrenderlib.a
CC := gcc
CFLAGS := -Wall -O0 -ggdb3 -I$(VTRLIB_PATH) -I.
BIN := vt-boids

.PHONY: all clean

all: $(BIN)

$(BIN): main.o
	$(CC) $(LDFLAGS) $< -o $@ $(VTRLIB_PATH)/$(VTRLIB) -lm

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

clean:
	rm -f *.o $(BIN)
