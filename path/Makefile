VTRLIB := vtrenderlib.a
CC := gcc
CFLAGS := -Wall -O2 -fPIC -I$(VTRLIB_PATH) -I.
BIN := vt-path

.PHONY: all clean

all: $(BIN)

$(BIN): main.o
	$(CC) $(LDFLAGS) $< -o $@ $(VTRLIB_PATH)/$(VTRLIB) -lm

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

clean:
	rm -f *.o $(BIN)
