VTRLIB := vtrenderlib.a
CC := gcc
AR := ar
CFLAGS := -Wall -O2 -fPIC -I.
SRC := vtrenderlib.c
OBJ := $(SRC:.c=.o)
DEMOS := loh path cliptest cpuutil

.PHONY: all clean demos

all: $(VTRLIB) demos

$(VTRLIB): $(OBJ)
	$(AR) rcs $@ $^

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

demos:
	for d in $(DEMOS); do \
		$(MAKE) -C $$d VTRLIB_PATH=$(PWD); \
	done

clean:
	rm -f $(OBJ) $(VTRLIB)
	for d in $(DEMOS); do \
		$(MAKE) -C $$d clean; \
	done
