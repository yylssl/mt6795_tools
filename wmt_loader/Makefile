CC=gcc
CFLAGS=-I.

WMT_LOADER=wmt_loader

all: $(WMT_LOADER)

.PHONY: clean

clean:
	rm -f $(WMT_LOADER) *.o

%.o: %.c
	$(CC) $(CFLAGS) -c -o $@ $^

$(WMT_LOADER): loader.o
	$(CC) $(CFLAGS) -o $@ $^

