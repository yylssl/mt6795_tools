CC=gcc
CFLAGS=-I.

6620_LAUNCHER=6620_launcher

all: $(6620_LAUNCHER)

.PHONY: clean

clean:
	rm -f $(6620_LAUNCHER) *.o

%.o: %.c
	$(CC) $(CFLAGS) -c -o $@ $^

$(6620_LAUNCHER): 6620_launcher.o
	$(CC) $(CFLAGS) -o $@ $^

