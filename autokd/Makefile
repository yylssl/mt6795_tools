CC=g++
CFLAGS=-I.

AUTOKD=autokd

all: $(AUTOKD)

.PHONY: clean

clean:
	rm -f $(AUTOKD) *.o

%.o: %.c
	$(CC) $(CFLAGS) -c -o $@ $^

%.o: %.cpp
	$(CC) $(CFLAGS) -c -o $@ $^

$(AUTOKD): autok_main.o node_ops.o nodes_data.o param_utility.o nvram_utility.o autok_flow.o uevent_utility.o
	$(CC) $(CFLAGS) -o $@ $^

