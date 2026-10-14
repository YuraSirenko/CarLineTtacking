CC ?= cc
CFLAGS ?= -std=c99 -Wall -Wextra -Wpedantic -Werror -O2
CPPFLAGS += -Iinclude
LDLIBS += -lm

LIB_SRC := src/pid.c src/line_sensor.c src/line_follower.c

.PHONY: all test plot clean

all: build/sim build/unit_tests build/mcu_main.o

build:
	mkdir -p build

build/sim: $(LIB_SRC) sim/sim.c | build
	$(CC) $(CPPFLAGS) $(CFLAGS) $^ -o $@ $(LDLIBS)

build/unit_tests: $(LIB_SRC) tests/test_main.c | build
	$(CC) $(CPPFLAGS) $(CFLAGS) $^ -o $@ $(LDLIBS)

build/mcu_main.o: examples/mcu_main.c | build
	$(CC) $(CPPFLAGS) $(CFLAGS) -c $< -o $@

test: all
	./build/unit_tests
	./build/sim

plot: build/sim
	./build/sim --csv > build/run.csv
	python3 tools/plot.py build/run.csv docs/run.png

clean:
	rm -rf build
