CC = gcc
CFLAGS = -O2 -Wall -Wextra -std=c99 -I../code/include
LDFLAGS = -lm

TARGET = tester
SRC = tester.c

all: $(TARGET)

$(TARGET): $(SRC)
	$(CC) $(CFLAGS) $(SRC) -o $(TARGET) $(LDFLAGS)

clean:
	rm -f $(TARGET) telemetry.log logs/*.log

.PHONY: all clean
