include ../common.mk

CFLAGS  =  $(CFLAGS_STD)  $(CFLAGS_COMMON)  $(CFLAGS_MATH)  $(CFLAGS_LCMTYPES) -O4
LDFLAGS = $(LDFLAGS_STD) $(LDFLAGS_COMMON) $(LDFLAGS_MATH) $(LDFLAGS_LCMTYPES)

DYNAMIXEL_OBJS = \
	dynamixel_axseries.o \
	dynamixel_mxseries.o \
	dynamixel_bus.o \
	dynamixel_device.o \
	dynamixel_serial_bus.o

BIN_DYNAMIXEL_TEST = $(BIN_PATH)/dynamixel_test
BIN_REXARM_DRIVER = $(BIN_PATH)/rexarm_driver
BIN_REXARM_EXAMPLE = $(BIN_PATH)/rexarm_example

ALL = $(BIN_DYNAMIXEL_TEST) $(BIN_REXARM_DRIVER) $(BIN_REXARM_EXAMPLE)

all: $(ALL)

$(BIN_DYNAMIXEL_TEST): dynamixel_test.o $(DYNAMIXEL_OBJS)
	@echo "\t$@"
	@$(CC) -o $@ $^ $(LDFLAGS)

$(BIN_REXARM_DRIVER): rexarm_driver.o $(DYNAMIXEL_OBJS)
	@echo "\t$@"
	@$(CC) -o $@ $^ $(LDFLAGS)

$(BIN_REXARM_EXAMPLE): rexarm_example.o $(DYNAMIXEL_OBJS)
	@echo "\t$@"
	@$(CC) -o $@ $^ $(LDFLAGS)

clean:
	@rm -f *.o *~ *.a
	@rm -f $(ALL)
