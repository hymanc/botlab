include ../common.mk

CFLAGS = -g -Wall -std=gnu99 $(CFLAGS_LCMTYPES)
LDFLAGS = $(LDFLAGS_LCMTYPES)

BIN_PIXY_DRIVER = $(BIN_PATH)/pixy_driver

ALL = $(BIN_PIXY_DRIVER)

all: $(ALL)

$(BIN_PIXY_DRIVER): pixy_driver.o
	@echo "    $@"
	@$(CC) -o $@ $^

clean:
	@rm -f *.o *~ *.a
	@rm -f $(ALL)
