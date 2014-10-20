include ../common.mk

CFLAGS  =  $(CFLAGS_STD)  $(CFLAGS_COMMON)  $(CFLAGS_LCMTYPES) -O4
LDFLAGS = $(LDFLAGS_STD) $(LDFLAGS_COMMON) $(LDFLAGS_LCMTYPES)
LIBDEPS = $(call libdeps, $(LDFLAGS))

BIN_MAEBOT_DRIVER = $(BIN_PATH)/maebot_driver
BIN_RPLIDAR_DRIVER = $(BIN_PATH)/maebot_rplidar_driver
BIN_LED_TEST = $(BIN_PATH)/maebot_led_test
BIN_LASER_TEST = $(BIN_PATH)/maebot_laser_test
BIN_MOTOR_FEEDBACK_TEST = $(BIN_PATH)/maebot_motor_feedback_test
BIN_DIFF_DRIVE_TEST = $(BIN_PATH)/maebot_diff_drive_test
BIN_SENSOR_DATA_TEST = $(BIN_PATH)/maebot_sensor_data_test

ALL = $(BIN_MAEBOT_DRIVER) $(BIN_LED_TEST) $(BIN_LASER_TEST) $(BIN_MOTOR_FEEDBACK_TEST) \
      $(BIN_DIFF_DRIVE_TEST) $(BIN_SENSOR_DATA_TEST) $(BIN_RPLIDAR_DRIVER)

all: $(ALL)

$(BIN_MAEBOT_DRIVER): maebot_driver.o types.o $(LIBDEPS)
	@echo "\t$@"
	@$(CC) -o $@ $^ $(LDFLAGS)

$(BIN_RPLIDAR_DRIVER): rplidar_driver.o rplidar.o $(LIBDEPS)
	@echo "\t$@"
	@$(CC) -o $@ $^ $(LDFLAGS)

$(BIN_LED_TEST): led_test.o $(LIBDEPS)
	@echo "\t$@"
	@$(CC) -o $@ $^ $(LDFLAGS)

$(BIN_LASER_TEST): laser_test.o $(LIBDEPS)
	@echo "\t$@"
	@$(CC) -o $@ $^ $(LDFLAGS)

$(BIN_MOTOR_FEEDBACK_TEST): motor_feedback_test.o $(LIBDEPS)
	@echo "\t$@"
	@$(CC) -o $@ $^ $(LDFLAGS)

$(BIN_DIFF_DRIVE_TEST): diff_drive_test.o $(LIBDEPS)
	@echo "\t$@"
	@$(CC) -o $@ $^ $(LDFLAGS)

$(BIN_SENSOR_DATA_TEST): sensor_data_test.o $(LIBDEPS)
	@echo "\t$@"
	@$(CC) -o $@ $^ $(LDFLAGS)

clean:
	@rm -f *.o *~ *.a
	@rm -f $(ALL)
