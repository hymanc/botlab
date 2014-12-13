include ../common.mk

# flags for building the gtk library
CFLAGS = $(CFLAGS_STD) \
	 $(CFLAGS_VX_GTK) \
         $(CFLAGS_IMAGESOURCE) \
	 $(CFLAGS_MATH) \
	 $(CFLAGS_COMMON) \
	 $(CFLAGS_LCMTYPES) \
	 -O2

LDFLAGS = $(LDFLAGS_STD) \
	  $(LDFLAGS_VX_GTK) \
	  $(LDFLAGS_IMAGESOURCE) \
	  $(LDFLAGS_MATH) \
	  $(LDFLAGS_COMMON) \
	  $(LDFLAGS_LCMTYPES)

LIBDEPS = $(call libdeps, $(LDFLAGS))

BIN_BOTLAB_ODOMETRY 			= $(BIN_PATH)/botlab_odometry
BIN_BOTLAB_APP   				= $(BIN_PATH)/botlab_app
BIN_BOTLAB_CAMERA_LIDAR  		= $(BIN_PATH)/botlab_camera_lidar
BIN_BOTLAB_XYT_TEST 			= $(BIN_PATH)/xyt_test
BIN_BOTLAB_MAEBOT_STRAIGHT_LINE = $(BIN_PATH)/maebot_straight_line
BIN_BOTLAB_GYRO_CAL 			= $(BIN_PATH)/gyroCal
BIN_BOTLAB_LOG_CONVERTER    	= $(BIN_PATH)/logConverter
BIN_BOTLAB_GYRO_TEST 			= $(BIN_PATH)/gyroTest

ALL = $(BIN_BOTLAB_ODOMETRY) $(BIN_BOTLAB_APP) \
$(BIN_BOTLAB_XYT_TEST) $(BIN_BOTLAB_MAEBOT_STRAIGHT_LINE) \
$(BIN_BOTLAB_GYRO_CAL) $(BIN_BOTLAB_GYRO_TEST) $(BIN_BOTLAB_LOG_CONVERTER) \
$(BIN_BOTLAB_CAMERA_LIDAR) \

all: $(ALL)

xyt_test: $(BIN_BOTLAB_XYT_TEST)


$(BIN_BOTLAB_ODOMETRY): odometry.o xyt.o $(LIBDEPS)
	@echo "\t$@"
	@$(CC) -o $@ $^ $(LDFLAGS)

$(BIN_BOTLAB_APP): botlab.o xyt.o $(LIBDEPS)
	@echo "\t$@"
	@$(CC) -o $@ $^ $(LDFLAGS)

$(BIN_BOTLAB_CAMERA_LIDAR): camera_lidar.o $(LIBDEPS)
	@echo "\t$@"
	@$(CC) -o $@ $^ $(LDFLAGS)

$(BIN_BOTLAB_XYT_TEST): xyt_test.o xyt.o $(LIBDEPS)
	@echo "\t$@"
	@$(CC) -o $@ $^ $(LDFLAGS)
	
$(BIN_BOTLAB_MAEBOT_STRAIGHT_LINE): maebot_straight_line.o $(LIBDEPS)
	@echo "\t$@"
	@$(CC) -o $@ $^ $(LDFLAGS)

$(BIN_BOTLAB_GYRO_CAL): gyroCal.o $(LIBDEPS)
	@echo "\t$@"
	@$(CC) -o $@ $^ $(LDFLAGS)

$(BIN_BOTLAB_GYRO_TEST): gyroTest.o $(LIBDEPS)
	@echo "\t$@"
	@$(CC) -o $@ $^ $(LDFLAGS)

$(BIN_BOTLAB_LOG_CONVERTER): logConverter.o $(LIBDEPS)
	@echo "\t$@"
	@$(CC) -o $@ $^ $(LDFLAGS)

clean:
	@rm -f *.o *~ *.a
	@rm -f $(ALL)
