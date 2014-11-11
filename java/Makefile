include ../src/common.mk

# Only add your directories to this folder if you are 100% sure that
# it will always compile without warnings.
SUBDIRS =

MAKEFLAGS += --no-print-directory

ROB550_JAR = $(JAVA_PATH)/rob550.jar
BUILD_PATH = build

all:
	@echo "[lcmtypes]" ; $(MAKE) -f Makefile.lcmgen all || exit 2;
	@echo "$(ROB550_JAR)"
	@jar cf $(ROB550_JAR) -C $(BUILD_PATH) .

clean:
	@echo "clean [lcmtypes]" ; $(MAKE) -f Makefile.lcmgen clean || exit 2;
	@rm -rf build/*
	@rm -f $(ROB550_JAR)
