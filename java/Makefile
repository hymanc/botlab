include ../src/common.mk

# Only add your directories to this folder if you are 100% sure that
# it will always compile without warnings.
SUBDIRS =

MAKEFLAGS += --no-print-directory

all:
	@$(MAKE) -f Makefile.lcmgen all || exit 2;

clean:
	@$(MAKE) -f Makefile.lcmgen clean || exit 2;
	@echo "clean [java/build]"; rm -rf build/*
