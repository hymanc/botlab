include ../src/common.mk

# Only add your directories to this folder if you are 100% sure that
# it will always compile without warnings.
SUBDIRS =

MAKEFLAGS += --no-print-directory

all:
	@echo "[rob550.jar]" ; $(MAKE) -f Makefile.lcmgen all || exit 2;

clean:
	@echo "clean [rob550.jar]" ; $(MAKE) -f Makefile.lcmgen clean || exit 2;
	@rm -rf build/*
