# Only add your directories to this folder if you are 100% sure that
# it will always compile without warnings.
SUBDIRS =

MAKEFLAGS += --no-print-directory

all:
	@echo [lcmtypes] ; $(MAKE) -f Makefile.lcmgen all || exit 2;

clean:
	@echo clean [lcmtypes] ; $(MAKE) -f Makefile.lcmgen clean || exit 2;
