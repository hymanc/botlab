# Only add your directories to this folder if you are 100% sure that
# it will always compile without warnings.
SUBDIRS = lcmtypes common math math/demos imagesource vx vx/gtk apps rexarm maebot

MAKEFLAGS += --no-print-directory

all:
	@$(MAKE) -f Makefile.lcmgen all || exit 2;
	@for dir in $(SUBDIRS); do \
	echo "[$$dir]"; $(MAKE) -C $$dir all || exit 2; done

clean:
	@for dir in $(SUBDIRS); do \
	echo "clean [$$dir]"; $(MAKE) -C $$dir clean || exit 2; done
	@rm -f *~
