SUBDIRS = src java solns

MAKEFLAGS += --no-print-directory

all:
	@for dir in $(SUBDIRS) ; do \
	echo [$$dir] ; $(MAKE) -C $$dir all || exit 2; done

clean:
	@for dir in $(SUBDIRS); do \
	echo "clean [$$dir]" ; $(MAKE) -C $$dir clean || exit 2; done
	@rm -f *~
