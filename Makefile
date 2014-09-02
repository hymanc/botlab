SUBDIRS = src java

MAKEFLAGS += --no-print-directory

all:
	@for dir in $(SUBDIRS) ; do \
	echo [$$dir] ; $(MAKE) -C $$dir all || exit 2; done
	@if test -d solns; then $(MAKE) -C solns all || exit 2; fi

clean:
	@for dir in $(SUBDIRS); do \
	echo "clean [$$dir]" ; $(MAKE) -C $$dir clean || exit 2; done
	@if test -d solns; then $(MAKE) -C solns clean || exit 2; fi
	@rm -f *~
