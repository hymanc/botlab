include ../../common.mk

CFLAGS  =  $(CFLAGS_STD)  $(CFLAGS_MATH)  $(CFLAGS_COMMON)  $(CFLAGS_GSL) -DNDEBUG -Wno-unused-but-set-variable
LDFLAGS = $(LDFLAGS_STD) $(LDFLAGS_MATH) $(LDFLAGS_COMMON) $(LDFLAGS_GSL)
LIBDEPS = $(call libdeps, $(LDFLAGS))

BIN_GSLU_DEMO = gslu_demo
BIN_SO3_DEMO = so3_demo
BIN_SSC_DEMO = ssc_demo
BIN_HOMOGENOUS_DEMO = homogenous_demo
BIN_PLANE_DEMO = plane_demo
BIN_DIJKSTRA_DEMO = dijkstra_demo

ALL = $(BIN_GSLU_DEMO) $(BIN_SO3_DEMO) $(BIN_SSC_DEMO) $(BIN_HOMOGENOUS_DEMO) $(BIN_PLANE_DEMO) $(BIN_DIJKSTRA_DEMO)

all: $(ALL)

$(BIN_GSLU_DEMO): gslu_demo.o $(LIBDEPS)
	@echo "\t$@"
	@$(CC) -o $@ $^ $(LDFLAGS)

$(BIN_SO3_DEMO): so3_demo.o $(LIBDEPS)
	@echo "\t$@"
	@$(CC) -o $@ $^ $(LDFLAGS)

$(BIN_SSC_DEMO): ssc_demo.o $(LIBDEPS)
	@echo "\t$@"
	@$(CC) -o $@ $^ $(LDFLAGS)

$(BIN_HOMOGENOUS_DEMO): homogenous_demo.o $(LIBDEPS)
	@echo "\t$@"
	@$(CC) -o $@ $^ $(LDFLAGS)

$(BIN_PLANE_DEMO): plane_demo.o $(LIBDEPS)
	@echo "\t$@"
	@$(CC) -o $@ $^ $(LDFLAGS)

$(BIN_DIJKSTRA_DEMO): dijkstra_demo.o $(LIBDEPS)
	@echo "\t$@"
	@$(CC) -o $@ $^ $(LDFLAGS)

clean:
	@rm -f *.o *~ *.a
	@rm -f $(ALL)
