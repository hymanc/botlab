include ../../common.mk

CFLAGS  =  $(CFLAGS_STD)  $(CFLAGS_MATH)  $(CFLAGS_COMMON)  $(CFLAGS_GSL) -DNDEBUG -Wno-unused-but-set-variable
LDFLAGS = $(LDFLAGS_STD) $(LDFLAGS_MATH) $(LDFLAGS_COMMON) $(LDFLAGS_GSL)

BIN_GSLU_DEMO = gslu_demo
BIN_FASTTRIG_DEMO = fasttrig_demo
BIN_SO3_DEMO = so3_demo
BIN_SSC_DEMO = ssc_demo
BIN_HOMOGENOUS_DEMO = homogenous_demo
BIN_PLANE_DEMO = plane_demo

ALL = $(BIN_GSLU_DEMO) $(BIN_FASTTRIG_DEMO) $(BIN_SO3_DEMO) $(BIN_SSC_DEMO) $(BIN_HOMOGENOUS_DEMO) $(BIN_PLANE_DEMO)

all: $(ALL)

$(BIN_GSLU_DEMO): gslu_demo.o
	@echo "\t$@"
	@$(CC) -o $@ $^ $(LDFLAGS)

$(BIN_FASTTRIG_DEMO): fasttrig_demo.o
	@echo "\t$@"
	@$(CC) -o $@ $^ $(LDFLAGS)

$(BIN_SO3_DEMO): so3_demo.o
	@echo "\t$@"
	@$(CC) -o $@ $^ $(LDFLAGS)

$(BIN_SSC_DEMO): ssc_demo.o
	@echo "\t$@"
	@$(CC) -o $@ $^ $(LDFLAGS)

$(BIN_HOMOGENOUS_DEMO): homogenous_demo.o
	@echo "\t$@"
	@$(CC) -o $@ $^ $(LDFLAGS)

$(BIN_PLANE_DEMO): plane_demo.o
	@echo "\t$@"
	@$(CC) -o $@ $^ $(LDFLAGS)

clean:
	@rm -f *.o *~ *.a
	@rm -f $(ALL)
