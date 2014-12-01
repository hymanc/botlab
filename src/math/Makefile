include ../common.mk

CFLAGS = $(CFLAGS_STD) $(CFLAGS_COMMON) $(CFLAGS_GSL) -O4 -DNDEBUG
LDFLAGS = $(LDFLAGS_STD) $(LDFLAGS_COMMON) $(LDFLAGS_GSL)
LIBDEPS = $(call libdeps, $(LDFLAGS))

LIB_MATH = $(LIB_PATH)/libmath.a
LIBMATH_OBJS = \
	april_graph.o \
	dm.o \
	dijkstra.o \
	exact_minimum_degree.o \
	gps_linearize.o \
	gsl_util_array.o \
	gsl_util_blas.o \
	gsl_util_boolean.o \
	gsl_util_eigen.o \
	gsl_util_index.o \
	gsl_util_linalg.o \
	gsl_util_math.o \
	gsl_util_matrix.o \
	gsl_util_rand.o \
	gsl_util_statistics.o \
	gsl_util_vector.o \
	homogenous.o \
	homography.o \
	matd.o \
	math_util.o \
	plane.o \
	ransac.o \
	smatd.o \
	ssc.o \
	svd22.o \
	unscented_transform.o

ALL = $(LIB_MATH)

all: $(ALL)

$(LIB_MATH): $(LIBMATH_OBJS) $(LIBDEPS)
	@echo "\t$@"
	@ar rc $@ $^

clean:
	@rm -f *.o *~ *.a
	@rm -f $(ALL)
