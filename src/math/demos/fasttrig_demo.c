#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>

#include "common/timestamp.h"
#include "math/gsl_util_rand.h"
#include "math/math_util.h"
#include "math/fasttrig.h"

void
test_trig (void)
{
    gsl_rng *r = gslu_rand_rng_alloc ();

    const double eps = 1. / (1<<17);
    const size_t n_trials = 2000000;

    printf ("test_trig(): n_trials=%zu, eps=%14.12f\n", n_trials, eps);
    printf ("     %20s : %15s %15s %15s %10s\n", "angle", "math", "fasttrig", "diff", "err/eps");

    struct max_err {
        double theta;
        double v_math;
        double v_fast;
        double err;
    } fs_max = {0}, fc_max = {0}, ft_max = {0};

    for (size_t i = 0; i < n_trials; i++) {
        double theta = gsl_rng_uniform (r) * 2. * M_PI;

        double s, c, fs, fc;
        sincos (theta, &s, &c);
        fsincos (theta, &fs, &fc);

        double t, ft;
        t = tan (theta);
        ft = ftan (theta);

        double fs_err = fabs (fs-s);
        if (fs_err > fs_max.err) {
            fs_max.theta = theta;
            fs_max.v_math = s;
            fs_max.v_fast = fs;
            fs_max.err = fs_err;
        }

        double fc_err = fabs (fc-c);
        if (fc_err > fc_max.err) {
            fc_max.theta = theta;
            fc_max.v_math = c;
            fc_max.v_fast = fc;
            fc_max.err = fc_err;
        }

        double ft_err = fabs (ft-t);
        if (ft_err > ft_max.err) {
            ft_max.theta = theta;
            ft_max.v_math = t;
            ft_max.v_fast = ft;
            ft_max.err = ft_err;
        }
    }
    printf("fsin max err %12.5f : %15.12f %15.12f %15.12f %10.5f\n",
           fs_max.theta * RTOD, fs_max.v_math, fs_max.v_fast, fs_max.err, fs_max.err/eps);
    printf("fcos max err %12.5f : %15.12f %15.12f %15.12f %10.5f\n",
           fc_max.theta * RTOD, fc_max.v_math, fc_max.v_fast, fc_max.err, fc_max.err/eps);
    printf("ftan max err %12.5f : %15.12f %15.12f %15.12f %10.5f\n\n",
           ft_max.theta * RTOD, fc_max.v_math, ft_max.v_fast, ft_max.err, ft_max.err/eps);
}

void
test_arctrig (void)
{
    gsl_rng *r = gslu_rand_rng_alloc ();

    const double eps = 1. / (1<<17);
    const size_t n_trials = 2000000;

    printf ("test_arctrig(): n_trials=%zu, eps=%14.12f\n", n_trials, eps);
    printf ("       %18s : %15s %15s %15s %10s\n", "arg(s)", "math", "fasttrig", "diff", "err/eps");

    struct max_err {
        double x;
        double y;
        double v_math;
        double v_fast;
        double err;
    } fas_max = {0}, fac_max = {0}, fat_max = {0};

    for (size_t i = 0; i < n_trials; i++) {
        double x = gsl_rng_uniform (r) * 2. - 1.;
        double y = gsl_rng_uniform (r) * 2. - 1.;

        double as = asin (x);
        double fas = fasin (x);
        double fas_err = fabs (fas-as);
        if (fas_err > fas_max.err) {
            fas_max.x = x;
            fas_max.y = y;
            fas_max.v_math = as;
            fas_max.v_fast = fas;
            fas_max.err = fas_err;
        }

        double ac = acos (x);
        double fac = facos (x);
        double fac_err = fabs (fac-ac);
        if (fac_err > fac_max.err) {
            fac_max.x = x;
            fac_max.y = y;
            fac_max.v_math = ac;
            fac_max.v_fast = fac;
            fac_max.err = fac_err;
        }

        double at = atan2 (y, x);
        double fat = fatan2 (y, x);
        double fat_err = fabs (fat-at);
        if (fat_err > fat_max.err) {
            fat_max.x = x;
            fat_max.y = y;
            fat_max.v_math = at;
            fat_max.v_fast = fat;
            fat_max.err = fat_err;
        }
    }
    printf ("fasin  max err  %10.5f : %15.10f %15.10f %15.12f %10.5f\n",
            fas_max.x, fas_max.v_math * RTOD, fas_max.v_fast * RTOD, fas_max.err * RTOD, fas_max.err/eps);
    printf ("facos  max err  %10.5f : %15.10f %15.10f %15.12f %10.5f\n",
            fac_max.x, fac_max.v_math * RTOD, fac_max.v_fast * RTOD, fac_max.err * RTOD, fac_max.err/eps);
    printf ("fatan2 max err  %10.5f : %15.10f %15.10f %15.12f %10.5f\n",
            fat_max.x, fat_max.v_math * RTOD, fat_max.v_fast * RTOD, fat_max.err * RTOD, fat_max.err/eps);
    printf ("                %10.5f\n\n", fat_max.y);
}

void
test_speed (void)
{
    // precomputed trial values so that it's not a part of timing
    const size_t n_trials = 2000000;
    double *theta = malloc (n_trials*sizeof(*theta));
    double *x = malloc (n_trials*sizeof(*x));
    double *y = malloc (n_trials*sizeof(*y));
    gsl_rng *r = gslu_rand_rng_alloc ();
    for (size_t i = 0; i < n_trials; i++) {
        theta[i] = gsl_rng_uniform (r) * 2. * M_PI;
        x[i] = gsl_rng_uniform (r) * 2. - 1.;
        y[i] = gsl_rng_uniform (r) * 2. - 1.;
    }

    int64_t t0, dt_math, dt_fast;
    double s, c, t, fs, fc, ft;
    printf ("test_speed(): n_trials=%zu\n", n_trials);
    printf ("         %15s %15s %15s\n", "math (us)", "fasttrig (us)", "speedup");

    // sincos
    t0 = utime_now ();
    for (size_t i = 0; i < n_trials; i++)
        sincos (theta[i], &s, &c);
    dt_math = utime_now() - t0;

    t0 = utime_now ();
    for (size_t i = 0; i < n_trials; i++)
        fsincos (theta[i], &fs, &fc);
    dt_fast = utime_now() - t0;
    printf ("fsincos  %15"PRId64" %15"PRId64" %15.2f\n",
            dt_math, dt_fast, ((double)dt_math)/dt_fast);

    // fsin
    t0 = utime_now ();
    for (size_t i = 0; i < n_trials; i++)
        s = sin (theta[i]);
    dt_math = utime_now() - t0;

    t0 = utime_now ();
    for (size_t i = 0; i < n_trials; i++)
        fs = fsin (theta[i]);
    dt_fast = utime_now() - t0;
    printf ("fsin     %15"PRId64" %15"PRId64" %15.2f\n",
            dt_math, dt_fast, ((double)dt_math)/dt_fast);

    // fcos
    t0 = utime_now ();
    for (size_t i = 0; i < n_trials; i++)
        c = cos (theta[i]);
    dt_math = utime_now() - t0;

    t0 = utime_now ();
    for (size_t i = 0; i < n_trials; i++)
        fc = fcos (theta[i]);
    dt_fast = utime_now() - t0;
    printf ("fcos     %15"PRId64" %15"PRId64" %15.2f\n",
            dt_math, dt_fast, ((double)dt_math)/dt_fast);

    // ftan
    t0 = utime_now ();
    for (size_t i = 0; i < n_trials; i++)
        t = tan (theta[i]);
    dt_math = utime_now() - t0;

    t0 = utime_now ();
    for (size_t i = 0; i < n_trials; i++)
        ft = ftan (theta[i]);
    dt_fast = utime_now() - t0;
    printf ("ftan     %15"PRId64" %15"PRId64" %15.2f\n",
            dt_math, dt_fast, ((double)dt_math)/dt_fast);

    double as, ac, at, fas, fac, fat;

    // fasin
    t0 = utime_now ();
    for (size_t i = 0; i < n_trials; i++)
        as = asin (x[i]);
    dt_math = utime_now() - t0;

    t0 = utime_now ();
    for (size_t i = 0; i < n_trials; i++)
        fas = fasin (x[i]);
    dt_fast = utime_now() - t0;
    printf ("fasin    %15"PRId64" %15"PRId64" %15.2f\n",
            dt_math, dt_fast, ((double)dt_math)/dt_fast);

    // facos
    t0 = utime_now ();
    for (size_t i = 0; i < n_trials; i++)
        ac = acos (x[i]);
    dt_math = utime_now() - t0;

    t0 = utime_now ();
    for (size_t i = 0; i < n_trials; i++)
        fac = facos (x[i]);
    dt_fast = utime_now() - t0;
    printf ("facos    %15"PRId64" %15"PRId64" %15.2f\n",
            dt_math, dt_fast, ((double)dt_math)/dt_fast);

    // fatan
    t0 = utime_now ();
    for (size_t i = 0; i < n_trials; i++)
        at = atan (x[i]);
    dt_math = utime_now() - t0;

    t0 = utime_now ();
    for (size_t i = 0; i < n_trials; i++)
        fat = fatan (x[i]);
    dt_fast = utime_now() - t0;
    printf ("fatan    %15"PRId64" %15"PRId64" %15.2f\n",
            dt_math, dt_fast, ((double)dt_math)/dt_fast);

    // fatan2
    t0 = utime_now ();
    for (size_t i = 0; i < n_trials; i++)
        at = atan2 (y[i], x[i]);
    dt_math = utime_now() - t0;

    t0 = utime_now ();
    for (size_t i = 0; i < n_trials; i++)
        fat = fatan2 (y[i], x[i]);
    dt_fast = utime_now() - t0;
    printf ("fatan2   %15"PRId64" %15"PRId64" %15.2f\nn",
            dt_math, dt_fast, ((double)dt_math)/dt_fast);

    // cleanup
    free (theta);
    free (x);
    free (y);
}


int
main (int argc, char *argv[])
{
    int64_t t0 = utime_now();
    fasttrig_init();
    int64_t dt = utime_now() - t0;
    printf ("Build time of fasttrig LUT %"PRId64" us\n\n", dt);

    test_trig();
    test_arctrig();
    test_speed();
}
