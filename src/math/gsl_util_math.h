#ifndef __MATH_GSL_UTIL_MATH_H__
#define __MATH_GSL_UTIL_MATH_H__

#include <gsl/gsl_math.h>

/**
 * @defgroup PerlsMathGsluMath GSL Util Math
 * @brief GSL utility math.
 * @ingroup PerlsMath
 *
 * @{
 */

#ifdef __cplusplus
extern "C" {
#endif

#ifndef DTOR
#define DTOR (M_PI/180.)
#endif

#ifndef RTOD
#define RTOD (180./M_PI)
#endif

/** wrap circular quantities between [-pi,pi]*/
static inline double
gslu_math_mod2pi (double angle)
{
    while (angle < -M_PI)
        angle += 2*M_PI;
    while (angle > M_PI)
        angle -= 2*M_PI;
    return angle;
}



#ifdef __cplusplus
}
#endif

/**
 * @}
 */

#endif // __MATH_GSL_UTIL_MATH_H__
