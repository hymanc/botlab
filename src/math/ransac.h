#ifndef __MATH_RANSAC_H__
#define __MATH_RANSAC_H__

#ifdef __cplusplus
extern "C" {
#endif

/*
  %ADAPT_TRIALS  Number of trials required to achieve an outlier-free sample set.
  %  N = ADAPT_TRIALS(n_inlier,n_total,p,s) calculates number of
  %  trials needed to get a outlier-free sample set of size s with
  %  probability p from a set of n_total measurements.
  %
  %  Reference: Algorithm 3.5 Hartley/Zisserman
*/
size_t
ransac_adapt_trials (size_t n_inlier, size_t n_total, double p, size_t s);

#ifdef __cplusplus
}
#endif

#endif //__MATH_RANSAC_H__
