#ifndef __XYT_H__
#define __XYT_H__

#include <gsl/gsl_matrix.h>
#include <gsl/gsl_vector.h>

#ifdef __cplusplus
extern "C" {
#endif

int xyt_rbt (double T[3*3], const double X_ij[3]);

int xyt_rbt_gsl (gsl_matrix *T, const gsl_vector *X_ij);

int xyt_inverse (double X_ji[3], double J_minus[3*3], const double X_ij[3]);

int xyt_inverse_gsl (gsl_vector *X_ji, gsl_matrix *J_minus, const gsl_vector *X_ij);

int xyt_head2tail (double X_ik[3], double J_plus[3*6], const double X_ij[3], const double X_jk[3]);

int xyt_head2tail_gsl (gsl_vector *X_ik, gsl_matrix *J_plus, const gsl_vector *X_ij, const gsl_vector *X_jk);

int xyt_tail2tail (double X_jk[3], double J_tail[3*6], const double X_ij[3], const double X_ik[3]);

int xyt_tail2tail_gsl (gsl_vector *X_jk, gsl_matrix *J_tail, const gsl_vector *X_ij, const gsl_vector *X_ik);


#ifdef __cplusplus
}
#endif

#endif //__XYT_H__
