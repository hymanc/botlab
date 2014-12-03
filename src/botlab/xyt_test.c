#include "xyt.h"
#include "../math/gsl_util.h"

/**
 * Cheeseman Test!
 */
int main(int argc, char **argv)
{
    // 
    gsl_matrix *rbt = gsl_matrix_alloc(3,3);
    gsl_vector *X_ij = gsl_vector_alloc(3);
    gsl_vector_set(X_ij, 0, 1);
    gsl_vector_set(X_ij, 1, 2);
    gsl_vector_set(X_ij, 2, M_PI/2);
    xyt_rbt_gsl (rbt, X_ij);

    gslu_matrix_printf(rbt, "RBT");
    

    /*
    xyt_inverse (double X_ji[3], double J_minus[3*3], const double X_ij[3]);

    xyt_inverse_gsl (gsl_vector *X_ji, gsl_matrix *J_minus, const gsl_vector *X_ij);

    xyt_head2tail (double X_ik[3], double J_plus[3*6], const double X_ij[3], const double X_jk[3]);

    xyt_head2tail_gsl (gsl_vector *X_ik, gsl_matrix *J_plus, const gsl_vector *X_ij, const gsl_vector *X_jk);

    xyt_tail2tail (double X_jk[3], double J_tail[3*6], const double X_ij[3], const double X_ik[3]);

    xyt_tail2tail_gsl (gsl_vector *X_jk, gsl_matrix *J_tail, const gsl_vector *X_ij, const gsl_vector *X_ik);
    */
    return 0;
}