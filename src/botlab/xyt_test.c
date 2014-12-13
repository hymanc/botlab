#include "xyt.h"
#include "../math/gsl_util.h"

/**
 * Cheeseman Test!
 */
int main(int argc, char **argv)
{
     
    gsl_matrix *rbt = gsl_matrix_alloc(3,3);
    gsl_vector *X_ij = gsl_vector_alloc(3);
    gsl_vector_set(X_ij, 0, 1);
    gsl_vector_set(X_ij, 1, 2);
    gsl_vector_set(X_ij, 2, M_PI/2);

    xyt_rbt_gsl (rbt, X_ij);

    gslu_matrix_printf(rbt, "RBT");
    
    gsl_vector *v1 = gsl_vector_alloc(3);
    gsl_vector *v2 = gsl_vector_alloc(3);
    gsl_vector *v3 = gsl_vector_alloc(3);
    gsl_vector *v4 = gsl_vector_alloc(3);
    gsl_vector *v5 = gsl_vector_alloc(3);
    
    v1->data = (double[3]) {0,0,M_PI/2};
    v2->data = (double[3]) {1,2,0};
    xyt_head2tail_gsl(v3 , NULL, v1, v2);
    gslu_vector_printf(v3, "HeadToHead1");
    xyt_head2tail_gsl(v5, NULL, v2, v1);
    gslu_vector_printf(v5, "HeadToHead2");
    
    xyt_tail2tail_gsl(v4, NULL, v3, v3);
    gslu_vector_printf(v4, "TailToTail");
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
