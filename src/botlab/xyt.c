#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

//#include "math/gsl_util_vector.h"
#include "math/gsl_util_matrix.h"
#include "math/gsl_util_blas.h"

//#include "math/gsl_util.h"

#include "xyt.h"

int xyt_rbt (double T[3*3], const double X_ij[3]) {   // this is a 3x3 matrix
    T[0] = cos(X_ij[2]);
    T[1] = -sin(X_ij[2]);
    T[2] = X_ij[0];
    T[3] = sin(X_ij[2]);
    T[4] = cos(X_ij[2]);
    T[5] = X_ij[1];
    T[6] = 0;
    T[7] = 0;
    T[8] = 1;
    return GSL_SUCCESS;
}

int xyt_rbt_gsl (gsl_matrix *T, const gsl_vector *X_ij) {
    assert (T->size1 == 3 && T->size2 == 3);
    assert (X_ij->size == 3 && X_ij->stride == 1);

    return xyt_rbt (T->data, X_ij->data);
}


int xyt_inverse (double X_ji[3], double J_minus[3*3], const double X_ij[3]) {
    double xij = X_ij[0];
    double yij = X_ij[1];
    double tij = X_ij[2];
    X_ji[0] = -xij*cos(tij) - yij*sin(tij);
    X_ji[1] = xij*sin(tij) - yij*cos(tij);
    X_ji[2] = -tij;
    if (J_minus != NULL) 
    {
	// Row 1
	J_minus[0] = -cos(tij);
	J_minus[1] = -sin(tij);
	J_minus[2] = X_ji[1];
	// Row 2
	J_minus[3] = sin(tij);
	J_minus[4] = -cos(tij);
	J_minus[5] = -X_ji[0];
	// Row 3
	J_minus[6] = 0;
	J_minus[7] = 0;
	J_minus[8] = 1;
    }
    return GSL_SUCCESS;
}

int xyt_inverse_gsl (gsl_vector *X_ji, gsl_matrix *J_minus, const gsl_vector *X_ij)
{
    assert (X_ji->size == 3 && X_ji->stride == 1);
    assert (X_ij->size == 3 && X_ij->stride == 1);

    if (J_minus) {
        assert (J_minus->size1 == 3 && J_minus->size2 == 3 && J_minus->tda == 3);
        return xyt_inverse (X_ji->data, J_minus->data, X_ij->data);
    }
    else
        return xyt_inverse (X_ji->data, NULL, X_ij->data);
}

int xyt_head2tail (double X_ik[3], double J_plus[3*6], const double X_ij[3], const double X_jk[3])
{
    double xij = X_ij[0];
    double yij = X_ij[1];
    double tij = X_ij[2];
    double xjk = X_jk[0];
    double yjk = X_jk[1];
    double tjk = X_jk[2];
    
    X_ik[0] = xjk*cos(tij) - yjk*sin(tij) + xij;
    X_ik[1] = xjk*sin(X_ij[2]) - yjk*cos(tij) + yij;
    X_ik[2] = tij + tjk;
   
    if (J_plus != NULL) 
    {
	// Row 1
        J_plus[0] = 1;
	J_plus[1] = 0;
	J_plus[2] = -xjk*sin(tij) - yjk*cos(tij);
	J_plus[3] = cos(tij);
	J_plus[4] = -sin(tij);
	J_plus[5] = 0;
	// Row 2
	J_plus[6] = 0;
	J_plus[7] = 1;
	J_plus[8] = xjk*cos(tij) - yjk*sin(tij);
	J_plus[9] = sin(tij);
	J_plus[10] = cos(tij);
	J_plus[11] = 0;
	// Row 3
	J_plus[12] = 0;
	J_plus[13] = 0;
	J_plus[14] = 1;
	J_plus[15] = 0;
	J_plus[16] = 0;
	J_plus[17] = 1;
    }
    return GSL_SUCCESS;
}

int xyt_head2tail_gsl (gsl_vector *X_ik, gsl_matrix *J_plus, const gsl_vector *X_ij, const gsl_vector *X_jk)
{
    assert (X_ik->size == 3 && X_ik->stride == 1);
    assert (X_ij->size == 3 && X_ij->stride == 1);
    assert (X_jk->size == 3 && X_jk->stride == 1);

    if (J_plus) {
        assert (J_plus->size1 == 3 && J_plus->size2 == 6 && J_plus->tda == 6);
        return xyt_head2tail (X_ik->data, J_plus->data, X_ij->data, X_jk->data);
    }
    else
        return xyt_head2tail (X_ik->data, NULL, X_ij->data, X_jk->data);
}


/**
 * @brief (-)xij(+)xik
 */
int xyt_tail2tail (double X_jk[3], double J_tail[3*6], const double X_ij[3], const double X_ik[3])
{
    double X_ji[3];
    if (J_tail == NULL) 
    {
        xyt_inverse (X_ji, NULL, X_ij);
	xyt_head2tail(X_jk, NULL, X_ji, X_ik);
    }
    else 
    {
	xyt_inverse (X_ji, NULL, X_ij);
	xyt_head2tail(X_jk, J_tail, X_ji, X_ik);
    }
    return GSL_SUCCESS;
}

int xyt_tail2tail_gsl (gsl_vector *X_jk, gsl_matrix *J_tail, const gsl_vector *X_ij, const gsl_vector *X_ik)
{
    assert (X_jk->size == 3 && X_jk->stride == 1);
    assert (X_ij->size == 3 && X_ij->stride == 1);
    assert (X_ik->size == 3 && X_ik->stride == 1);

    if (J_tail) {
        assert (J_tail->size1 == 3 && J_tail->size2 == 6 && J_tail->tda == 6);
        return xyt_tail2tail (X_jk->data, J_tail->data, X_ij->data, X_ik->data);
    }
    else
        return xyt_tail2tail (X_jk->data, NULL, X_ij->data, X_ik->data);
}
