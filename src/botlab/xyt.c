#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "math/gsl_util_vector.h"
#include "math/gsl_util_matrix.h"
#include "math/gsl_util_blas.h"

#include "xyt.h"

int
xyt_rbt (double T[3*3], const double X_ij[3])
{
    // IMPLEMENT ME
    return GSL_SUCCESS;
}

int
xyt_rbt_gsl (gsl_matrix *T, const gsl_vector *X_ij)
{
    assert (T->size1 == 3 && T->size2 == 3);
    assert (X_ij->size == 3 && X_ij->stride == 1);

    return xyt_rbt (T->data, X_ij->data);
}


int
xyt_inverse (double X_ji[3], double J_minus[3*3], const double X_ij[3])
{
    // IMPLEMENT ME

    if (J_minus != NULL) {
        // IMPLEMENT ME
    }
    return GSL_SUCCESS;
}

int
xyt_inverse_gsl (gsl_vector *X_ji, gsl_matrix *J_minus, const gsl_vector *X_ij)
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

int
xyt_head2tail (double X_ik[3], double J_plus[3*6], const double X_ij[3], const double X_jk[3])
{
    // IMPLEMENT ME

    if (J_plus != NULL) {
        // IMPLEMENT ME
    }
    return GSL_SUCCESS;
}

int
xyt_head2tail_gsl (gsl_vector *X_ik, gsl_matrix *J_plus, const gsl_vector *X_ij, const gsl_vector *X_jk)
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


int
xyt_tail2tail (double X_jk[3], double J_tail[3*6], const double X_ij[3], const double X_ik[3])
{
    if (J_tail == NULL) {
        // IMPLEMENT ME
    }
    else {
        // IMPLEMENT ME
    }
    return GSL_SUCCESS;
}

int
xyt_tail2tail_gsl (gsl_vector *X_jk, gsl_matrix *J_tail, const gsl_vector *X_ij, const gsl_vector *X_ik)
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
