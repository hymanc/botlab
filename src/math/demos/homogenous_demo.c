#include <stdio.h>
#include <stdlib.h>
#include <gsl/gsl_matrix.h>

#include "math/gsl_util_matrix.h"
#include "math/homogenous.h"

int
main (int argc, char *argv[])
{
    gsl_matrix *uv = gsl_matrix_alloc (3, 5);
    for (size_t i=0; i<gslu_matrix_numel(uv); i++)
        uv->data[i] = i;
    gsl_matrix *uv_h = homogenize_alloc (uv);
    gsl_matrix *uv_back = dehomogenize_alloc (uv_h);

    gslu_matrix_printf (uv, "\nuv");
    gslu_matrix_printf (uv_h, "\nuv_h");
    gslu_matrix_printf (uv_back, "\nuv_back");

    gsl_matrix_free (uv);
    gsl_matrix_free (uv_h);
    gsl_matrix_free (uv_back);

    return 0;
}
