#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <inttypes.h>

#include "common/timestamp.h"

#include "math/math_util.h"
#include "math/gsl_util_vector.h"
#include "math/gsl_util_matrix.h"
#include "math/ssc.h"

int
main (int argc, char *argv[])
{
    GSLU_VECTOR_VIEW (x_ij, 6, {1, 2, 3, 4, 5, 6});
    gslu_vector_printfc (&x_ij.vector, "x_ij", NULL, CblasTrans);

    GSLU_VECTOR_VIEW (x_jk, 6, {6, 5, 4, 3, 2, 1});
    gslu_vector_printfc (&x_jk.vector, "x_jk", NULL, CblasTrans);

    // ssc_inverse
    printf ("\nssc_inverse: x_ji = (-)x_ij\n");
    GSLU_VECTOR_VIEW (x_ji, 6);
    GSLU_MATRIX_VIEW (Jminus, 6, 6);

    ssc_inverse_gsl (&x_ji.vector, &Jminus.matrix, &x_ij.vector);
    gslu_vector_printfc (&x_ji.vector, "x_ji_gsl", NULL, CblasTrans);
    gslu_matrix_printf (&Jminus.matrix, "Jminus_gsl");

    ssc_inverse (x_ji.data, Jminus.data, x_ij.data);
    gslu_vector_printfc (&x_ji.vector, "x_ji", NULL, CblasTrans);
    gslu_matrix_printf (&Jminus.matrix, "Jminus");

    // ssc_head2tail
    printf ("\nssc_head2tail: x_ik = x_ij (+) x_jk\n");
    GSLU_VECTOR_VIEW (x_ik, 6);
    GSLU_MATRIX_VIEW (Jplus, 6, 12);

    ssc_head2tail_gsl (&x_ik.vector, &Jplus.matrix, &x_ij.vector, &x_jk.vector);
    gslu_vector_printfc (&x_ik.vector, "x_ik_gsl", NULL, CblasTrans);
    gslu_matrix_printf (&Jplus.matrix, "Jplus_gsl");

    ssc_head2tail (x_ik.data, Jplus.data, x_ij.data, x_jk.data);
    gslu_vector_printfc (&x_ik.vector, "x_ik", NULL, CblasTrans);
    gslu_matrix_printf (&Jplus.matrix, "Jplus");

    // ssc_tail2tail
    printf ("\nssc_tail2tail: x_ik = (-)x_ji (+) x_jk\n");
    GSLU_VECTOR_VIEW (x_ik2, 6);
    GSLU_MATRIX_VIEW (Jtail, 6, 12);

    ssc_tail2tail_gsl (&x_ik2.vector, &Jtail.matrix, &x_ji.vector, &x_jk.vector);
    gslu_vector_printfc (&x_ik2.vector, "x_ik2_gsl", NULL, CblasTrans);
    gslu_matrix_printf (&Jtail.matrix, "Jtail_gsl");

    ssc_tail2tail (x_ik2.data, Jtail.data, x_ji.data, x_jk.data);
    gslu_vector_printfc (&x_ik2.vector, "x_ik2", NULL, CblasTrans);
    gslu_matrix_printf (&Jtail.matrix, "Jtail");

    // ssc_homo4x4
    printf ("\nssc_homo4x4: x_ik\n");
    GSLU_MATRIX_VIEW (H_ik, 4, 4);

    ssc_homo4x4_gsl (&H_ik.matrix, &x_ik.vector);
    gslu_matrix_printf (&H_ik.matrix, "H_ik_gsl");

    ssc_homo4x4 (H_ik.data, x_ik.data);
    gslu_matrix_printf (&H_ik.matrix, "H_ik_gsl");
}
