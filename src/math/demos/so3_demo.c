#include <stdio.h>
#include <stdlib.h>

#include "math/fasttrig.h"
#include "math/math_util.h"
#include "math/gsl_util_array.h"
#include "math/gsl_util_vector.h"
#include "math/gsl_util_matrix.h"
#include "math/so3.h"

int
main (int argc, char *argv[])
{
    fasttrig_init ();

    // Rx using gsl_util matrix representation
    GSLU_MATRIX_VIEW (Rx, 3, 3);
    so3_rotx (Rx.data, 30*DTOR);
    gslu_matrix_printf (&Rx.matrix, "Rx");

    // Ry using double array representation
    double Ry[9];
    so3_roty (Ry, 30*DTOR);
    gslu_array_printf (Ry, 3, 3, "Ry");

    // Rz using gsl_matrix representation
    gsl_matrix *Rz = gsl_matrix_alloc (3, 3);
    so3_rotz (Rz->data, 30*DTOR);
    gslu_matrix_printf (Rz, "Rz");

    // convert Euler rph to SO3 rotation matrix R
    double R[3*3];
    double rph[] = {30*DTOR, 50*DTOR, 27*DTOR};
    so3_rotxyz (R, rph);
    gslu_array_printf (R, 3, 3, "R");

    // convert SO3 rotation matrix back to Euler rph
    so3_rot2rph (R, rph);
    gsl_vector_view rph_view = gsl_vector_view_array (rph, 3);
    gsl_vector_scale (&rph_view.vector, RTOD);
    gslu_vector_printf (&rph_view.vector, "rph");
}
