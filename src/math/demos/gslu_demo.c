#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>
#include <math.h>
#include <gsl/gsl_linalg.h>

#include "math/so3.h"
#include "math/gsl_util.h"

int
main (int argc, char *argv[])
{
    printf ("===================================================================\n");
    printf ("gsl_util_vector\n");
    printf ("===================================================================\n");

    // example of how to use gslu macros to create vector elements
    // on the stack with vector views.  x, and b each have subfields
    // .data and .vector.  The advantage is that elements created on the stack
    // do not have to be manually freed by the programmer, they automatically
    // have limited scope
    GSLU_VECTOR_VIEW (x, 4);
    x.data[0] = 0;  gsl_vector_set (&x.vector, 0, 0);
    x.data[1] = 0;  gsl_vector_set (&x.vector, 1, 0);
    x.data[2] = 1;  gsl_vector_set (&x.vector, 2, 1);
    x.data[3] = 0;  gsl_vector_set (&x.vector, 3, 0);
    gslu_vector_printf (&x.vector, "x");
    printf ("\n");

    // and here's another syntax that allows you to create and
    // simultaneously initialize, which is equivalent to the above
    GSLU_VECTOR_VIEW (xx, 4, {0, 0, 1, 0});
    gslu_vector_printf (&xx.vector, "xx");
    printf ("\n");

    // clone a vector
    gsl_vector *y = gslu_vector_clone (&x.vector);
    gslu_vector_printf (y, "y");
    printf ("\n");

    // select a subset of a vector
    GSLU_INDEX_VIEW (isel, 2, {2, 3});
    gsl_vector *x_sub = gslu_vector_sel_alloc (&x.vector, &isel.vector);
    gslu_vector_printf (x_sub, "x_sub");
    printf ("\n");

    // set a subvector
    gslu_vector_set_subvector (y, 0, x_sub);
    gslu_vector_printf (y, "y");
    printf ("\n");

    // vector sum
    double sum = gslu_vector_sum (y);
    printf ("sum_ =\n     %g\n\n", sum);

    // dist
    double dist = gslu_vector_dist (&x.vector, y);
    printf ("dist =\n     %g\n\n", dist);

    // cross
    {
        GSLU_VECTOR_VIEW (a, 3, {1, 2, 3});
        GSLU_VECTOR_VIEW (b, 3, {6, 5, 4});

        gsl_vector *c = gslu_vector_cross_alloc (&a.vector, &b.vector);
        gslu_vector_printf (c, "c");
        printf ("\n");

        // scalar triple prod
        double stp = gslu_vector_scalar_triple_prod (&a.vector, &b.vector, c);
        printf ("stp =\n  %g\n\n", stp);

        // vector triple prod
        gsl_vector *vtp = gslu_vector_triple_prod_alloc (&a.vector, &b.vector, c);
        gslu_vector_printf (vtp, "vtp");
        printf ("\n");

        gsl_vector_free (c);
        gsl_vector_free (vtp);
    }


    printf ("===================================================================\n");
    printf ("gsl_util_matrix\n");
    printf ("===================================================================\n");

    // static allocation
    GSLU_MATRIX_VIEW (A_static, 4, 4,
                      { 4, 10, 7, 4,
                        0, 1,  7, 6,
                        7, 5,  7, 1,
                        8, 6,  7, 1 });
    gsl_matrix *A = &A_static.matrix;
    gslu_matrix_printf (A, "A");
    printf ("\n");
    gslu_matrix_printfc (A, "A_transpose", NULL, CblasTrans);
    printf ("\n");

    GSLU_MATRIX_VIEW (B_static, 4, 4,
                      {  9, 1, 8, 2,
                        10, 3, 2, 3,
                         5, 8, 9, 6,
                         1, 3, 3, 5 });
    gsl_matrix *B = &B_static.matrix;
    gslu_matrix_printf (B, "B");
    printf ("\n");


    GSLU_MATRIX_VIEW (C_static, 4, 4,
                      {0,   1,  2,  3,
                       4,   5,  6,  7,
                       8,   9, 10, 11,
                       12, 13, 14, 15}
        );
    gsl_matrix *C = &C_static.matrix;
    gslu_matrix_printf (C, "C");
    printf ("\n");


    // selrow
    gsl_matrix *D = gslu_matrix_selrow_alloc (C, &isel.vector);
    gslu_matrix_printf (D, "D");
    printf ("\n");


    // selcol
    gsl_matrix *E = gslu_matrix_selcol_alloc (C, &isel.vector);
    gslu_matrix_printf (E, "E");
    printf ("\n");

    // equal
    printf ("Aequal =\n     %d\n\n", gslu_matrix_is_equal (A, A, 1e-14));

    // determinant
    double Adet = gslu_matrix_det (A);
    printf ("Adet =\n   %g\n\n", Adet);

    // trace
    double Atrace = gslu_matrix_trace (A);
    printf ("Atrace =\n   %g\n\n", Atrace);

    // inverse
    gsl_matrix *Ainv = gsl_matrix_alloc (A->size1, A->size2);
    gslu_matrix_inv (Ainv, A);
    gslu_matrix_printf (Ainv, "Ainv");
    printf ("\n");

    // submatrix
    gslu_matrix_set_submatrix (C, 0, 2, E);
    gslu_matrix_set_submatrix (C, 2, 0, D);
    gslu_matrix_printf (C, "C");
    printf ("\n");

    // reshaping a matrix into a new size
    GSLU_MATRIX_VIEW (C_2x8, 2, 8);
    gslu_matrix_reshape (&C_2x8.matrix, C, CblasNoTrans);
    gslu_matrix_printf (&C_2x8.matrix, "C_2x8");
    printf ("\n");

    // reshaping the transpose of matrix
    GSLU_MATRIX_VIEW (Ctrans_2x8, 2, 8);
    gslu_matrix_reshape (&Ctrans_2x8.matrix, C, CblasTrans);
    gslu_matrix_printf (&Ctrans_2x8.matrix, "Ctrans_2x8");
    printf ("\n");

    // stacking a matrix into vector form
    GSLU_VECTOR_VIEW (c, 16);
    gslu_matrix_stack (&c.vector, C, CblasNoTrans);
    gslu_vector_printf (&c.vector, "c");
    printf ("\n");

    // reshaping a vector back into a matrix
    GSLU_MATRIX_VIEW (Cprime, 4, 4);
    gslu_vector_reshape (&Cprime.matrix, &c.vector, CblasNoTrans);
    gslu_matrix_printf (&Cprime.matrix, "Cprime");
    printf ("\n");

    // skew symmetric matrix
    GSLU_VECTOR_VIEW (t, 3, {1, 2, 3});
    gsl_matrix *skew = gslu_matrix_skewsym_alloc (&t.vector);
    gslu_matrix_printf (skew, "skew");
    printf ("\n");



    printf ("===================================================================\n");
    printf ("gsl_util_blas\n");
    printf ("===================================================================\n");

    // matrix * vector
    GSLU_VECTOR_VIEW (b, 4);
    gslu_blas_mv (&b.vector, A, &x.vector); // A*x = b
    gslu_vector_printf (&b.vector, "b");
    printf ("\n");

    // vector^T * matrix
    gsl_vector *btrans = gslu_blas_vTm_alloc (&x.vector, A); // b' = x'*A
    gslu_vector_printfc (btrans, "btrans", NULL, CblasTrans);
    printf ("\n");

    // vectorT * matrix * vector
    double scalar = gslu_blas_vTmv (&x.vector, A, y);
    printf ("scalar =\n    %g\n", scalar);
    printf ("\n");

    // computes C1 = A*B
    gsl_matrix *C1 = gslu_blas_mm_alloc (A, B);
    gslu_matrix_printf (C1, "C1");
    printf ("\n");

    // computes C2 = A*B'
    gsl_matrix *C2 = gslu_blas_mmT_alloc (A, B);
    gslu_matrix_printf (C2, "C2");
    printf ("\n");

    // computes C3 = A'*B
    gsl_matrix *C3 = gslu_blas_mTm_alloc (A, B);
    gslu_matrix_printf (C3, "C3");
    printf ("\n");

    // computes C4 = A'*B'
    gsl_matrix *C4 = gslu_blas_mTmT_alloc (A, B);
    gslu_matrix_printf (C4, "C4");


    // computes D1 = A*B*C
    gsl_matrix *D1 = gslu_blas_mmm_alloc (A, B, C, NULL);
    gslu_matrix_printf (D1, "D1");
    printf ("\n");

    // computes D2 = A*B*C'
    gsl_matrix *D2 = gslu_blas_mmmT_alloc (A, B, C, NULL);
    gslu_matrix_printf (D2, "D2");
    printf ("\n");

    // computes D3 = A*B'*C
    gsl_matrix *D3 = gslu_blas_mmTm_alloc (A, B, C, NULL);
    gslu_matrix_printf (D3, "D3");
    printf ("\n");

    // computes D4 = A'*B*C
    gsl_matrix *D4 = gslu_blas_mTmm_alloc (A, B, C, NULL);
    gslu_matrix_printf (D4, "D4");
    printf ("\n");

    // computes D5 = A*B'*C'
    gsl_matrix *D5 = gslu_blas_mmTmT_alloc (A, B, C, NULL);
    gslu_matrix_printf (D5, "D5");
    printf ("\n");

    // computes D6 = A'*B'*C
    gsl_matrix *D6 = gslu_blas_mTmTm_alloc (A, B, C, NULL);
    gslu_matrix_printf (D6, "D6");
    printf ("\n");

    // computes D7 = A'*B*C'
    gsl_matrix *D7 = gslu_blas_mTmmT_alloc (A, B, C, NULL);
    gslu_matrix_printf (D7, "D7");
    printf ("\n");

    // computes D8 = A'*B'*C'
    gsl_matrix *D8 = gslu_blas_mTmTmT_alloc (A, B, C, NULL);
    gslu_matrix_printf (D8, "D8");
    printf ("\n");


    printf ("===================================================================\n");
    printf ("gsl_util_linalg\n");
    printf ("===================================================================\n");

    printf ("\nQR factorization of A\n");
    gslu_linalg_QR *qr = gslu_linalg_QR_decomp_alloc (A);
    gsl_matrix *Q = gsl_matrix_alloc (A->size1, A->size1);
    gsl_matrix *R = gsl_matrix_alloc (A->size1, A->size2);
    gsl_linalg_QR_unpack (qr->QR, qr->tau, Q, R);
    gslu_matrix_printf (Q, "Q");
    printf ("\n");
    gslu_matrix_printf (R, "R");
    printf ("\n");
    gslu_linalg_QR_free (qr);
    gsl_matrix_free (Q);
    gsl_matrix_free (R);


    printf ("SVD decomposition of A (econ) for thin or square matrices\n");
    gslu_linalg_SV *sv = gslu_linalg_SV_decomp_econ_alloc (A);
    gslu_matrix_printf (sv->U, "U");
    printf ("\n");
    gslu_vector_printf (sv->S, "S");
    printf ("\n");
    gslu_matrix_printf (sv->V, "V");
    printf ("\n");
    gslu_linalg_SV_free (sv);


    printf ("SVD decomposition of A_fat (full)\n");
    gsl_matrix_view A_fat = gsl_matrix_submatrix (&A_static.matrix, 0, 0, 3, 4);
    gslu_matrix_printf (&A_fat.matrix, "A fat");
    printf ("\n");
    gslu_linalg_SV *sv2 = gslu_linalg_SV_decomp_full_alloc (&A_fat.matrix);
    gslu_matrix_printf (sv2->U, "U2");
    printf ("\n");
    gslu_vector_printf (sv2->S, "S2");
    printf ("\n");
    gslu_matrix_printf (sv2->V, "V2");
    printf ("\n");
    gslu_linalg_SV_free (sv2);

    printf ("Reduced row echelon form of A\n");
    gsl_matrix *A_rref = gslu_linalg_rref_alloc (A, 0);
    gslu_matrix_printf (A_rref, "A_rref");
    printf ("\n");
    gslu_matrix_free (A_rref);


#if 0
    printf ("===================================================================\n");
    printf ("gsl_util_rand\n");
    printf ("===================================================================\n");

    gsl_rng *rng = gslu_rand_rng_alloc ();

    // w = zeros(5,1) and W = zeros(5,5)
    double s = 0;
    GSLU_VECTOR_VIEW (w_static, 5, {0.0});
    GSLU_MATRIX_VIEW (W_static, 5, 5, {0.0});
    gsl_vector *w = &w_static.vector;
    gsl_matrix *W = &W_static.matrix;
    gslu_vector_printf (w, "w");
    printf ("\n");
    gslu_matrix_printf (W, "W");
    printf ("\n");

    // uniform
    printf ("uniform\n");
    s = gslu_rand_uniform (rng);
    gslu_rand_uniform_vector (rng, w);
    gslu_rand_uniform_matrix (rng, W);
    printf ("s =\n   %g\n", s);
    gslu_vector_printf (w, "w");
    gslu_matrix_printf (W, "W");
    printf ("\n");

    // uniform-pos
    printf ("uniform-pos\n");
    s = gslu_rand_uniform_pos (rng);
    gslu_rand_uniform_pos_vector (rng, w);
    gslu_rand_uniform_pos_matrix (rng, W);
    printf ("s =\n    %g\n", s);
    gslu_vector_printf (w, "w");
    gslu_matrix_printf (W, "W");
    printf ("\n");

    // normal
    printf ("normal\n");
    s = gslu_rand_normal (rng);
    gslu_rand_normal_vector (rng, w);
    gslu_rand_normal_matrix (rng, W);
    printf ("s =\n    %g\n", s);
    gslu_vector_printf (w, "w");
    gslu_matrix_printf (W, "W");
    printf ("\n");

    // Gaussian
    printf ("gaussian\n");
    const double rho = 0.75;
    GSLU_VECTOR_VIEW (mu, 5, {0, 0, 200, 300, 400});
    GSLU_MATRIX_VIEW (Sigma, 5, 5,
                      {   1, -rho,  0,  0,    0,
                       -rho,    1,  0,  0,    0,
                          0,    0, .1,  0,    0,
                          0,    0,  0,  1,    0,
                          0,    0,  0,  0, .001 });
    s = gslu_rand_gaussian (rng, 1.0, 3.0);
    gslu_rand_gaussian_vector (rng, w, &mu.vector, &Sigma.matrix, NULL);
    gslu_rand_gaussian_matrix (rng, W, &mu.vector, &Sigma.matrix, NULL);
    printf ("s =\n    %g\n", s);
    gslu_vector_printf (w, "w");
    gslu_matrix_printf (W, "W");
    printf ("\n");

    gsl_rng_free (rng);

#endif

    // clean up
    gsl_vector_free (btrans);
    gsl_vector_free (y);
    gsl_vector_free (x_sub);
    gsl_matrix_free (Ainv);
    gsl_matrix_free (C1);
    gsl_matrix_free (C2);
    gsl_matrix_free (C3);
    gsl_matrix_free (C4);
    gsl_matrix_free (D);
    gsl_matrix_free (D1);
    gsl_matrix_free (D2);
    gsl_matrix_free (D3);
    gsl_matrix_free (D4);
    gsl_matrix_free (D5);
    gsl_matrix_free (D6);
    gsl_matrix_free (D7);
    gsl_matrix_free (D8);
    gsl_matrix_free (E);

    printf ("===================================================================\n");
    printf ("compare gslu_matrix_inv() to Matlab 5x5\n");
    printf ("===================================================================\n");


    GSLU_MATRIX_VIEW (M_static, 5, 5,
                      { 0.8147,  0.0975,  0.1576,  0.1419,  0.6557,
                        0.9058,  0.2785,  0.9706,  0.4218,  0.0357,
                        0.1270,  0.5469,  0.9572,  0.9157,  0.8491,
                        0.9134,  0.9575,  0.4854,  0.7922,  0.9340,
                        0.6324,  0.9649,  0.8003,  0.9595,  0.6787 }
        );
    gsl_matrix *M = &M_static.matrix;
    gslu_matrix_printf (M, "M");
    printf ("\n");

    gsl_matrix *M_inv = gslu_matrix_inv_alloc (M);
    gslu_matrix_printf (M_inv, "M_inv");
    printf ("\n");

    GSLU_MATRIX_VIEW (M_inv_matlab_static, 5, 5,
                      {  3.13342290447731,  -0.805816035240376,  -1.87652301269401,  -4.21217231519133,   5.15943254230071,
                        -8.59849017455945,   3.526888281072080,   2.88540052118297,  13.70380515107170, -14.34688006160760,
                        -6.27503964814357,   3.718425084991690,   3.60897475879111,   9.99519721809115, -12.40326940862400,
                        13.60150840629180,  -6.874489097114450,  -6.38449099370088, -23.50011412870400,  27.54838381865690,
                        -2.52486600970489,   1.070753280450900,   2.41674789941611,   5.87915036792562,  -7.25778449180101 }
        );
    gsl_matrix *M_inv_matlab = &M_inv_matlab_static.matrix;
    gslu_matrix_printf (M_inv_matlab, "M_inv_matlab");
    printf ("\n");

    if (gslu_matrix_is_equal (M_inv, M_inv_matlab, 1e-13))
        printf ("M_inv and M_inv_matlab are the same\n");
    else
        printf ("ERROR: M_inv and M_inv_matlab are not the same!\n");

    gsl_matrix_free (M_inv);
}
