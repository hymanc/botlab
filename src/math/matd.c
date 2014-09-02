#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <assert.h>
#include <math.h>

#include "matd.h"

// a matd_t with rows=0 cols=0 is a SCALAR.

// to ease creating mati, matf, etc. in the future.
#define TYPE double

matd_t *matd_create(int rows, int cols)
{
    assert(rows >= 0);
    assert(cols >= 0);

    if (rows == 0 || cols == 0)
        return matd_create_scalar(0);

    matd_t *m = calloc(1, sizeof(matd_t));
    m->nrows = rows;
    m->ncols = cols;
    m->data = calloc(m->nrows * m->ncols, sizeof(TYPE));

    return m;
}

matd_t *matd_create_scalar(TYPE v)
{
    matd_t *m = calloc(1, sizeof(matd_t));
    m->nrows = 0;
    m->ncols = 0;
    m->data = calloc(1, sizeof(TYPE));
    m->data[0] = v;

    return m;
}

matd_t *matd_create_data(int rows, int cols, const TYPE *data)
{
    if (rows == 0 || cols == 0)
        return matd_create_scalar(data[0]);

    matd_t *m = matd_create(rows, cols);
    for (int i = 0; i < rows * cols; i++)
        m->data[i] = data[i];

    return m;
}

matd_t *matd_identity(int dim)
{
    if (dim == 0)
        return matd_create_scalar(1);

    matd_t *m = matd_create(dim, dim);
    for (int i = 0; i < dim; i++)
        MATD_EL(m, i, i) = 1;

    return m;
}

// row and col are zero-based
TYPE matd_get(const matd_t *m, int row, int col)
{
    assert(m != NULL);
    assert(!matd_is_scalar(m));
    assert(row >= 0);
    assert(row < m->nrows);
    assert(col >= 0);
    assert(col < m->ncols);

    return MATD_EL(m, row, col);
}

// row and col are zero-based
void matd_set(matd_t *m, int row, int col, TYPE value)
{
    assert(m != NULL);

    if (matd_is_scalar(m)) {
        matd_set_scalar(m, value);
        return;
    }

    assert(row >= 0);
    assert(row < m->nrows);
    assert(col >= 0);
    assert(col < m->ncols);

    MATD_EL(m, row, col) = value;
}

TYPE matd_get_scalar(const matd_t *m)
{
    assert(m != NULL);
    assert(matd_is_scalar(m));

    return (m->data[0]);
}

void matd_set_scalar(matd_t *m, TYPE value)
{
    assert(m != NULL);
    assert(matd_is_scalar(m));

    m->data[0] = value;
}

matd_t *matd_copy(const matd_t *m)
{
    assert(m != NULL);

    matd_t *x = matd_create(m->nrows, m->ncols);
    if (matd_is_scalar(m))
        x->data[0] = m->data[0];
    else
        memcpy(x->data, m->data, sizeof(TYPE)*m->ncols*m->nrows);

    return x;
}

matd_t *matd_select(const matd_t * a, int r0, int r1, int c0, int c1)
{
    assert(a != NULL);

    assert(r0 >= 0 && r0 < a->nrows);
    assert(c0 >= 0 && c0 < a->ncols);

    int nrows = r1 - r0 + 1;
    int ncols = c1 - c0 + 1;

    matd_t * r = matd_create(nrows, ncols);

    for (int row = r0; row <= r1; row++)
        for (int col = c0; col <= c1; col++)
            MATD_EL(r,row-r0,col-c0) = MATD_EL(a,row,col);

    return r;
}

void matd_print(const matd_t *m, const char *fmt)
{
    assert(m != NULL);
    assert(fmt != NULL);

    if (matd_is_scalar(m)) {
        printf(fmt, MATD_EL(m, 0, 0));
        printf("\n");
    } else {
        for (int i = 0; i < m->nrows; i++) {
            for (int j = 0; j < m->ncols; j++) {
                printf(fmt, MATD_EL(m, i, j));
            }
            printf("\n");
        }
    }
}

void matd_print_transpose(const matd_t *m, const char *fmt)
{
    assert(m != NULL);
    assert(fmt != NULL);

    if (matd_is_scalar(m)) {
        printf(fmt, MATD_EL(m, 0, 0));
        printf("\n");
    } else {
        for (int j = 0; j < m->ncols; j++) {
            for (int i = 0; i < m->nrows; i++) {
                printf(fmt, MATD_EL(m, i, j));
            }
            printf("\n");
        }
    }
}

void matd_destroy(matd_t *m)
{
    assert(m != NULL);

    free(m->data);

    // set data pointer to NULL to cause segfault if used
    // after the destroy call (hard to catch failure mode)
    m->data = NULL;

    free(m);
}

matd_t *matd_multiply(const matd_t *a, const matd_t *b)
{
    assert(a != NULL);
    assert(b != NULL);

    if (matd_is_scalar(a))
        return matd_scale(b, a->data[0]);
    if (matd_is_scalar(b))
        return matd_scale(a, b->data[0]);

    assert(a->ncols == b->nrows);
    matd_t *m = matd_create(a->nrows, b->ncols);

    for (int i = 0; i < m->nrows; i++) {
        for (int j = 0; j < m->ncols; j++) {
            TYPE acc = 0;
            for (int k = 0; k < a->ncols; k++) {
                acc += MATD_EL(a, i, k) * MATD_EL(b, k, j);
            }
            MATD_EL(m, i, j) = acc;
        }
    }

    return m;
}

matd_t *matd_scale(const matd_t *a, double s)
{
    assert(a != NULL);

    if (matd_is_scalar(a))
        return matd_create_scalar(a->data[0] * s);

    matd_t *m = matd_create(a->nrows, a->ncols);

    for (int i = 0; i < m->nrows; i++) {
        for (int j = 0; j < m->ncols; j++) {
            MATD_EL(m, i, j) = s * MATD_EL(a, i, j);
        }
    }

    return m;
}

void matd_scale_inplace(matd_t *a, double s)
{
    assert(a != NULL);

    if (matd_is_scalar(a)) {
        a->data[0] *= s;
        return;
    }

    for (int i = 0; i < a->nrows; i++) {
        for (int j = 0; j < a->ncols; j++) {
            MATD_EL(a, i, j) *= s;
        }
    }
}

matd_t *matd_add(const matd_t *a, const matd_t *b)
{
    assert(a != NULL);
    assert(b != NULL);
    assert(a->nrows == b->nrows);
    assert(a->ncols == b->ncols);

    if (matd_is_scalar(a))
        return matd_create_scalar(a->data[0] + b->data[0]);

    matd_t *m = matd_create(a->nrows, a->ncols);

    for (int i = 0; i < m->nrows; i++) {
        for (int j = 0; j < m->ncols; j++) {
            MATD_EL(m, i, j) = MATD_EL(a, i, j) + MATD_EL(b, i, j);
        }
    }

    return m;
}

void matd_add_inplace(matd_t *a, const matd_t *b)
{
    assert(a != NULL);
    assert(b != NULL);
    assert(a->nrows == b->nrows);
    assert(a->ncols == b->ncols);

    if (matd_is_scalar(a)) {
        a->data[0] += b->data[0];
        return;
    }

    for (int i = 0; i < a->nrows; i++) {
        for (int j = 0; j < a->ncols; j++) {
            MATD_EL(a, i, j) += MATD_EL(b, i, j);
        }
    }
}


matd_t *matd_subtract(const matd_t *a, const matd_t *b)
{
    assert(a != NULL);
    assert(b != NULL);
    assert(a->nrows == b->nrows);
    assert(a->ncols == b->ncols);

    if (matd_is_scalar(a))
        return matd_create_scalar(a->data[0] - b->data[0]);

    matd_t *m = matd_create(a->nrows, a->ncols);

    for (int i = 0; i < m->nrows; i++) {
        for (int j = 0; j < m->ncols; j++) {
            MATD_EL(m, i, j) = MATD_EL(a, i, j) - MATD_EL(b, i, j);
        }
    }

    return m;
}

void matd_subtract_inplace(matd_t *a, const matd_t *b)
{
    assert(a != NULL);
    assert(b != NULL);
    assert(a->nrows == b->nrows);
    assert(a->ncols == b->ncols);

    if (matd_is_scalar(a)) {
        a->data[0] -= b->data[0];
        return;
    }

    for (int i = 0; i < a->nrows; i++) {
        for (int j = 0; j < a->ncols; j++) {
            MATD_EL(a, i, j) -= MATD_EL(b, i, j);
        }
    }
}


matd_t *matd_transpose(const matd_t *a)
{
    assert(a != NULL);

    if (matd_is_scalar(a))
        return matd_create_scalar(a->data[0]);

    matd_t *m = matd_create(a->ncols, a->nrows);

    for (int i = 0; i < a->nrows; i++) {
        for (int j = 0; j < a->ncols; j++) {
            MATD_EL(m, j, i) = MATD_EL(a, i, j);
        }
    }
    return m;
}

double matd_det(const matd_t *a)
{
    assert(a != NULL);
    assert(a->nrows == a->ncols);

    switch(a->nrows) {
        case 0:
            // scalar: invalid
            assert(a->nrows > 0);
            break;

        case 1:
            // 1x1 matrix
            return a->data[0];
            break;

        case 2:
            // 2x2 matrix
            return a->data[0] * a->data[3] - a->data[1] * a->data[2];
            break;

        case 3:
            // 3x3 matrix
            return a->data[0]*a->data[4]*a->data[8]
                - a->data[0]*a->data[5]*a->data[7]
                + a->data[1]*a->data[5]*a->data[6]
                - a->data[1]*a->data[3]*a->data[8]
                + a->data[2]*a->data[3]*a->data[7]
                - a->data[2]*a->data[4]*a->data[6];
            break;

        case 4: {
            // 4x4 matrix
            double m00 = MATD_EL(a,0,0), m01 = MATD_EL(a,0,1), m02 = MATD_EL(a,0,2), m03 = MATD_EL(a,0,3);
            double m10 = MATD_EL(a,1,0), m11 = MATD_EL(a,1,1), m12 = MATD_EL(a,1,2), m13 = MATD_EL(a,1,3);
            double m20 = MATD_EL(a,2,0), m21 = MATD_EL(a,2,1), m22 = MATD_EL(a,2,2), m23 = MATD_EL(a,2,3);
            double m30 = MATD_EL(a,3,0), m31 = MATD_EL(a,3,1), m32 = MATD_EL(a,3,2), m33 = MATD_EL(a,3,3);

            return m00 * m11 * m22 * m33 - m00 * m11 * m23 * m32 -
                m00 * m21 * m12 * m33 + m00 * m21 * m13 * m32 + m00 * m31 * m12 * m23 -
                m00 * m31 * m13 * m22 - m10 * m01 * m22 * m33 +
                m10 * m01 * m23 * m32 + m10 * m21 * m02 * m33 -
                m10 * m21 * m03 * m32 - m10 * m31 * m02 * m23 +
                m10 * m31 * m03 * m22 + m20 * m01 * m12 * m33 -
                m20 * m01 * m13 * m32 - m20 * m11 * m02 * m33 +
                m20 * m11 * m03 * m32 + m20 * m31 * m02 * m13 -
                m20 * m31 * m03 * m12 - m30 * m01 * m12 * m23 +
                m30 * m01 * m13 * m22 + m30 * m11 * m02 * m23 -
                m30 * m11 * m03 * m22 - m30 * m21 * m02 * m13 +
                m30 * m21 * m03 * m12;
            }
            break;

        default: {
            matd_lu_t *lu = matd_lu(a);
            double det = matd_lu_det(lu);
            matd_lu_destroy(lu);
            return det;
        }
    }

    return 0;
}

static double make_non_zero(double v)
{
    if (fabs(v) < MATD_EPS) {
        if (v < 0)
            return -MATD_EPS;
        return MATD_EPS;
    }

    return v;
}

matd_t *matd_inverse(const matd_t *x)
{
    assert(x != NULL);
    assert(x->nrows == x->ncols);

    if (matd_is_scalar(x))
        return matd_create_scalar(1.0 / x->data[0]);

    if (x->nrows == 1) {
        // a 1x1 matrix
        matd_t *m = matd_create(x->nrows, x->nrows);
        MATD_EL(m, 0, 0) = 1.0 / x->data[0];
        return m;
    }

    if (x->nrows > 2) {
        matd_lu_t *lu = matd_lu(x);
        matd_t *eye = matd_identity(x->nrows);
        matd_t *inv = matd_lu_solve(lu, eye);

        matd_destroy(eye);
        matd_lu_destroy(lu);
        return inv;
    }

    double invdet = 1.0 / make_non_zero(matd_det(x));
    matd_t *m = NULL;

    switch(x->nrows) {

        case 2:
            m = matd_create(x->nrows, x->nrows);
            MATD_EL(m, 0, 0) = MATD_EL(x, 1, 1) * invdet;
            MATD_EL(m, 0, 1) = - MATD_EL(x, 0, 1) * invdet;
            MATD_EL(m, 1, 0) = - MATD_EL(x, 1, 0) * invdet;
            MATD_EL(m, 1, 1) = MATD_EL(x, 0, 0) * invdet;
            return m;

        case 3:
            // dead code due to poor numerical stability
            m = matd_create(x->nrows, x->nrows);

            double a = MATD_EL(x, 0, 0), b = MATD_EL(x, 0, 1), c = MATD_EL(x, 0, 2);
            double d = MATD_EL(x, 1, 0), e = MATD_EL(x, 1, 1), f = MATD_EL(x, 1, 2);
            double g = MATD_EL(x, 2, 0), h = MATD_EL(x, 2, 1), i = MATD_EL(x, 2, 2);

            MATD_EL(m,0,0) = invdet*(e*i-f*h);
            MATD_EL(m,0,1) = invdet*(-b*i+c*h);
            MATD_EL(m,0,2) = invdet*(b*f-c*e);
            MATD_EL(m,1,0) = invdet*(-d*i+f*g);
            MATD_EL(m,1,1) = invdet*(a*i-c*g);
            MATD_EL(m,1,2) = invdet*(-a*f+c*d);
            MATD_EL(m,2,0) = invdet*(d*h-e*g);
            MATD_EL(m,2,1) = invdet*(-a*h+b*g);
            MATD_EL(m,2,2) = invdet*(a*e-b*d);
            return m;

        case 4: {
            // dead code due to poor numerical stability
            double m00 = MATD_EL(x,0,0), m01 = MATD_EL(x,0,1), m02 = MATD_EL(x,0,2), m03 = MATD_EL(x,0,3);
            double m10 = MATD_EL(x,1,0), m11 = MATD_EL(x,1,1), m12 = MATD_EL(x,1,2), m13 = MATD_EL(x,1,3);
            double m20 = MATD_EL(x,2,0), m21 = MATD_EL(x,2,1), m22 = MATD_EL(x,2,2), m23 = MATD_EL(x,2,3);
            double m30 = MATD_EL(x,3,0), m31 = MATD_EL(x,3,1), m32 = MATD_EL(x,3,2), m33 = MATD_EL(x,3,3);

            m = matd_create(x->nrows, x->nrows);
            MATD_EL(m,0,0) =   m11 * m22 * m33 - m11 * m23 * m32 - m21 * m12 * m33 + m21 * m13 * m32 + m31 * m12 * m23 - m31 * m13 * m22;
            MATD_EL(m,1,0) = - m10 * m22 * m33 + m10 * m23 * m32 + m20 * m12 * m33 - m20 * m13 * m32 - m30 * m12 * m23 + m30 * m13 * m22;
            MATD_EL(m,2,0) =   m10 * m21 * m33 - m10 * m23 * m31 - m20 * m11 * m33 + m20 * m13 * m31 + m30 * m11 * m23 - m30 * m13 * m21;
            MATD_EL(m,3,0) = - m10 * m21 * m32 + m10 * m22 * m31 + m20 * m11 * m32 - m20 * m12 * m31 - m30 * m11 * m22 + m30 * m12 * m21;
            MATD_EL(m,0,1) = - m01 * m22 * m33 + m01 * m23 * m32 + m21 * m02 * m33 - m21 * m03 * m32 - m31 * m02 * m23 + m31 * m03 * m22;
            MATD_EL(m,1,1) =   m00 * m22 * m33 - m00 * m23 * m32 - m20 * m02 * m33 + m20 * m03 * m32 + m30 * m02 * m23 - m30 * m03 * m22;
            MATD_EL(m,2,1) = - m00 * m21 * m33 + m00 * m23 * m31 + m20 * m01 * m33 - m20 * m03 * m31 - m30 * m01 * m23 + m30 * m03 * m21;
            MATD_EL(m,3,1) =   m00 * m21 * m32 - m00 * m22 * m31 - m20 * m01 * m32 + m20 * m02 * m31 + m30 * m01 * m22 - m30 * m02 * m21;
            MATD_EL(m,0,2) =   m01 * m12 * m33 - m01 * m13 * m32 - m11 * m02 * m33 + m11 * m03 * m32 + m31 * m02 * m13 - m31 * m03 * m12;
            MATD_EL(m,1,2) = - m00 * m12 * m33 + m00 * m13 * m32 + m10 * m02 * m33 - m10 * m03 * m32 - m30 * m02 * m13 + m30 * m03 * m12;
            MATD_EL(m,2,2) =   m00 * m11 * m33 - m00 * m13 * m31 - m10 * m01 * m33 + m10 * m03 * m31 + m30 * m01 * m13 - m30 * m03 * m11;
            MATD_EL(m,3,2) = - m00 * m11 * m32 + m00 * m12 * m31 + m10 * m01 * m32 - m10 * m02 * m31 - m30 * m01 * m12 + m30 * m02 * m11;
            MATD_EL(m,0,3) = - m01 * m12 * m23 + m01 * m13 * m22 + m11 * m02 * m23 - m11 * m03 * m22 - m21 * m02 * m13 + m21 * m03 * m12;
            MATD_EL(m,1,3) =   m00 * m12 * m23 - m00 * m13 * m22 - m10 * m02 * m23 + m10 * m03 * m22 + m20 * m02 * m13 - m20 * m03 * m12;
            MATD_EL(m,2,3) = - m00 * m11 * m23 + m00 * m13 * m21 + m10 * m01 * m23 - m10 * m03 * m21 - m20 * m01 * m13 + m20 * m03 * m11;
            MATD_EL(m,3,3) =   m00 * m11 * m22 - m00 * m12 * m21 - m10 * m01 * m22 + m10 * m02 * m21 + m20 * m01 * m12 - m20 * m02 * m11;

            for (int i = 0; i < 4; i++)
                for (int j = 0; j < 4; j++)
                    MATD_EL(m,i,j) *= invdet;

            return m;
        }

        default:
            assert(0);
    }

    assert(0);

    return NULL; // unreachable
}

// TODO Optimization: Some operations we could perform in-place,
// saving some memory allocation work. E.g., ADD, SUBTRACT. Just need
// to make sure that we don't do an in-place modification on a matrix
// that was an input argument!

// handle right-associative operators, greedily consuming them. These
// include transpose and inverse. This is called by the main recursion
// method.
static inline matd_t *matd_op_gobble_right(const char *expr, int *pos, matd_t *acc, matd_t **garb, int *garbpos)
{
    while (expr[*pos] != 0) {

        switch (expr[*pos]) {

        case '\'': {
            assert(acc != NULL); // either a syntax error or a math op failed, producing null
            matd_t *res = matd_transpose(acc);
            garb[*garbpos] = res;
            (*garbpos)++;
            acc = res;

            (*pos)++;
            break;
        }

            // handle inverse ^-1. No other exponents are allowed.
        case '^': {
            assert(acc != NULL);
            assert(expr[*pos+1] == '-');
            assert(expr[*pos+2] == '1');

            matd_t *res = matd_inverse(acc);
            garb[*garbpos] = res;
            (*garbpos)++;
            acc = res;

            (*pos)+=3;
            break;
        }

        default:
            return acc;
        }
    }

    return acc;
}

// @garb, garbpos  A list of every matrix allocated during evaluation... used to assist cleanup.
// @oneterm: we should return at the end of this term (i.e., stop at a PLUS, MINUS, LPAREN).
static matd_t *matd_op_recurse(const char *expr, int *pos, matd_t *acc, matd_t **args, int *argpos,
                               matd_t **garb, int *garbpos, int oneterm)
{
    while (expr[*pos] != 0) {

        switch (expr[*pos]) {

        case '(': {
            if (oneterm && acc != NULL)
                return acc;
            (*pos)++;
            matd_t *rhs = matd_op_recurse(expr, pos, NULL, args, argpos, garb, garbpos, 0);
            rhs = matd_op_gobble_right(expr, pos, rhs, garb, garbpos);

            if (acc == NULL) {
                acc = rhs;
            } else {
                matd_t *res = matd_multiply(acc, rhs);
                garb[*garbpos] = res;
                (*garbpos)++;
                acc = res;
            }

            break;
        }

        case ')': {
            if (oneterm)
                return acc;

            (*pos)++;
            return acc;
        }

        case '*': {
            (*pos)++;

            matd_t *rhs = matd_op_recurse(expr, pos, NULL, args, argpos, garb, garbpos, 1);
            rhs = matd_op_gobble_right(expr, pos, rhs, garb, garbpos);

            if (acc == NULL) {
                acc = rhs;
            } else {
                matd_t *res = matd_multiply(acc, rhs);
                garb[*garbpos] = res;
                (*garbpos)++;
                acc = res;
            }

            break;
        }

        case 'F': {
            matd_t *rhs = args[*argpos];
            garb[*garbpos] = rhs;
            (*garbpos)++;

            (*pos)++;
            (*argpos)++;

            rhs = matd_op_gobble_right(expr, pos, rhs, garb, garbpos);

            if (acc == NULL) {
                acc = rhs;
            } else {
                matd_t *res = matd_multiply(acc, rhs);
                garb[*garbpos] = res;
                (*garbpos)++;
                acc = res;
            }

            break;
        }

        case 'M': {
            matd_t *rhs = args[*argpos];

            (*pos)++;
            (*argpos)++;

            rhs = matd_op_gobble_right(expr, pos, rhs, garb, garbpos);

            if (acc == NULL) {
                acc = rhs;
            } else {
                matd_t *res = matd_multiply(acc, rhs);
                garb[*garbpos] = res;
                (*garbpos)++;
                acc = res;
            }

            break;
        }

/*
        case 'D': {
            int rows = expr[*pos+1]-'0';
            int cols = expr[*pos+2]-'0';

            matd_t *rhs = matd_create(rows, cols);

            break;
        }
*/
            // a constant (SCALAR) defined inline. Treat just like M, creating a matd_t on the fly.
        case '0':
        case '1':
        case '2':
        case '3':
        case '4':
        case '5':
        case '6':
        case '7':
        case '8':
        case '9':
        case '.': {
            const char *start = &expr[*pos];
            char *end;
            double s = strtod(start, &end);
            (*pos) += (end - start);
            matd_t *rhs = matd_create_scalar(s);
            garb[*garbpos] = rhs;
            (*garbpos)++;

            rhs = matd_op_gobble_right(expr, pos, rhs, garb, garbpos);

            if (acc == NULL) {
                acc = rhs;
            } else {
                matd_t *res = matd_multiply(acc, rhs);
                garb[*garbpos] = res;
                (*garbpos)++;
                acc = res;
            }

            break;
        }

        case '+': {
            if (oneterm && acc != NULL)
                return acc;

            // don't support unary plus
            assert(acc != NULL);
            (*pos)++;
            matd_t *rhs = matd_op_recurse(expr, pos, NULL, args, argpos, garb, garbpos, 1);
            rhs = matd_op_gobble_right(expr, pos, rhs, garb, garbpos);

            matd_t *res = matd_add(acc, rhs);

            garb[*garbpos] = res;
            (*garbpos)++;
            acc = res;
            break;
        }

        case '-': {
            if (oneterm && acc != NULL)
                return acc;

            if (acc == NULL) {
                // unary minus
                (*pos)++;
                matd_t *rhs = matd_op_recurse(expr, pos, NULL, args, argpos, garb, garbpos, 1);
                rhs = matd_op_gobble_right(expr, pos, rhs, garb, garbpos);

                matd_t *res = matd_scale(rhs, -1);
                garb[*garbpos] = res;
                (*garbpos)++;
                acc = res;
            } else {
                // subtract
                (*pos)++;
                matd_t *rhs = matd_op_recurse(expr, pos, NULL, args, argpos, garb, garbpos, 1);
                rhs = matd_op_gobble_right(expr, pos, rhs, garb, garbpos);

                matd_t *res = matd_subtract(acc, rhs);
                garb[*garbpos] = res;
                (*garbpos)++;
                acc = res;
            }
            break;
        }

        case ' ': {
            // nothing to do. spaces are meaningless.
            (*pos)++;
            break;
        }

        default: {
            fprintf(stderr, "matd_op(): Unknown character: '%c'\n", expr[*pos]);
            assert(expr[*pos] != expr[*pos]);
        }
        }
    }
    return acc;
}

// always returns a new matrix.
matd_t *matd_op(const char *expr, ...)
{
    int nargs = 0;
    int exprlen = 0;

    assert(expr != NULL);

    for (const char *p = expr; *p != 0; p++) {
        if (*p == 'M' || *p == 'F')
            nargs++;
        exprlen++;
    }

    if (!exprlen) // expr = ""
        return NULL;

    va_list ap;
    va_start(ap, expr);

    matd_t *args[nargs];
    for (int i = 0; i < nargs; i++) {
        args[i] = va_arg(ap, matd_t*);
        // XXX: sanity check argument; emit warning/error if args[i]
        // doesn't look like a matd_t*.
    }

    va_end(ap);

    int pos = 0;
    int argpos = 0;
    int garbpos = 0;

    matd_t *garb[2*exprlen]; // can't create more than 2 new result per character
                             // one result, and possibly one argument to free

    matd_t *res = matd_op_recurse(expr, &pos, NULL, args, &argpos, garb, &garbpos, 0);

    // 'res' may need to be freed as part of garbage collection (i.e. expr = "F")
    matd_t *res_copy = (res ? matd_copy(res) : NULL);

    for (int i = 0; i < garbpos; i++) {
        matd_destroy(garb[i]);
    }

    return res_copy;
}

static inline double sq(double v)
{
    return v*v;
}


double matd_vec_mag(const matd_t *a)
{
    assert(a != NULL);
    assert(matd_is_vector(a));

    double mag = 0.0;
    int len = a->nrows*a->ncols;
    for (int i = 0; i < len; i++)
        mag += sq(a->data[i]);
    return sqrt(mag);
}

double matd_vec_dist(const matd_t *a, const matd_t *b)
{
    assert(a != NULL);
    assert(b != NULL);
    assert(matd_is_vector(a) && matd_is_vector(b));

    int lena = a->nrows*a->ncols;
//    int lenb = b->nrows*b->ncols;

    assert((a->nrows*a->ncols) == (b->nrows*b->ncols));

    double mag = 0.0;
    for (int i = 0; i < lena; i++)
        mag += sq(a->data[i] - b->data[i]);
    return sqrt(mag);
}

double matd_vec_dot_product(const matd_t *a, const matd_t *b)
{
    assert(a != NULL);
    assert(b != NULL);
    assert(matd_is_vector(a) && matd_is_vector(b));
    int adim = a->ncols*a->nrows;
//    int bdim = b->ncols*b->nrows;

    assert((a->nrows*a->ncols) == (b->nrows*b->ncols));

    double acc = 0;
    for (int i = 0; i < adim; i++) {
        acc += a->data[i] * b->data[i];
    }
    return acc;
}


matd_t *matd_vec_normalize(const matd_t *a)
{
    assert(a != NULL);
    assert(matd_is_vector(a));

    double mag = matd_vec_mag(a);
    assert(mag > 0);

    matd_t *b = matd_create(a->nrows, a->ncols);

    int len = a->nrows*a->ncols;
    for(int i = 0; i < len; i++)
        b->data[i] = a->data[i] / mag;

    return b;
}

matd_t *matd_crossproduct(const matd_t *a, const matd_t *b)
{ // only defined for vecs (col or row) of length 3
    assert(a != NULL);
    assert(b != NULL);
    assert(matd_is_vector_len(a, 3) && matd_is_vector_len(b, 3));

    matd_t * r = matd_create(a->nrows, a->ncols);

    r->data[0] = a->data[1] * b->data[2] - a->data[2] * b->data[1];
    r->data[1] = a->data[2] * b->data[0] - a->data[0] * b->data[2];
    r->data[2] = a->data[0] * b->data[1] - a->data[1] * b->data[0];

    return r;
}

/*
static TYPE matd_err_inf(const matd_t *a, const matd_t *b)
{
    assert(a->nrows == b->nrows);
    assert(a->ncols == b->ncols);

    TYPE maxf = 0;

    for (int i = 0; i < a->nrows; i++) {
        for (int j = 0; j < a->ncols; j++) {
            TYPE av = MATD_EL(a, i, j);
            TYPE bv = MATD_EL(b, i, j);

            TYPE err = fabs(av - bv);
            maxf = fmax(maxf, err);
        }
    }

    return maxf;
}
*/

/** SVD 2x2.

    A = USV'

    U = [ cos(theta) -sin(theta) ]
        [ sin(theta)  cos(theta) ]

    S = [ e  0 ]
        [ 0  f ]

    V = [ cos(phi)   -sin(phi) ]
        [ sin(phi)   cos(phi)  ]


    Note that A'A = VS'U'USV' = VSSV', which we can expand. (This
    operation lets us just worry about phi, without having to think
    about theta simultaneously.) Let s = sin(phi), c = cos(phi):

    A'A = [ w x ] = [ e^2c^2 + f^2s^2      e^2sc - f^2sc  ]
          [ x y ]   [ e^2sc - f^2sc       e^2s^2 + f^2c^2 ]

    This gives us simultaneous equations:

    e^2c^2 + f^2s^2 = w    (1)
    e^2sc - f^2sc   = x    (2)
    e^2s^2 + f^2c^2 = y    (3)

    e^2 + f^2              = w + y   (1 + 3)
    (e^2 - f^2)(c^2 - s^2) = w - y   (1 - 3)
    (e^2 - f^2)sc          = x       (2)

    (e^2 - f^2)cos(2phi)   = w - y
    (e^2 - f^2)sin(2phi)   = 2x

    tan(2phi) = 2x / (w - y)

    /////////////////////////

    Similarly, we obtain for theta:

    AA' = USV'VSU' = USS'U'

    AA' = [ w' x' ] (though these are different w,x,y than for phi)
          [ x' y' ]

    tan(2theta) = 2x' / (w' - y')

    ////////////////////////

    We now recover S. We could first recover e^2 and f^2, from which
    we might be tempted into concluding that S = diag([e f]). But note
    that all of our math has been in terms of S'S, and there are
    values of S such that S'S = diag[e^2 f^2] even when S does not
    equal diag([e f]). We have done nothing to prevent such a case.

    In particular, this is possible when S can be written S = WD, S'S
    = D'W'WD = D'D. We can trivially recover WD by starting with the
    definition of SVD and using our U and V matrices already
    recovered. A=U(WD)V', thus WD = U'AV. We can factor WD into W and
    D, then incorporate the W term into U in order to obtain our SVD.

    (Consider the example case A = [12 13; 13 -12].) A'A=AA' =
    diag([313 313]). This will yield theta=phi=0, and thus WD=A (and
    is clearly not diagonal!).


    We then solve for the "singular values matrix" with WS = U'AV. WS
is often equal to S, except in the case that Sx */
struct svd22 {
    // A = USV'
    double U[4];
    double S[2]; // just the diagonal elements
    double V[4];
};

void matd_svd22_impl(const double A[4], struct svd22 *svd)
{
    double w, x, y;

    // compute V's phi from A'A
    w = A[0]*A[0] + A[2]*A[2];
    x = A[0]*A[1] + A[2]*A[3];
    y = A[1]*A[1] + A[3]*A[3];

    double phi = 0.5*atan2(2*x, w - y);

    // compute U's theta from AA'
    w = A[0]*A[0] + A[1]*A[1];
    x = A[0]*A[2] + A[1]*A[3];
    y = A[2]*A[2] + A[3]*A[3];

    double theta = 0.5*atan2(2*x, w - y);

    // Now we'll actually construct U and V (which may be modified
    // later...) For now, they're pure rotations.
    double ct = cos(theta), st = sin(theta);
    double cp = cos(phi), sp = sin(phi);

    svd->U[0] = ct;
    svd->U[1] = -st;
    svd->U[2] = st;
    svd->U[3] = ct;

    svd->V[0] = cp;
    svd->V[1] = -sp;
    svd->V[2] = sp;
    svd->V[3] = cp;

    // Solve for the "singular value matrix" WS... if UWSV' = A, then
    // WS = U'AV.

    // WS = U'*A*V
    double T[4] = { A[0]*svd->V[0] + A[1]*svd->V[2],    // T = A*V
                    A[0]*svd->V[1] + A[1]*svd->V[3],
                    A[2]*svd->V[0] + A[3]*svd->V[2],
                    A[2]*svd->V[1] + A[3]*svd->V[3] };

    double WS[4] = { svd->U[0]*T[0] + svd->U[2]*T[2],   // WS = U'*T = U'*A*V
                     svd->U[0]*T[1] + svd->U[2]*T[3],
                     svd->U[1]*T[0] + svd->U[3]*T[2],
                     svd->U[1]*T[1] + svd->U[3]*T[3] };

    svd->S[0] = sqrtf(WS[0]*WS[0] + WS[1]*WS[1]);
    svd->S[1] = sqrtf(WS[2]*WS[2] + WS[3]*WS[3]);

    // Solve for W from WS, being careful to handle singular cases
    // such that W is unitary.
    double eps = 0.0000001;
    double W[4];
    if (svd->S[0] > eps) {
        W[0] = WS[0] / svd->S[0];
        W[2] = WS[2] / svd->S[0];
    } else {
        W[0] = 1;
        W[2] = 0;
    }

    if (svd->S[1] > eps) {
        W[1] = WS[1] / svd->S[1];
        W[3] = WS[3] / svd->S[1];
    } else {
        W[1] = 0;
        W[3] = 1;
    }

    // updated U = UW
    double UW[4] = { svd->U[0]*W[0] + svd->U[1]*W[2],
                     svd->U[0]*W[1] + svd->U[1]*W[3],
                     svd->U[2]*W[0] + svd->U[3]*W[2],
                     svd->U[2]*W[1] + svd->U[3]*W[3] };

    memcpy(svd->U, UW, 4*sizeof(double));

    assert(svd->S[0] >= 0);
    assert(svd->S[1] >= 0);

    // sort singular values.
    if (abs(svd->S[1]) > abs(svd->S[0])) {
        // Curiously, this code never seems to get invoked.  Why is it
        // that S[0] always ends up the dominant vector?  However,
        // this code has been tested (flipping the logic forces us to
        // sort the singular values in ascending order).
        //
        // P = [ 0 1 ; 1 0 ]
        // USV' = (UP)(PSP)(PV')
        //      = (UP)(PSP)(VP)'
        //      = (UP)(PSP)(P'V')'
        double t = svd->S[0];
        svd->S[0] = svd->S[1];
        svd->S[1] = t;

        // exchange columns of U and V
        double s[2];
        s[0] = svd->U[0];
        s[1] = svd->U[2];
        svd->U[0] = svd->U[1];
        svd->U[2] = svd->U[3];
        svd->U[1] = s[0];
        svd->U[3] = s[1];

        s[0] = svd->V[0];
        s[1] = svd->V[2];
        svd->V[0] = svd->V[1];
        svd->V[2] = svd->V[3];
        svd->V[1] = s[0];
        svd->V[3] = s[1];
    }
}

// Computes an SVD for square or tall matrices. This code doesn't work
// for wide matrices, because the bidiagonalization results in one
// non-zero element too far to the right for us to rotate away.
//
// Caller is responsible for destroying U, S, and V.
static matd_svd_t matd_svd_tall(matd_t *A)
{
    matd_t *B = matd_copy(A);

    // Apply householder reflections on each side to reduce A to
    // bidiagonal form. Specifically:
    //
    // A = LS*B*RS'
    //
    // Where B is bidiagonal, and LS/RS are unitary.
    //
    // Why are we doing this? Some sort of transformation is necessary
    // to reduce the matrix's nz elements to a square region. QR could
    // work too. We need nzs confined to a square region so that the
    // subsequent iterative process, which is based on rotations, can
    // work. (To zero out a term at (i,j), our rotations will also
    // affect (j,i).
    //
    // We prefer bidiagonalization over QR because it gets us "closer"
    // to the SVD, which should mean fewer iterations.

    // LS: cumulative left-handed transformations
    matd_t *LS = matd_identity(A->nrows);

    // RS: cumulative right-handed transformations.
    matd_t *RS = matd_identity(A->ncols);

    for (int hhidx = 0; hhidx < A->nrows; hhidx++)  {

        if (hhidx + 1 < A->ncols) {
            // We construct the normal of the reflection plane: let u
            // be the vector to reflect, x =[ M 0 0 0 ] the target
            // location for u (u') after reflection (with M = ||u||).
            //
            // The normal vector is then n = (u - x), but since we
            // could equally have the target location be x = [-M 0 0 0
            // ], we could use n = (u + x).
            //
            // We then normalize n. To ensure a reasonable magnitude,
            // we select the sign of M so as to maximize the magnitude
            // of the first element of (x +/- M). (Otherwise, we could
            // end up with a divide-by-zero if u[0] and M cancel.)
            //
            // The householder reflection matrix is then H=(I - nn'), and
            // u' = Hu.
            //
            //
            int vlen = A->nrows - hhidx;

            double v[vlen];

            double mag2 = 0;
            for (int i = 0; i < vlen; i++) {
                v[i] = MATD_EL(B, hhidx+i, hhidx);
                mag2 += v[i]*v[i];
            }

            double oldv0 = v[0];
            if (oldv0 < 0)
                v[0] -= sqrtf(mag2);
            else
                v[0] += sqrtf(mag2);

            mag2 += -oldv0*oldv0 + v[0]*v[0];

            // normalize v
            double mag = sqrt(mag2);
            for (int i = 0; i < vlen; i++)
                v[i] /= mag;

            // Q = I - 2vv'
            //matd_t *Q = matd_identity(A->nrows);
            //for (int i = 0; i < vlen; i++)
            //  for (int j = 0; j < vlen; j++)
            //    MATD_EL(Q, i+hhidx, j+hhidx) -= 2*v[i]*v[j];


            // LS = matd_op("F*M", LS, Q);
            // Implementation: take each row of LS, compute dot product with n,
            // subtract n (scaled by dot product) from it.
            for (int i = 0; i < LS->nrows; i++) {
                double dot = 0;
                for (int j = 0; j < vlen; j++)
                    dot += MATD_EL(LS, i, hhidx+j) * v[j];
                for (int j = 0; j < vlen; j++)
                    MATD_EL(LS, i, hhidx+j) -= 2*dot*v[j];
            }

            //  B = matd_op("M*F", Q, B); // should be Q', but Q is symmetric.
            for (int i = 0; i < B->ncols; i++) {
                double dot = 0;
                for (int j = 0; j < vlen; j++)
                    dot += MATD_EL(B, hhidx+j, i) * v[j];
                for (int j = 0; j < vlen; j++)
                    MATD_EL(B, hhidx+j, i) -= 2*dot*v[j];
            }
        }

        if (hhidx+2 < A->ncols) {
            int vlen = A->ncols - hhidx - 1;

            double v[vlen];

            double mag2 = 0;
            for (int i = 0; i < vlen; i++) {
                v[i] = MATD_EL(B, hhidx, hhidx+i+1);
                mag2 += v[i]*v[i];
            }

            double oldv0 = v[0];
            if (oldv0 < 0)
                v[0] -= sqrtf(mag2);
            else
                v[0] += sqrtf(mag2);

            mag2 += -oldv0*oldv0 + v[0]*v[0];

            // compute magnitude of ([1 0 0..]+v)
            double mag = sqrt(mag2);

            for (int i = 0; i < vlen; i++)
                v[i] /= mag;

            // TODO: optimize these multiplications
            // matd_t *Q = matd_identity(A->ncols);
            //  for (int i = 0; i < vlen; i++)
            //    for (int j = 0; j < vlen; j++)
            //       MATD_EL(Q, i+1+hhidx, j+1+hhidx) -= 2*v[i]*v[j];

            //  RS = matd_op("F*M", RS, Q);
            for (int i = 0; i < RS->nrows; i++) {
                double dot = 0;
                for (int j = 0; j < vlen; j++)
                    dot += MATD_EL(RS, i, hhidx+1+j) * v[j];
                for (int j = 0; j < vlen; j++)
                    MATD_EL(RS, i, hhidx+1+j) -= 2*dot*v[j];
            }

            //   B = matd_op("F*M", B, Q); // should be Q', but Q is symmetric.
            for (int i = 0; i < B->nrows; i++) {
                double dot = 0;
                for (int j = 0; j < vlen; j++)
                    dot += MATD_EL(B, i, hhidx+1+j) * v[j];
                for (int j = 0; j < vlen; j++)
                    MATD_EL(B, i, hhidx+1+j) -= 2*dot*v[j];
            }
        }
    }

/*
    double bidiag_err = matd_err_inf(matd_op("M*M*M'", LS, B, RS), A);
    if (bidiag_err > 1e-8) {
        matd_print(A, "%15.10f");
        printf("\n");
        matd_print(matd_op("M*M*M'", LS, B, RS), "%15.10f");
        printf("bidiag_err: %.10f\n", bidiag_err);
    }
*/

    // empirically, we find a roughly linear worst-case number of iterations
    // as a function of rows*cols. maxiters ~= 1.5*nrows*ncols
    // we're a bit conservative below.
    int maxiters = 150 + A->nrows*A->ncols*5;
    int iter;
    for (iter = 0; iter < maxiters; iter++) {

        // find the largest off-diagonal element of B
        //
        // XXX this "search from scratch" approach is simple but
        // wasteful.. However, probably a small contributor to overall
        // runtime.
        int maxi = -1, maxj = -1;
        double max = -1;
        for (int i = 0; i < A->ncols; i++) {
            for (int j = 0; j < A->ncols; j++) {
                if (i == j)
                    continue;

                if (fabs(MATD_EL(B, i, j)) > max) {
                    maxi = i;
                    maxj = j;
                    max = fabs(MATD_EL(B, i, j));
                }
            }
        }

        // termination condition.
        if (max < 1E-8)
            break;

        // Now, solve the 2x2 SVD problem for the matrix
        // [ A0 A1 ]
        // [ A2 A3 ]
        double A0 = MATD_EL(B, maxi, maxi);
        double A1 = MATD_EL(B, maxi, maxj);
        double A2 = MATD_EL(B, maxj, maxi);
        double A3 = MATD_EL(B, maxj, maxj);

        double w = A0*A0 + A2*A2;
        double x = A0*A1 + A2*A3;
        double y = A1*A1 + A3*A3;

        double phi = .5 * atan2(2*x, w-y);

        w = A0*A0 + A1*A1;
        x = A0*A2 + A1*A3;
        y = A2*A2 + A3*A3;

        double theta = .5 * atan2(2*x, w-y);

        // LS = LS * ROT(theta)
        //matd_t *QL = matd_identity(A->nrows);
        //MATD_EL(QL, maxi, maxi) = cos(theta);
        //MATD_EL(QL, maxi, maxj) = -sin(theta);
        //MATD_EL(QL, maxj, maxi) = sin(theta);
        //MATD_EL(QL, maxj, maxj) = cos(theta);

        //matd_t *QR = matd_identity(A->ncols);
        //MATD_EL(QR, maxi, maxi) = cos(phi);
        //MATD_EL(QR, maxi, maxj) = -sin(phi);
        //MATD_EL(QR, maxj, maxi) = sin(phi);
        //MATD_EL(QR, maxj, maxj) = cos(phi);

        // TODO: optimize the three multiplications below.
        if (1) {
            //  LS = matd_op("F*M", LS, QL);
            double c = cos(theta), s = sin(theta);

            for (int i = 0; i < LS->nrows; i++) {
                double vi = MATD_EL(LS, i, maxi);
                double vj = MATD_EL(LS, i, maxj);

                MATD_EL(LS, i, maxi) = c*vi + s*vj;
                MATD_EL(LS, i, maxj) = -s*vi + c*vj;
            }

            //  RS = matd_op("F*M", RS, QR); // remember we'll transpose RS.
            double d = cos(phi), t = sin(phi);

            for (int i = 0; i < RS->nrows; i++) {
                double vi = MATD_EL(RS, i, maxi);
                double vj = MATD_EL(RS, i, maxj);

                MATD_EL(RS, i, maxi) = d*vi + t*vj;
                MATD_EL(RS, i, maxj) = -t*vi + d*vj;
            }

            // B = matd_op("M'*F*M", QL, B, QR);
            // The QL matrix mixes rows of B.
            for (int i = 0; i < B->ncols; i++) {
                double vi = MATD_EL(B, maxi, i);
                double vj = MATD_EL(B, maxj, i);

                MATD_EL(B, maxi, i) = c*vi + s*vj;
                MATD_EL(B, maxj, i) = -s*vi + c*vj;
            }

            // The QR matrix mixes columns of B.
            for (int i = 0; i < B->nrows; i++) {
                double vi = MATD_EL(B, i, maxi);
                double vj = MATD_EL(B, i, maxj);

                MATD_EL(B, i, maxi) = d*vi + t*vj;
                MATD_EL(B, i, maxj) = -t*vi + d*vj;
            }
        }
    }

    if (iter == maxiters) {
        printf("WARNING: maximum iters (%d)\n", iter);
    }

//    printf("Unfixed SVD check: should be zero:\n");
//    matd_t *Acheck2 = matd_op("M*M*M'-M", LS, B, RS, A);
//    matd_print(Acheck2, "%15f");

//    printf("iter: %d\n", iter);

//    matd_print(B, "%15f");

    // Now, the fixups. Sort the singular values by magnitude and make
    // them all positive by flipping the corresponding columns of
    // U/LS.
    int idxs[A->ncols];
    double vals[A->ncols];
    for (int i = 0; i < A->ncols; i++) {
        idxs[i] = i;
        vals[i] = MATD_EL(B, i, i);
    }

    // A bubble sort. Seriously.
    int changed;
    do {
        changed = 0;

        for (int i = 0; i + 1 < A->ncols; i++) {
            if (fabs(vals[i+1]) > fabs(vals[i])) {
                int tmpi = idxs[i];
                idxs[i] = idxs[i+1];
                idxs[i+1] = tmpi;

                double tmpv = vals[i];
                vals[i] = vals[i+1];
                vals[i+1] = tmpv;

                changed = 1;
            }
        }
    } while (changed);

    matd_t *LP = matd_identity(A->nrows);
    matd_t *RP = matd_identity(A->ncols);

    // We currently have:
    //
    // LS * B * RS'
    //
    // compute the permutation matrices that will re-order our diagonal
    // matrix to be in descending order of magnitude.
    //
    // B2 = LP*B*RP'
    //
    // We will then do:
    //
    // LS * LP' * LP * B * RP' * RP * RS =
    // (LS*LP')*(B2)*(RS*RP')' =
    //   LS2  * B2 * RS2'    =
    //    U     S     V'
    for (int i = 0; i < A->ncols; i++) {
        MATD_EL(LP, i, i) = 0; // undo the identity above
        MATD_EL(RP, i, i) = 0;

        MATD_EL(LP, i, idxs[i]) = vals[i] < 0 ? -1 : 1;
        MATD_EL(RP, i, idxs[i]) = 1;
    }

    // solve for B2.
    // XXX permutations could be done more efficiently than matrix multiply.
    B = matd_op("M*F*M'", LP, B, RP);

    // update LS and RS, remembering that RS will be transposed.
    // XXX permutation could be done more efficiently than matrix multiply.
    LS = matd_op("F*M'", LS, LP);
    RS = matd_op("F*M'", RS, RP);

    matd_destroy(LP);
    matd_destroy(RP);

//    printf("Fixed SVD: should be zero:\n");
//    matd_t *Acheck3 = matd_op("M*M*M'-M", LS, B, RS, A);
//    matd_print(Acheck3, "%15f");

    matd_svd_t res;
    memset(&res, 0, sizeof(res));

    res.U = LS;
    res.S = B;
    res.V = RS;

    return res;
}

matd_svd_t matd_svd(matd_t *A)
{
    matd_svd_t res;

    if (A->ncols <= A->nrows) {
        res = matd_svd_tall(A);
    } else {
        matd_t *At = matd_transpose(A);

        // A =U  S  V'
        // A'=V  S' U'

        matd_svd_t tmp = matd_svd_tall(At);

        memset(&res, 0, sizeof(res));
        res.U = tmp.V; //matd_transpose(tmp.V);
        res.S = matd_transpose(tmp.S);
        res.V = tmp.U; //matd_transpose(tmp.U);

        matd_destroy(tmp.S);
        matd_destroy(At);
    }

/*
    matd_t *check = matd_op("M*M*M'-M", res.U, res.S, res.V, A);
    double maxerr = 0;

    for (int i = 0; i < check->nrows; i++)
        for (int j = 0; j < check->ncols; j++)
            maxerr = fmax(maxerr, fabs(MATD_EL(check, i, j)));

    matd_destroy(check);

    if (maxerr > 1e-7) {
        printf("bad maxerr: %15f\n", maxerr);
    }

    if (maxerr > 1e-5) {
        printf("bad maxerr: %15f\n", maxerr);
        matd_print(A, "%15f");
        assert(0);
    }

*/
    return res;
}


matd_lu_t *matd_lu(const matd_t *a)
{
    int *piv = calloc(a->nrows, sizeof(int));
    int pivsign = 1;
    matd_t *lu = matd_copy(a);

    matd_lu_t *mlu = calloc(1, sizeof(matd_lu_t));

    for (int i = 0; i < a->nrows; i++)
        piv[i] = i;

    for (int j = 0; j < a->ncols; j++) {
        for (int i = 0; i < a->nrows; i++) {
            int kmax = i < j ? i : j; // min(i,j)

            // compute dot product of row i with column j (up through element kmax)
            double acc = 0;
            for (int k = 0; k < kmax; k++)
                acc += MATD_EL(lu, i, k) * MATD_EL(lu, k, j);

            MATD_EL(lu, i, j) -= acc;
        }

        // find pivot and exchange if necessary.
        int p = j;
        if (1) {
            for (int i = j+1; i < lu->nrows; i++) {
                if (fabs(MATD_EL(lu,i,j)) > fabs(MATD_EL(lu, p, j))) {
                    p = i;
                }
            }
        }

        // swap rows p and j?
        if (p != j) {
            TYPE tmp[lu->ncols];
            memcpy(tmp, &MATD_EL(lu, p, 0), sizeof(TYPE) * lu->ncols);
            memcpy(&MATD_EL(lu, p, 0), &MATD_EL(lu, j, 0), sizeof(TYPE) * lu->ncols);
            memcpy(&MATD_EL(lu, j, 0), tmp, sizeof(TYPE) * lu->ncols);
            int k = piv[p];
            piv[p] = piv[j];
            piv[j] = k;
            pivsign = -pivsign;
        }

        double LUjj = MATD_EL(lu, j, j);

        // If our pivot is very small (which means the matrix is
        // singular or nearly singular), replace with a new pivot of the
        // right sign.
        if (fabs(LUjj) < MATD_EPS) {
            if (LUjj < 0)
                LUjj = -MATD_EPS;
            else
                LUjj = MATD_EPS;

            MATD_EL(lu, j, j) = LUjj;

            mlu->singular = 1;
        }

        if (j < lu->ncols && j < lu->nrows && LUjj != 0) {
            LUjj = 1.0 / LUjj;
            for (int i = j+1; i < lu->nrows; i++)
                MATD_EL(lu, i, j) *= LUjj;
        }
    }

    mlu->lu = lu;
    mlu->piv = piv;
    mlu->pivsign = pivsign;

    return mlu;
}

void matd_lu_destroy(matd_lu_t *mlu)
{
    matd_destroy(mlu->lu);
    free(mlu->piv);
    free(mlu);
}

double matd_lu_det(const matd_lu_t *mlu)
{
    matd_t *lu = mlu->lu;
    double det = mlu->pivsign;

    if (lu->nrows == lu->ncols) {
        for (int i = 0; i < lu->ncols; i++)
            det *= MATD_EL(lu, i, i);
    }

    return det;
}

matd_t *matd_lu_l(const matd_lu_t *mlu)
{
    matd_t *lu = mlu->lu;

    matd_t *L = matd_create(lu->nrows, lu->ncols);
    for (int i = 0; i < lu->nrows; i++) {
        for (int j = 0; j < lu->ncols; j++) {
            if (i > j)
                MATD_EL(L, i, j) = MATD_EL(lu, i, j);
            else if (i == j)
                MATD_EL(L, i, j) = 1;
        }
    }

    return L;
}

matd_t *matd_lu_u(const matd_lu_t *mlu)
{
    matd_t *lu = mlu->lu;

    matd_t *U = matd_create(lu->ncols, lu->ncols);
    for (int i = 0; i < lu->ncols; i++) {
        for (int j = 0; j < lu->ncols; j++) {
            if (i <= j)
                MATD_EL(U, i, j) = MATD_EL(lu, i, j);
        }
    }

    return U;
}

matd_t *matd_lu_solve(const matd_lu_t *mlu, const matd_t *b)
{
    matd_t *x = matd_copy(b);

    // permute right hand side
    for (int i = 0; i < mlu->lu->nrows; i++)
        memcpy(&MATD_EL(x, mlu->piv[i], 0), &MATD_EL(b, i, 0), sizeof(TYPE) * b->ncols);

    // solve Ly = b
    for (int k = 0; k < mlu->lu->nrows; k++) {
        for (int i = k+1; i < mlu->lu->nrows; i++) {
            double LUik = -MATD_EL(mlu->lu, i, k);
            for (int t = 0; t < b->ncols; t++)
                MATD_EL(x, i, t) += MATD_EL(x, k, t) * LUik;
        }
    }

    // solve Ux = y
    for (int k = mlu->lu->ncols-1; k >= 0; k--) {
        double LUkk = 1.0 / MATD_EL(mlu->lu, k, k);
        for (int t = 0; t < b->ncols; t++)
            MATD_EL(x, k, t) *= LUkk;

        for (int i = 0; i < k; i++) {
            double LUik = -MATD_EL(mlu->lu, i, k);
            for (int t = 0; t < b->ncols; t++)
                MATD_EL(x, i, t) += MATD_EL(x, k, t) *LUik;
        }
    }

    return x;
}

matd_t *matd_solve(matd_t *A, matd_t *b)
{
    matd_lu_t *mlu = matd_lu(A);
    matd_t *x = matd_lu_solve(mlu, b);

    matd_lu_destroy(mlu);
    return x;
}


#if 0

static int randi()
{
    int v = random()&31;
    v -= 15;
    return v;
}

static double randf()
{
    double v = 1.0 *random() / RAND_MAX;
    return 2*v - 1;
}

int main(int argc, char *argv[])
{
    if (1) {
        int maxdim = 5;
        matd_t *A = matd_create(maxdim, maxdim);

        for (int row = 0; row < A->nrows; row++) {
            for (int col = 0; col < A->ncols; col++) {

                MATD_EL(A, row, col) = randi();
            }
        }

        matd_t *Ainv = matd_inverse(A);
        matd_print(matd_op("M*F", A, Ainv), "%15f");

        for (int iter = 0; 1; iter++) {
            srand(iter);

            if (iter % 1000 == 0)
                printf("%d\n", iter);

            int m = 1 + (random()%(maxdim-1));
            int n = 1 + (random()%(maxdim-1));

            for (int i = 0; i < m*n; i++)
                A->data[i] = randi();

            A->nrows = m;
            A->ncols = n;

//            printf("%d %d ", m, n);
            matd_svd_t svd = matd_svd(A);
            matd_destroy(svd.U);
            matd_destroy(svd.S);
            matd_destroy(svd.V);

        }

/*        matd_t *A = matd_create_data(2, 5, (double[]) { 1, 5, 2, 6,
                    3, 3, 0, 7,
                    1, 1, 0, -2,
                    4, 0, 9, 9, 2, 6, 1, 3, 2, 5, 5, 4, -1, 2, 5, 9, 8, 2 });

        matd_svd(A);
*/
        return 0;
    }


    struct svd22 s;

    srand(0);

    matd_t *A = matd_create(2, 2);
    MATD_EL(A,0,0) = 4;
    MATD_EL(A,0,1) = 7;
    MATD_EL(A,1,0) = 2;
    MATD_EL(A,1,1) = 6;

    matd_t *U = matd_create(2, 2);
    matd_t *V = matd_create(2, 2);
    matd_t *S = matd_create(2, 2);

    for (int iter = 0; 1; iter++) {
        if (iter % 100000 == 0)
            printf("%d\n", iter);

        MATD_EL(A,0,0) = randf();
        MATD_EL(A,0,1) = randf();
        MATD_EL(A,1,0) = randf();
        MATD_EL(A,1,1) = randf();

        matd_svd22_impl(A->data, &s);

        memcpy(U->data, s.U, 4*sizeof(double));
        memcpy(V->data, s.V, 4*sizeof(double));
        MATD_EL(S,0,0) = s.S[0];
        MATD_EL(S,1,1) = s.S[1];

        assert(s.S[0] >= s.S[1]);
        assert(s.S[0] >= 0);
        assert(s.S[1] >= 0);
        if (s.S[0] == 0) {
//            printf("*"); fflush(NULL);
//            printf("%15f %15f %15f %15f\n", MATD_EL(A,0,0), MATD_EL(A,0,1), MATD_EL(A,1,0), MATD_EL(A,1,1));
        }
        if (s.S[1] == 0) {
//            printf("#"); fflush(NULL);
        }

        matd_t *USV = matd_op("M*M*M'", U, S, V);

        double maxerr = 0;
        for (int i = 0; i < 4; i++)
            maxerr = fmax(maxerr, fabs(USV->data[i] - A->data[i]));

        if (0) {
            printf("------------------------------------\n");
            printf("A:\n");
            matd_print(A, "%15f");
            printf("\nUSV':\n");
            matd_print(USV, "%15f");
            printf("maxerr: %.15f\n", maxerr);
            printf("\n\n");
        }

        matd_destroy(USV);

        assert(maxerr < 0.00001);
    }
}


#endif

