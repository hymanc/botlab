#include <assert.h>
#include <math.h>
#include <stdio.h>

#include "common/string_util.h"
#include "common/timeprofile.h"

#include "math_util.h"
#include "smatd.h"
#include "april_graph.h"

int *exact_minimum_degree_ordering(smatd_t *mat);

double alt_mod2pi(double v)
{
    while (v > M_PI)
        v -= 2*M_PI;
    while (v < -M_PI)
        v += 2*M_PI;
    return v;
}

struct tok_feeder
{
    zarray_t *toks;
    int pos;
};

static int tf_int(struct tok_feeder *tf)
{
    char *tok;
    zarray_get(tf->toks, tf->pos++, &tok);
    return atoi(tok);
}

static double tf_double(struct tok_feeder *tf)
{
    char *tok;
    zarray_get(tf->toks, tf->pos++, &tok);
    return atof(tok);
}

static char* tf_string(struct tok_feeder *tf)
{
    char *tok;
    zarray_get(tf->toks, tf->pos++, &tok);
    return tok;
}

april_graph_t *april_graph_create_from_file(const char *path)
{
    april_graph_t *graph = april_graph_create();

    FILE *f = fopen(path, "r");
    char line[1024];

    while (fgets(line, sizeof(line), f) != NULL) {

        zarray_t *toks = str_split(line, " ");
        struct tok_feeder tf = { .toks = toks, .pos = 0 };

        char *type = tf_string(&tf);

        if (!strcmp(type, "xytnode") && zarray_size(toks)==10) {

            double *state = (double[]) { tf_double(&tf), tf_double(&tf), tf_double(&tf) };
            double  *init = (double[]) { tf_double(&tf), tf_double(&tf), tf_double(&tf) };
            double *truth = (double[]) { tf_double(&tf), tf_double(&tf), tf_double(&tf) };

            april_graph_node_t *n = april_graph_node_xyt_create(state, init, truth);

            zarray_add(graph->nodes, &n);

        } else if (!strcmp(type, "xytedge") && zarray_size(toks)==15) {

            int a = tf_int(&tf), b = tf_int(&tf);
            double *z      = (double[]) { tf_double(&tf), tf_double(&tf), tf_double(&tf) };
            double *ztruth = (double[]) { tf_double(&tf), tf_double(&tf), tf_double(&tf) };

            matd_t *P = matd_create_data(3, 3,
                                         (double[]) { tf_double(&tf), tf_double(&tf), tf_double(&tf),
                                                 0,              tf_double(&tf), tf_double(&tf),
                                                 0,              0,              tf_double(&tf) });
            for (int i = 0; i < 3; i++)
                for (int j = i+1; j < 3; j++)
                    MATD_EL(P, j, i) = MATD_EL(P, i, j);

            matd_t *W = matd_op("M^-1", P);

            april_graph_factor_t *f = april_graph_factor_xyt_create(a, b,
                                                                    z, ztruth,
                                                                    W);

            matd_destroy(P);
            matd_destroy(W);

            zarray_add(graph->factors, &f);

        } else {
            printf("Unknown type '%s' with %d tokens\n", type, zarray_size(toks));
        }

        zarray_vmap(toks, free);
        zarray_destroy(toks);
    }

    fclose(f);

    return graph;
}

april_graph_t *april_graph_create()
{
    april_graph_t *graph = calloc(1, sizeof(april_graph_t));

    graph->factors = zarray_create(sizeof(april_graph_factor_t*));
    graph->nodes = zarray_create(sizeof(april_graph_node_t*));

    return graph;
}

void april_graph_destroy(april_graph_t *graph)
{
    for (int i = 0; i < zarray_size(graph->nodes); i++) {
        april_graph_node_t *n;
        zarray_get(graph->nodes, i, &n);

        n->destroy(n);
    }

    for (int i = 0; i < zarray_size(graph->factors); i++) {
        april_graph_factor_t *factor;
        zarray_get(graph->factors, i, &factor);

        factor->destroy(factor);
    }

    zarray_destroy(graph->nodes);
    zarray_destroy(graph->factors);

    free(graph);
}

int april_graph_dof(april_graph_t *graph)
{
    int factor_dof = 0, state_dof = 0;

    for (int i = 0; i < zarray_size(graph->nodes); i++) {
        april_graph_node_t *n;
        zarray_get(graph->nodes, i, &n);

        state_dof += n->length;
    }

    for (int i = 0; i < zarray_size(graph->factors); i++) {
        april_graph_factor_t *factor;
        zarray_get(graph->factors, i, &factor);

        factor_dof += factor->length;
    }

    return factor_dof - state_dof;
}

double april_graph_chi2(april_graph_t *graph)
{
    double chi2 = 0;

    for (int i = 0; i < zarray_size(graph->factors); i++) {
        april_graph_factor_t *factor;
        zarray_get(graph->factors, i, &factor);
        april_graph_factor_eval_t *eval = factor->eval(factor, graph, NULL);

        chi2 += eval->chi2;

        april_graph_factor_eval_destroy(eval);
    }

    return chi2;
}

void april_graph_postscript(april_graph_t *graph, const char *path)
{
    FILE *f = fopen(path, "w");
    fprintf(f, "%%!PS\n\n");

    fprintf(f, "0 setlinewidth\n");

    double x0 = HUGE, x1 = -HUGE, y0 = HUGE, y1 = -HUGE;

    double page_width = 612, page_height = 792;

    for (int i = 0; i < zarray_size(graph->nodes); i++) {
        april_graph_node_t *n;
        zarray_get(graph->nodes, i, &n);

        if (n->type == APRIL_GRAPH_NODE_XYT_TYPE) {
            x0 = fmin(x0, n->state[0]);
            x1 = fmax(x1, n->state[0]);
            y0 = fmin(y0, n->state[1]);
            y1 = fmax(y1, n->state[1]);
        }
    }

    double s = fmin(page_width / (x1-x0), page_height / (y1-y0));

    fprintf(f, "/node {\n"); // invocation: x y theta node
    fprintf(f, "gsave\n");
    fprintf(f, "3 1 roll\n"); // convert tx ty theta => theta tx ty
    fprintf(f, "translate\n");
    fprintf(f, "57.296 mul rotate\n");
    fprintf(f, ".2 .2 scale\n"); // size of robot
    fprintf(f, ".5 setgray\n");
    fprintf(f, "newpath\n-1 -1 moveto\n2 0 lineto\n-1 1 lineto\nclosepath\nfill\n");
    fprintf(f, "grestore\n");
    fprintf(f, "} def\n");
    fprintf(f, "\n");

    fprintf(f, "/edge {\n"); // invocation: x0 y0 x1 y1 dposes edge
    fprintf(f, "gsave\n");
//    fprintf(f, "pop\n");
    fprintf(f, "1 eq { 0 0 1 setrgbcolor } { 0 1 1 setrgbcolor } ifelse\n");
    fprintf(f, "moveto lineto stroke\n");
    fprintf(f, "grestore\n");
    fprintf(f, "} def\n");
    fprintf(f, "\n");

    fprintf(f, "%f %f translate\n", page_width / 2, page_height / 2);
    fprintf(f, "%f %f scale\n", s, s);
    fprintf(f, "%f %f translate\n", -(x0+x1)/2, -(y0+y1)/2);

    for (int i = 0; i < zarray_size(graph->factors); i++) {
        april_graph_factor_t *factor;
        zarray_get(graph->factors, i, &factor);

        if (factor->type != APRIL_GRAPH_FACTOR_XYT_TYPE)
            continue;

        april_graph_node_t *na, *nb;
        zarray_get(graph->nodes, factor->nodes[0], &na);
        zarray_get(graph->nodes, factor->nodes[1], &nb);

        fprintf(f, "%f %f %f %f %d edge\n",
                na->state[0], na->state[1], nb->state[0], nb->state[1],
                abs(factor->nodes[1] - factor->nodes[0]));
    }

    for (int i = 0; i < zarray_size(graph->nodes); i++) {
        april_graph_node_t *n;
        zarray_get(graph->nodes, i, &n);

        if (n->type != APRIL_GRAPH_NODE_XYT_TYPE)
            continue;

        fprintf(f, "%f %f %f node\n", n->state[0], n->state[1], n->state[2]);
    }

    fclose(f);
}

/////////////////////////////////////////////////////////////////////////////////////////
// XYT Factor
static april_graph_factor_t* xyt_factor_copy(april_graph_factor_t *factor)
{
    assert(0);
    return NULL;
}

static april_graph_factor_eval_t* xyt_factor_eval(april_graph_factor_t *factor, april_graph_t *graph, april_graph_factor_eval_t *eval)
{
    if (eval == NULL) {
        eval = calloc(1, sizeof(april_graph_factor_eval_t));
        eval->jacobians = calloc(3, sizeof(matd_t*)); // NB: NULL-terminated
        eval->jacobians[0] = matd_create(3,3);
        eval->jacobians[1] = matd_create(3,3);
        eval->r = calloc(3, sizeof(double));
        eval->W = matd_create(3,3);
    }

    april_graph_node_t *na, *nb;
    zarray_get(graph->nodes, factor->nodes[0], &na);
    zarray_get(graph->nodes, factor->nodes[1], &nb);

    double xa = na->state[0], ya = na->state[1], ta = na->state[2];
    double xb = nb->state[0], yb = nb->state[1], tb = nb->state[2];

    double ca = cos(ta), sa = sin(ta);

    // predicted obs
    double dx = xb - xa, dy = yb - ya;
    double zhat[3];
    zhat[0] =  ca*dx + sa*dy;
    zhat[1] = -sa*dx + ca*dy;
    zhat[2] =  tb    - ta;

    // partial derivatives of zhat WRT node 0 [xa ya ta]
    matd_set_data(eval->jacobians[0], (double[]) {
               -ca,  -sa, -sa*dx+ca*dy,
                sa,  -ca, -ca*dx-sa*dy,
                0,     0,   -1 });

    // partial derivatives of zhat WRT node 1 [xb yb tb]
    matd_set_data(eval->jacobians[1], (double[]) {
                ca,  sa, 0,
                -sa, ca, 0,
                0,    0, 1 });


    eval->length = 3;

    eval->r[0] = factor->z[0] - zhat[0];
    eval->r[1] = factor->z[1] - zhat[1];
    eval->r[2] = alt_mod2pi(factor->z[2] - zhat[2]);

    memcpy(eval->W->data, factor->W->data, 3*3*sizeof(double));

    // chi^2 = r'*W*r, via X = W*r
    double X[3] = { MATD_EL(eval->W, 0, 0)*eval->r[0] +
                    MATD_EL(eval->W, 0, 1)*eval->r[1] +
                    MATD_EL(eval->W, 0, 2)*eval->r[2],
                    MATD_EL(eval->W, 1, 0)*eval->r[0] +
                    MATD_EL(eval->W, 1, 1)*eval->r[1] +
                    MATD_EL(eval->W, 1, 2)*eval->r[2],
                    MATD_EL(eval->W, 2, 0)*eval->r[0] +
                    MATD_EL(eval->W, 2, 1)*eval->r[1] +
                    MATD_EL(eval->W, 2, 2)*eval->r[2] };
    eval->chi2 = eval->r[0]*X[0] + eval->r[1]*X[1] + eval->r[2]*X[2];

    return eval;
}

void xyt_node_update(april_graph_node_t *node, double *dstate)
{
    for (int i = 0; i < 3; i++)
        node->state[i] += dstate[i];
}

static april_graph_node_t* xyt_node_copy(april_graph_node_t *node)
{
    assert(0);
    return NULL;
}

static void xyt_factor_destroy(april_graph_factor_t *factor)
{
    free(factor->nodes);
    free(factor->z);
    free(factor->ztruth);
    matd_destroy(factor->W);
    free(factor);
}

april_graph_factor_t *april_graph_factor_xyt_create(int a, int b, double *z, double *ztruth, matd_t *W)
{
    april_graph_factor_t *factor = calloc(1, sizeof(april_graph_factor_t));

    factor->type = APRIL_GRAPH_FACTOR_XYT_TYPE;
    factor->nnodes = 2;
    factor->nodes = calloc(2, sizeof(int));
    factor->nodes[0] = a;
    factor->nodes[1] = b;
    factor->length = 3;

    factor->copy = xyt_factor_copy;
    factor->eval = xyt_factor_eval;
    factor->destroy = xyt_factor_destroy;

    factor->z = doubles_copy(z, 3);
    factor->ztruth = doubles_copy(ztruth, 3);
    factor->W = matd_copy(W);

    return factor;
}

/////////////////////////////////////////////////////////////////////////////////////////
// XYTPos Factor
static april_graph_factor_t* xytpos_factor_copy(april_graph_factor_t *factor)
{
    assert(0);
    return NULL;

}

static april_graph_factor_eval_t* xytpos_factor_eval(april_graph_factor_t *factor, april_graph_t *graph, april_graph_factor_eval_t *eval)
{
    if (eval == NULL) {
        eval = calloc(1, sizeof(april_graph_factor_eval_t));
        eval->jacobians = calloc(2, sizeof(matd_t*)); // NB: NULL-terminated
        eval->jacobians[0] = matd_create(3, 3);
        eval->r = calloc(3, sizeof(double));
        eval->W = matd_create(3,3);
    }

    // partial derivatives of zhat WRT node 0 [xa ya ta]
    matd_set_data(eval->jacobians[0], (double[]) {
                1, 0, 0,
                0, 1, 0,
                0, 0, 1 });

    april_graph_node_t *na;
    zarray_get(graph->nodes, factor->nodes[0], &na);

    eval->length = 3;

    eval->r[0] = factor->z[0] - na->state[0];
    eval->r[1] = factor->z[1] - na->state[1];
    eval->r[2] = alt_mod2pi(factor->z[2] - na->state[2]);

    memcpy(eval->W->data, factor->W->data, 3*3*sizeof(double));

    // chi^2 = r'*W*r, via X = W*r
    double X[3] = { MATD_EL(eval->W, 0, 0)*eval->r[0] +
                    MATD_EL(eval->W, 0, 1)*eval->r[1] +
                    MATD_EL(eval->W, 0, 2)*eval->r[2],
                    MATD_EL(eval->W, 1, 0)*eval->r[0] +
                    MATD_EL(eval->W, 1, 1)*eval->r[1] +
                    MATD_EL(eval->W, 1, 2)*eval->r[2],
                    MATD_EL(eval->W, 2, 0)*eval->r[0] +
                    MATD_EL(eval->W, 2, 1)*eval->r[1] +
                    MATD_EL(eval->W, 2, 2)*eval->r[2] };
    eval->chi2 = eval->r[0]*X[0] + eval->r[1]*X[1] + eval->r[2]*X[2];

    return eval;
}

static void xytpos_factor_destroy(april_graph_factor_t *factor)
{
    free(factor->nodes);
    free(factor->z);
    free(factor->ztruth);
    matd_destroy(factor->W);
    free(factor);
}

april_graph_factor_t *april_graph_factor_xytpos_create(int a, double *z, double *ztruth, matd_t *W)
{
    april_graph_factor_t *factor = calloc(1, sizeof(april_graph_factor_t));

    factor->type = APRIL_GRAPH_FACTOR_XYTPOS_TYPE;
    factor->nnodes = 1;
    factor->nodes = calloc(1, sizeof(int));
    factor->nodes[0] = a;
    factor->length = 3;

    factor->copy = xytpos_factor_copy;
    factor->eval = xytpos_factor_eval;
    factor->destroy = xytpos_factor_destroy;

    factor->z = doubles_copy(z, 3);
    factor->ztruth = doubles_copy(ztruth, 3);
    factor->W = matd_copy(W);

    return factor;
}

/////////////////////////////////////////////////////////////////////////////////////////
// XYT Node
static void xyt_node_destroy(april_graph_node_t *node)
{
    free(node->state);
    free(node->init);
    free(node->truth);
    free(node);
}

april_graph_node_t *april_graph_node_xyt_create(double *state, double *init, double *truth)
{
    april_graph_node_t *node = calloc(1, sizeof(april_graph_node_t));
    node->type = APRIL_GRAPH_NODE_XYT_TYPE;
    node->length = 3;
    node->state = doubles_copy(state, 3);
    node->init = doubles_copy(init, 3);
    node->truth = doubles_copy(truth, 3);

    node->update = xyt_node_update;
    node->copy = xyt_node_copy;
    node->destroy = xyt_node_destroy;

    return node;
}

void april_graph_factor_eval_destroy(april_graph_factor_eval_t *eval)
{
    if (!eval)
        return;

    for (int i = 0; i < eval->length; i++) {
        if (!eval->jacobians[i])
            break;

        matd_destroy(eval->jacobians[i]);
    }

    free(eval->jacobians);
    free(eval->r);
    matd_destroy(eval->W);
    free(eval);
}

/////////////////////////////////////////////////////////////////////////////////////////
// Cholesky Solver
void april_graph_cholesky_param_init(april_graph_cholesky_param_t *param)
{
    memset(param, 0, sizeof(april_graph_cholesky_param_t));

    param->ordering = NULL;
    param->max_cond = 1e16;
    param->show_timing = 0;
}

// Compute a Gauss-Newton update on the graph, using the specified
// node ordering. NULL can be passed in for parameters.
void april_graph_cholesky(april_graph_t *graph, april_graph_cholesky_param_t *_param)
{
    april_graph_cholesky_param_t param;
    april_graph_cholesky_param_init(&param);

    if (_param) {
        memcpy(&param, _param, sizeof(april_graph_cholesky_param_t));
    }

    timeprofile_t *tp = timeprofile_create();
    timeprofile_stamp(tp, "begin");

    int *ordering = NULL;

    if (param.ordering == NULL) {
        // make symbolic matrix for variable reordering.
        smatd_t *Asym = smatd_create(zarray_size(graph->nodes), zarray_size(graph->nodes));
        for (int fidx = 0; fidx < zarray_size(graph->factors); fidx++) {
            april_graph_factor_t *factor;
            zarray_get(graph->factors, fidx, &factor);

            for (int i = 0; i < factor->nnodes; i++) {
                for (int j = 0; j < factor->nnodes; j++) {
                    smatd_set(Asym, factor->nodes[i], factor->nodes[j], 1);
                }
            }
        }

        timeprofile_stamp(tp, "make symbolic");

        ordering = exact_minimum_degree_ordering(Asym);
        smatd_destroy(Asym);
    }

    // idxs[j]: what index in x do the state variables for node j start at?
    int *idxs = calloc(zarray_size(graph->nodes), sizeof(int));
    int xlen = 0;
    for (int i = 0; i < zarray_size(graph->nodes); i++) {
        april_graph_node_t *node;
        zarray_get(graph->nodes, ordering[i], &node);
        idxs[ordering[i]] = xlen;
        xlen += node->length;
    }

    if (_param == NULL || _param->ordering == NULL)
        free(param.ordering);

    timeprofile_stamp(tp, "compute ordering");

    // we'll solve normal equations, Ax = B
    smatd_t *A = smatd_create(xlen, xlen);
    double  *B = calloc(xlen, sizeof(double));

    for (int i = 0; i < zarray_size(graph->factors); i++) {
        april_graph_factor_t *factor;
        zarray_get(graph->factors, i, &factor);
        april_graph_factor_eval_t *eval = factor->eval(factor, graph, NULL);

        for (int z0 = 0; z0 < factor->nnodes; z0++) {
            int n0 = factor->nodes[z0];

            matd_t *JatW = matd_op("M'*M", eval->jacobians[z0], eval->W);

            for (int z1 = 0; z1 < factor->nnodes; z1++) {
                int n1 = factor->nodes[z1];

                matd_t *JatWJb = matd_op("M*M", JatW, eval->jacobians[z1]);

                for (int row = 0; row < JatWJb->nrows; row++) {
                    for (int col = 0; col < JatWJb->ncols; col++) {
                        double v = smatd_get(A, row+idxs[n0], col+idxs[n1]);
                        smatd_set(A, row+idxs[n0], col+idxs[n1], v + MATD_EL(JatWJb, row, col));
                    }
                }

                matd_destroy(JatWJb);
            }

            matd_t *R = matd_create_data(eval->length, 1, eval->r);
            matd_t *JatWr = matd_op("M*M", JatW, R);
            for (int row = 0; row < JatWr->nrows; row++)
                B[idxs[n0]+row] += MATD_EL(JatWr, row, 0);

            matd_destroy(R);
            matd_destroy(JatW);
            matd_destroy(JatWr);
        }

        april_graph_factor_eval_destroy(eval);
    }

    // tikhanov regularization
    // Ensure a maximum condition number of no more than maxcond.
    // trace(A) = sum of eigenvalues. worst-case scenario is that
    // we're rank 1 and that one eigenvalue is trace(A). Thus,
    // ensure all other eigenvalues are at least trace(A)/maxcond.
    if (param.max_cond > 0) {

        double trace = 0;
        for (int i = 0; i < xlen; i++) {
            double v = smatd_get(A, i, i);
            trace += v;
        }

        if (trace == 0) {
            printf("warning: trace is zero!");
            trace = 1;
        }

        double lambda = trace / param.max_cond;

        lambda = .001;

        for (int i = 0; i < xlen; i++) {
            double v = smatd_get(A, i, i);
            smatd_set(A, i, i, v + lambda);
        }
    }

    timeprofile_stamp(tp, "build A, B");

    smatd_chol_t *chol = smatd_chol(A);
    double *x = calloc(xlen, sizeof(double));
    smatd_chol_solve(chol, B, x);

    for (int i = 0; i < zarray_size(graph->nodes); i++) {
        april_graph_node_t *node;
        zarray_get(graph->nodes, i, &node);

        node->update(node, &x[idxs[i]]);
    }

    timeprofile_stamp(tp, "solve");

    smatd_chol_destroy(chol);
    smatd_destroy(A);

    free(B);
    free(x);
    free(idxs);

    if (param.show_timing)
        timeprofile_display(tp);
    timeprofile_destroy(tp);
}
