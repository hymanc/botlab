#ifndef __MATH_APRIL_GRAPH_H__
#define __MATH_APRIL_GRAPH_H__

#include <stdlib.h>

#include "common/zarray.h"
#include "common/zhash.h"

#include "matd.h"

typedef struct april_graph april_graph_t;
struct april_graph
{
    zarray_t *factors;
    zarray_t *nodes;

    zhash_t *attr;  // string (char*) to arbitrary pointer
};

typedef struct april_graph_factor_eval april_graph_factor_eval_t;
struct april_graph_factor_eval
{
    double chi2;

    // One jacobian for every node that this factor is connected to.
    // (order given by the factor's 'nodes' array). Should be NULL
    // terminated so that this structure can be deallocated without knowing
    // which factor created it.
    matd_t **jacobians;

    int length;
    double *r; // residual (length x 1)
    matd_t *W; // information matrix of observation (length x length)
};

#define APRIL_GRAPH_FACTOR_XYT_TYPE 1
#define APRIL_GRAPH_FACTOR_XYTPOS_TYPE 2

#define APRIL_GRAPH_NODE_XYT_TYPE 100

typedef struct april_graph_factor april_graph_factor_t;
struct april_graph_factor
{
    int type; // a unique identifier

    int nnodes;
    int *nodes;

    int length; // how many DOF?

    zhash_t *attr;  // string (char*) to arbitrary pointer

    april_graph_factor_t* (*copy)(april_graph_factor_t *factor);

    // evaluate this factor potential given the current graph. The
    // "eval" object can be expensive to build; for efficiency, a
    // caller can recycle an eval object previously constructed by
    // this object. In this case, the object will return the passed-in
    // "eval" object (with recomputed numerical values). Otherwise, a
    // new "eval" object is created and returned.
    april_graph_factor_eval_t* (*eval)(april_graph_factor_t *factor, april_graph_t *graph, april_graph_factor_eval_t *eval);

    void (*destroy)(april_graph_factor_t *factor);

    // storage for the implementation. They can use these pointers or
    // not.
    double *z;
    double *ztruth;
    matd_t *W;

    void *impl;
};

typedef struct april_graph_node april_graph_node_t;
struct april_graph_node
{
    int type;  // a unique identifier
    int length; // # of DOF

    double *state; // length x 1  vectors
    double *init;
    double *truth;

    zhash_t *attr;  // string (char*) to arbitrary pointer

    april_graph_node_t* (*copy)(april_graph_node_t *node);

    // called after some optimization; generically, should
    // implement state += dstate.
    void (*update)(april_graph_node_t *node, double *dstate);

    void (*destroy)(april_graph_node_t *node);

    void *impl;
};

april_graph_t *april_graph_create();
april_graph_t *april_graph_create_from_file(const char *path);
void april_graph_destroy(april_graph_t *graph);

void april_graph_factor_eval_destroy(april_graph_factor_eval_t *eval);

typedef struct april_graph_cholesky_param april_graph_cholesky_param_t;
struct april_graph_cholesky_param
{
    // if non-zero, apply tikhanov regularization to improve the
    // condition number of the matrix. (The default behavior is
    // max_cond = 100000.
    double max_cond;

    // Use the specified node ordering to reduce fill-in. If not
    // specified, an ordering is computed automatically.
    int *ordering;

    int show_timing;
};

// initialize to default values.
void april_graph_cholesky_param_init(april_graph_cholesky_param_t *param);

// Compute a Gauss-Newton update on the graph, using the specified
// node ordering. The ordering should specify the order for each
// april_graph_node_t; if NULL, a default ordering is computed. The
// ordering passed in belongs to the caller.
void april_graph_cholesky(april_graph_t *graph, april_graph_cholesky_param_t *param);

int april_graph_dof(april_graph_t *graph);
double april_graph_chi2(april_graph_t *graph);

void april_graph_postscript(april_graph_t *graph, const char *path);

april_graph_factor_t *april_graph_factor_xyt_create(int a, int b, double *z, double *ztruth, matd_t *W);
april_graph_node_t *april_graph_node_xyt_create(double *state, double *init, double *truth);

april_graph_factor_t *april_graph_factor_xytpos_create(int a, double *z, double *ztruth, matd_t *W);

#endif // __MATH_APRIL_GRAPH_H__
