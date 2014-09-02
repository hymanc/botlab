#include <stdio.h>

#include "smatd.h"
#include "april_graph.h"
#include "timeprofile.h"

int *exact_minimum_degree_ordering(smatd_t *mat);

int main(int argc, char *argv[])
{
    if (0) {
        april_graph_t *graph = april_graph_create();

        for (int i = 0; i < 3; i++) {
            april_graph_node_t *node = april_graph_node_xyt_create((double[]) { 0, 0, 0}, NULL, NULL);
            zarray_add(graph->nodes, &node);
        }

        if (1) {
            april_graph_factor_t *factor = april_graph_factor_xyt_create(0, 1,
                                                                         (double[]) { 1, 0, 0},
                                                                         NULL,
                                                                         matd_create_data(3, 3, (double[]) {
                                                                                 1, 0, 0,
                                                                                     0, 1, 0,
                                                                                     0, 0, 1 }));
            zarray_add(graph->factors, &factor);
        }

        if (1) {
            april_graph_factor_t *factor = april_graph_factor_xyt_create(1, 2,
                                                                         (double[]) { 1, 0, 0},
                                                                         NULL,
                                                                         matd_create_data(3, 3, (double[]) {
                                                                                 1, 0, 0,
                                                                                     0, 1, 0,
                                                                                     0, 0, 1 }));
            zarray_add(graph->factors, &factor);
        }

        if (1) {
            april_graph_factor_t *factor = april_graph_factor_xyt_create(0, 2,
                                                                         (double[]) { 3, 0, 0},
                                                                         NULL,
                                                                         matd_create_data(3, 3, (double[]) {
                                                                                 100, 0, 0,
                                                                                     0, 100, 0,
                                                                                     0, 0, 100 }));
            zarray_add(graph->factors, &factor);
        }

        april_graph_cholesky(graph, NULL);
    }

    if (1) {
//        april_graph_t *g = april_graph_create_from_file("/tmp/out.graph");
        april_graph_t *g = april_graph_create_from_file("/home/ebolson/csw0.graph");

        april_graph_factor_t *factor = april_graph_factor_xytpos_create(0,
                                                                        (double[]) { 0, 0, 0 },
                                                                        NULL,
                                                                        matd_identity(3));
        zarray_add(g->factors, &factor);

        april_graph_cholesky_param_t param;
        april_graph_cholesky_param_init(&param);
        param.max_cond = 0;
        param.show_timing = 1;

        for (int i = 0; i < 10; i++) {
            april_graph_postscript(g, "/tmp/b.ps");

            double chi2 = april_graph_chi2(g);
            int dof = april_graph_dof(g);
            printf("chi2: %15f (%15f)\n", chi2, chi2 / dof);

            april_graph_cholesky(g, &param);

        }

        april_graph_destroy(g);
    }

    return 0;
}
