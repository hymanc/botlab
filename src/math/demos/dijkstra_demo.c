#include <stdio.h>
#include <stdlib.h>

#include "math/dijkstra.h"

int
main (int argc, char *argv[])
{
    const int n_nodes = ('f' - 'a' + 1);

    dijkstra_graph_t *graph = dijkstra_create (n_nodes, 0);

	for (int i=0; i<n_nodes; i++) {
        char *name = calloc (4, sizeof(*name));
        sprintf (name, "%c", 'a' + i);
        dijkstra_set_user (graph, i, name);
    }

#define E(a, b, c) dijkstra_add_edge_undir (graph, (a - 'a'), (b - 'a'), c)
	E('a', 'b', 4);	 E('a', 'c', 9);  E('a', 'f', 14);
	E('b', 'c', 10); E('b', 'd', 15); E('c', 'd', 11);
	E('c', 'f', 2);  E('d', 'e', 6);  E('e', 'f', 9);
#undef E

    for (int i=0; i<n_nodes; i++) {
        printf ("\nsrc=%d\n", i);
        printf ("------------------------------\n");
        dijkstra_calc_all (graph, i);

        for (int j=0; j<n_nodes; j++) {
            dijkstra_print_path (graph, j);
            printf ("\n");
        }

        printf ("\n");
        for (int j=0; j<n_nodes; j++) {
            int *path, *dist;
            int path_len = dijkstra_get_path (graph, j, &path, &dist);

            for (int k=0; k < path_len; k++) {
                if (k==0)
                    printf ("%d(%d)", path[k], dist[k]);
                else
                    printf ("-> %d(%d)", path[k], dist[k]);
            }
            printf ("\n");
        }
    }

    // clean up
    for (int i=0; i<dijkstra_n_nodes (graph); i++)
        free (dijkstra_get_user (graph, i));
    dijkstra_destroy (graph);
}
