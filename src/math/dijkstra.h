#ifndef __MATH_DIJKSTRA_H__
#define __MATH_DIJKSTRA_H__

#ifdef __cplusplus
extern "C" {
#endif

typedef struct dijkstra_graph dijkstra_graph_t;

/* initializes a graph capable of holding n nodes
   block_size is used for memory efficiency in allocating edges and ideally should match the number of expected edges
   block_size <= 0, automatically attempts to compute a good edge queue size
 */
dijkstra_graph_t *
dijkstra_create (int n_nodes, int block_size);

void
dijkstra_destroy (dijkstra_graph_t *graph);

/* adds a directed edge from node i to node j with edge weight of d */
void
dijkstra_add_edge (dijkstra_graph_t *graph, int i, int j, double d);

/* adds an undirected edge between node i and node j with edge weight of d */
void
dijkstra_add_edge_undir (dijkstra_graph_t *graph, int i, int j, double d);

/* assign a user allocated pointer to node i */
void
dijkstra_set_user (dijkstra_graph_t *graph, int i, void *user);

/* retrieve user allocated pointer from node i */
void *
dijkstra_get_user (dijkstra_graph_t *graph, int i);

/* returns number of nodes in the graph */
int
dijkstra_n_nodes (dijkstra_graph_t *graph);

/* returns number of edges in the graph
   NOTE: undirected edges count as 2 */
int
dijkstra_n_edges (dijkstra_graph_t *graph);

/* does a Dijkstra shortest path search over the graph emanating from the src to all destination nodes */
void
dijkstra_calc_all (dijkstra_graph_t *graph, int src);

/* does a Dijkstra shortest path search from src to dest nodes */
void
dijkstra_calc_dest (dijkstra_graph_t *graph, int src, int dest);

/* returns the path from src to dest */
int
dijkstra_get_path (dijkstra_graph_t *graph, int src, int dest, int **path, double **dist);

/* prints the path from src to dest */
void
dijkstra_print_path (dijkstra_graph_t *graph, int src, int dest);

#ifdef __cplusplus
}
#endif

#endif // __MATH_DIJKSTRA_H__
