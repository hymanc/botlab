#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <assert.h>

#define MAX(a,b) (a > b ? a : b)

typedef struct node node_t;
typedef struct edge edge_t;

struct node {
    edge_t *edge; // singly linked list of edges
    node_t *via;  // where previous node is in shortest path
    double  dist; // distance from originating node
    int     id;   // node integer id
    void   *user; // user value
    int heap_idx; //
};

struct edge {
    node_t *node;      // target of this edge
    edge_t *sibling;   // for singly linked list
    int cost;          // edge cost
};

typedef struct dijkstra_graph dijkstra_graph_t;
struct dijkstra_graph {
    node_t *nodes;      // 1D array of nodes
    node_t *nodes_ORIG; // clean copy for initializing queries
    int n_nodes;        // number of nodes

    edge_t *edge_root;  // pointer to queue of edges
    edge_t *edge_next;  // next edge on the queue
    int block_size;     // number of edges to allocate at a time
    int n_edges;        // number of edges

    bool initialized;

    // --- priority queue stuff ---
    node_t **heap;
    int heap_len;
    int heap_idx;
};

void
set_dist (dijkstra_graph_t *graph, node_t *node, node_t *via, double d)
{
    assert (graph);

	// already know better path?
	if (node->via && node->dist <= d)
        return;

	// find existing heap entry, or create a new one
	node->dist = d;
	node->via = via;

	int i = node->heap_idx;
	if (!i)
        i = ++graph->heap_len;

	// upheap
	for (int j; i > 1 && node->dist < graph->heap[j = i/2]->dist; i = j)
		(graph->heap[i] = graph->heap[j])->heap_idx = i;

	graph->heap[i] = node;
	node->heap_idx = i;
}

node_t *
pop_queue (dijkstra_graph_t *graph)
{
    assert (graph);

	if (!graph->heap_len)
        return 0;

	// remove leading element, pull tail element there and downheap
	node_t *node = graph->heap[1];
	node_t *tmp = graph->heap[graph->heap_len--];

    int i, j;
	for (i = 1; i < graph->heap_len && (j = i * 2) <= graph->heap_len; i = j) {
		if (j < graph->heap_len && graph->heap[j]->dist > graph->heap[j+1]->dist)
            j++;

		if (graph->heap[j]->dist >= tmp->dist)
            break;

		(graph->heap[i] = graph->heap[j])->heap_idx = i;
	}

	graph->heap[i] = tmp;
	tmp->heap_idx = i;

	return node;
}

// --- Dijkstra stuff; unreachable nodes will never make into the queue ---
void
dijkstra_calc_all (dijkstra_graph_t *graph, int src)
{
    assert (graph);

    if (!graph->initialized) {
        memcpy (graph->nodes_ORIG, graph->nodes, graph->n_nodes * sizeof(node_t));
        graph->initialized = 1;
    }
    else
        memcpy (graph->nodes, graph->nodes_ORIG, graph->n_nodes * sizeof(node_t));

    node_t *start = graph->nodes + src, *lead;
	set_dist (graph, start, start, 0);
	while (lead = pop_queue (graph)) {
		for (edge_t *e = lead->edge; e; e = e->sibling)
			set_dist (graph, e->node, lead, lead->dist + e->cost);
    }
}

void
dijkstra_print_path (dijkstra_graph_t *graph, int dest)
{
    node_t *node = graph->nodes + dest;

	if (node->via == node)
		printf ("%d", node->id);
	else if (!node->via)
		printf ("%d(unreached)", node->id);
	else {
		dijkstra_print_path (graph, node->via->id);
		printf ("-> %d(%g) ", node->id, node->dist);
	}
}


dijkstra_graph_t *
dijkstra_create (int n_nodes, int block_size)
{
    dijkstra_graph_t *graph = calloc (1, sizeof(*graph));
    graph->nodes = calloc (n_nodes, sizeof(node_t));
    graph->nodes_ORIG = calloc (n_nodes, sizeof(node_t));
    graph->n_nodes = n_nodes;
    if (block_size > 0)
        graph->block_size = block_size; // user specified
    else if (n_nodes < 100)
        graph->block_size = n_nodes * n_nodes; // just assume a dense graph
    else
        graph->block_size = MAX (0.01*n_nodes*n_nodes, 100*100); // alloc edges in 1% increments

    for (int i=0; i<n_nodes; i++)
        graph->nodes[i].id = i;

    graph->heap = calloc (n_nodes + 1, sizeof(node_t *));
    return graph;
}

void
dijkstra_destroy (dijkstra_graph_t *graph)
{
    if (!graph)
        return;

    // free edges
    for (; graph->edge_root; graph->edge_root = graph->edge_next) {
        graph->edge_next = graph->edge_root[graph->block_size].sibling;
        free (graph->edge_root);
    }

    if (graph->nodes)
        free (graph->nodes);

    if (graph->nodes_ORIG)
        free (graph->nodes_ORIG);

    if (graph->heap)
        free (graph->heap);

    free (graph);
}

void
dijkstra_add_edge (dijkstra_graph_t *graph, int i, int j, double d)
{
    assert (graph);
    const int block_size = graph->block_size;

    /* Don't mind the memory management stuff, they are besides the point. Pretend
       edge_next = malloc (sizeof(edge_t)) */
    if (graph->edge_next == graph->edge_root) {
        graph->edge_root = malloc ((block_size + 1) * sizeof(edge_t));
        graph->edge_root[block_size].sibling = graph->edge_next;
        graph->edge_next = graph->edge_root + block_size;
    }
    edge_t *edge = --graph->edge_next;

    node_t *a = graph->nodes + i;
    node_t *b = graph->nodes + j;

    edge->node = b;
    edge->cost = d;
    edge->sibling = a->edge;
    a->edge = edge;
}

void
dijkstra_add_edge_undir (dijkstra_graph_t *graph, int i, int j, double d)
{
    assert (graph);
    dijkstra_add_edge (graph, i, j, d);
    dijkstra_add_edge (graph, j, i, d);
}

void
dijkstra_set_user (dijkstra_graph_t *graph, int i, void *user)
{
    assert (graph);
    assert (i < graph->n_nodes);
    graph->nodes[i].user = user;
}

void *
dijkstra_get_user (dijkstra_graph_t *graph, int i)
{
    assert (graph);
    assert (i < graph->n_nodes);
    return graph->nodes[i].user;
}

int
dijkstra_n_nodes (dijkstra_graph_t *graph)
{
    assert (graph);
    return graph->n_nodes;
}

int
dijkstra_n_edges (dijkstra_graph_t *graph)
{
    assert (graph);
    return graph->n_edges;
}

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

#	define E(a, b, c) dijkstra_add_edge_undir (graph, (a - 'a'), (b - 'a'), c)
	E('a', 'b', 4);	E('a', 'c', 9); E('a', 'f', 14);
	E('b', 'c', 10);E('b', 'd', 15);E('c', 'd', 11);
	E('c', 'f', 2); E('d', 'e', 6);	E('e', 'f', 9);
#	undef E

    for (int i=0; i<n_nodes; i++) {
        printf ("\nnode i=%d user=%s\n", i, (char *) dijkstra_get_user (graph, i));
        dijkstra_calc_all (graph, i);
        for (int j=0; j<n_nodes; j++) {
            dijkstra_print_path (graph, j);
            printf ("\n");
        }
    }

    // clean up
    for (int i=0; i<dijkstra_n_nodes (graph); i++)
        free (dijkstra_get_user (graph, i));
    dijkstra_destroy (graph);
}
