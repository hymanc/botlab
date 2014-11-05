/* Adapted from rosetta code example
 * http://rosettacode.org/wiki/Dijkstra's_algorithm#C
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <assert.h>

#include "dijkstra.h"

#define MAX(a,b) (a > b ? a : b)

typedef struct node node_t;
typedef struct edge edge_t;

struct node {
    edge_t *edge; // singly linked list of edges
    node_t *via;  // backpointer in shortest path
    double  dist; // distance from originating node
    int     id;   // node integer id
    void   *user; // user value
    int heap_idx; //
};

struct edge {
    node_t *node;      // target of this edge
    edge_t *sibling;   // for singly linked list
    double weight;     // edge weight
};

struct dijkstra_graph {
    node_t *nodes;      // 1D array of nodes
    node_t *nodes_cc;   // clean copy for initializing queries
    int n_nodes;        // number of nodes
    bool initialized ;  /* 0 need to create a clean copy of nodes
                           1 graph structure is locked */

    edge_t *edge_root;  // pointer to edge pool
    edge_t *edge_next;  // next edge to pull from the pool
    int block_size;     // number of edges to allocate at a time
    int n_edges;        // number of edges

    int src;            // node id from which all paths originate from
    int dest;           // node id of destination, or -1 if all paths have been searched

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


dijkstra_graph_t *
dijkstra_create (int n_nodes, int block_size)
{
    dijkstra_graph_t *graph = calloc (1, sizeof(*graph));
    graph->nodes = calloc (n_nodes, sizeof(node_t));
    graph->nodes_cc = calloc (n_nodes, sizeof(node_t));
    graph->n_nodes = n_nodes;
    if (block_size > 0)
        graph->block_size = block_size; // user specified
    else if (n_nodes < 100)
        graph->block_size = n_nodes * n_nodes; // just assume a dense graph
    else
        graph->block_size = MAX (0.01*n_nodes*n_nodes, 100*100); // alloc edges in 1% increments

    for (int i=0; i<n_nodes; i++)
        graph->nodes[i].id = i;

    graph->src = -1;

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

    if (graph->nodes_cc)
        free (graph->nodes_cc);

    if (graph->heap)
        free (graph->heap);

    free (graph);
}

void
dijkstra_add_edge (dijkstra_graph_t *graph, int i, int j, double d)
{
    assert (graph);
    assert (!graph->initialized);
    assert (d > 0);

    /* Don't mind the memory management stuff, they are besides the point. Pretend
       edge_next = malloc (sizeof(edge_t)) */
    const int block_size = graph->block_size;
    if (graph->edge_next == graph->edge_root) {
        graph->edge_root = malloc ((block_size + 1) * sizeof(edge_t));
        graph->edge_root[block_size].sibling = graph->edge_next;
        graph->edge_next = graph->edge_root + block_size;
    }
    edge_t *edge = --graph->edge_next;

    node_t *a = graph->nodes + i;
    node_t *b = graph->nodes + j;

    edge->node = b;
    edge->weight = d;
    edge->sibling = a->edge;
    a->edge = edge;

    graph->n_edges++;
}

void
dijkstra_add_edge_undir (dijkstra_graph_t *graph, int i, int j, double d)
{
    assert (graph);
    dijkstra_add_edge (graph, i, j, d);
    dijkstra_add_edge (graph, j, i, d);
}

double
dijkstra_get_edge_weight (dijkstra_graph_t *graph, int i, int j)
{
    assert (graph);
    node_t *a = graph->nodes + i;
    node_t *b = graph->nodes + j;

    for (edge_t *e = a->edge; e; e = e->sibling) {
        if (e->node == b)
            return e->weight;
    }

    return -1.0;
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

void
dijkstra_calc_all (dijkstra_graph_t *graph, int src)
{
    assert (graph);

    if (!graph->initialized) {
        memcpy (graph->nodes_cc, graph->nodes, graph->n_nodes * sizeof(node_t));
        graph->initialized = 1;
    }

    if (graph->src == src && graph->dest == -1)
        return; // we've already computed this

    graph->src = src;
    graph->dest = -1;

    // reset the heap
    memset (graph->heap, 0, graph->n_nodes*sizeof (node_t *));
    graph->heap_len = 0;
    graph->heap_idx = 0;

    // --- Dijkstra stuff; unreachable nodes will never make into the queue ---
    memcpy (graph->nodes, graph->nodes_cc, graph->n_nodes * sizeof(node_t));
    node_t *start = graph->nodes + src, *lead;
	set_dist (graph, start, start, 0);
	while ((lead = pop_queue (graph))) {
		for (edge_t *e = lead->edge; e; e = e->sibling)
			set_dist (graph, e->node, lead, lead->dist + e->weight);
    }
}

void
dijkstra_calc_dest (dijkstra_graph_t *graph, int src, int dest)
{
    assert (graph);

    if (!graph->initialized) {
        memcpy (graph->nodes_cc, graph->nodes, graph->n_nodes * sizeof(node_t));
        graph->initialized = 1;
    }

    if (graph->src == src && graph->dest == dest)
        return; // we've already computed this

    graph->src = src;
    graph->dest = dest;

    // reset the heap
    memset (graph->heap, 0, graph->n_nodes*sizeof (node_t *));
    graph->heap_len = 0;
    graph->heap_idx = 0;

    // --- Dijkstra stuff; unreachable nodes will never make into the queue ---
    memcpy (graph->nodes, graph->nodes_cc, graph->n_nodes * sizeof(node_t));
    node_t *start = graph->nodes + src, *goal = graph->nodes + dest, *lead;
	set_dist (graph, start, start, 0);
	while ((lead = pop_queue (graph)) && lead != goal) {
		for (edge_t *e = lead->edge; e; e = e->sibling)
			set_dist (graph, e->node, lead, lead->dist + e->weight);
    }
}

int
dijkstra_get_path (dijkstra_graph_t *graph, int src, int dest, int **_path, double **_dist)
{
    if (!(graph->src == src && (graph->dest == dest || graph->dest == -1))) {
        fprintf (stderr, "error: [graph->src=%d, graph->dest=%d], [src=%d, dest=%d]\n",
                 graph->src, graph->dest, src, dest);
        assert (graph->src == src && (graph->dest == dest || graph->dest == -1));
        exit (EXIT_FAILURE);
    }

    int path_len = 1;
    for (node_t *node = graph->nodes + dest; node->via != node; node = node->via) {
        if (!node->via)
            return -1; // dest unreachable
        else
            path_len++;
    }

    int *path = malloc (path_len * sizeof(*path));
    double *dist = NULL;
    if (_dist)
        dist = malloc (path_len * sizeof(*dist));
    node_t *node = graph->nodes + dest;
    for (int i=path_len-1; i>=0; i--, node = node->via) {
        path[i] = node->id;
        if (_dist)
            dist[i] = node->dist;
    }

    *_path = path;
    if (_dist)
        *_dist = dist;

    return path_len;
}

void
print_path (dijkstra_graph_t *graph, int src, int dest)
{
    node_t *node = graph->nodes + dest;

	if (node->via == node)
		printf ("%d", node->id);
	else if (!node->via)
		printf ("%d(unreached)", node->id);
	else {
		print_path (graph, node->id, node->via->id);
		printf ("-> %d(%g) ", node->id, node->dist);
	}
}

void
dijkstra_print_path (dijkstra_graph_t *graph, int src, int dest)
{
    if (!(graph->src == src && (graph->dest == dest || graph->dest == -1))) {
        fprintf (stderr, "error: [graph->src=%d, graph->dest=%d], [src=%d, dest=%d]\n",
                 graph->src, graph->dest, src, dest);
        assert (graph->src == src && (graph->dest == dest || graph->dest == -1));
        exit (EXIT_FAILURE);
    }

    print_path (graph, src, dest);
}
