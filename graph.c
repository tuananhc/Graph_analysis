/*
graph.c

Set of vertices and edges implementation.

Implementations for helper functions for graph construction and manipulation.

Skeleton written by Grady Fitzpatrick for COMP20007 Assignment 1 2021
*/
#include <stdlib.h>
#include <assert.h>
#include <limits.h>
#include <limits.h>
#include <stdbool.h>

#include "graph.h"
#include "utils.h"
#include "list.h"
#include "pq.h"

#define INITIALEDGES 32

struct edge;

/* Definition of a graph. */
struct graph
{
  int numVertices;
  int numEdges;
  int allocedEdges;
  struct edge **edgeList;
};

/* Definition of an edge. */
struct edge
{
  int start;
  int end;
};

struct graph *newGraph(int numVertices)
{
  struct graph *g = (struct graph *)malloc(sizeof(struct graph));
  assert(g);
  /* Initialise edges. */
  g->numVertices = numVertices;
  g->numEdges = 0;
  g->allocedEdges = 0;
  g->edgeList = NULL;
  return g;
}

/* Adds an edge to the given graph. */
void addEdge(struct graph *g, int start, int end)
{
  assert(g);
  struct edge *newEdge = NULL;
  /* Check we have enough space for the new edge. */
  if ((g->numEdges + 1) > g->allocedEdges)
  {
    if (g->allocedEdges == 0)
    {
      g->allocedEdges = INITIALEDGES;
    }
    else
    {
      (g->allocedEdges) *= 2;
    }
    g->edgeList = (struct edge **)realloc(g->edgeList,
                                          sizeof(struct edge *) * g->allocedEdges);
    assert(g->edgeList);
  }

  /* Create the edge */
  newEdge = (struct edge *)malloc(sizeof(struct edge));
  assert(newEdge);
  newEdge->start = start;
  newEdge->end = end;

  /* Add the edge to the list of edges. */
  g->edgeList[g->numEdges] = newEdge;
  (g->numEdges)++;
}

/* Frees all memory used by graph. */
void freeGraph(struct graph *g)
{
  int i;
  for (i = 0; i < g->numEdges; i++)
  {
    free((g->edgeList)[i]);
  }
  if (g->edgeList)
    free(g->edgeList);
  free(g);
}

void get_neighbors(struct graph *graph, int *neighbors, int u)
{
  int count = 0, i;
  for (i = 0; i < graph->numEdges; ++i)
  {
    if (graph->edgeList[i]->start == u)
    {
      neighbors[count] = graph->edgeList[i]->end;
      count++;
    }
    else if (graph->edgeList[i]->end == u)
    {
      neighbors[count] = graph->edgeList[i]->start;
      count++;
    }
  }
}

int get_degree(struct graph *graph, int u)
{
  int degree = 0, i;
  for (i = 0; i < graph->numEdges; ++i)
  {
    if (graph->edgeList[i]->start == u || graph->edgeList[i]->end == u)
      degree++;
  }
  return degree;
}

int *solve(struct graph *g, int *subnet_count, int *largest_subnet_size)
{
  bool *visited = calloc(g->numVertices, sizeof(bool));
  assert(visited);
  struct pq **pq = malloc(sizeof(struct pq *) * g->numVertices);
  int u;
  for (u = 0; u < g->numVertices; u++)
  {
    if (!visited[u])
    {
      (*subnet_count)++;
      pq[*subnet_count] = newPQ();
      enqueue(pq[*subnet_count], u, u);
      explore(g, u, visited, pq[*subnet_count]);
    }
  }
  (*subnet_count)++;
  int cur, max_index, max = 0, i;
  for (i = 0; i < *subnet_count; i++)
  {
    cur = get_queue_size(pq[i]);
    if (cur > max)
    {
      max = cur;
      max_index = i;
    }
  }
  *largest_subnet_size = max;
  int *largest_subnet = malloc(sizeof(int) * max);
  for (i = 0; i < max; i++)
  {
    largest_subnet[i] = deletemin(pq[max_index]);
  }
  for (i = 0; i < *subnet_count; i++)
  {
    freePQ(pq[i]);
  }
  free(pq);
  free(visited);
  return largest_subnet;
}

void explore(struct graph *graph, int u, bool *visited, struct pq *queue)
{
  int i;
  visited[u] = true;
  int degree = get_degree(graph, u);
  int *neighbors = malloc(sizeof(int) * degree);
  get_neighbors(graph, neighbors, u);
  for (i = 0; i < degree; i++)
  {
    if (!visited[neighbors[i]])
    {
      enqueue(queue, neighbors[i], neighbors[i]);
      explore(graph, neighbors[i], visited, queue);
    }
  }
  free(neighbors);
}

// create a new graph after the outage occurs
struct graph *create_new_graph(struct graph *g, int *outages, int num_outages)
{
  // keep the same number of vertices but omit the edges with the servers that
  // were damaged
  int i, j;
  struct graph *new_graph = newGraph(g->numVertices);
  for (i = 0; i < g->numEdges; i++)
  {
    bool pass = true;
    for (j = 0; j < num_outages; j++)
    {
      if (g->edgeList[i]->start == outages[j] || g->edgeList[i]->end == outages[j])
        pass = false;
    }
    if (pass)
      addEdge(new_graph, g->edgeList[i]->start, g->edgeList[i]->end);
  }
  return new_graph;
}

void dijkstra(struct graph *g, int *distance, int *prev, int u)
{
  int i;
  struct pq *queue = newPQ();
  bool *visited = calloc(g->numVertices, sizeof(bool));
  for (i = 0; i < g->numVertices; i++)
  {
    distance[i] = -1;
    prev[i] = -1;
  }
  distance[u] = prev[u] = 0;
  enqueue(queue, u, distance[u]);
  while (!empty(queue))
  {
    int min_node = deletemin(queue);
    visited[min_node] = true;
    int degree = get_degree(g, min_node);
    int *neighbors = malloc(sizeof(int) * degree);
    get_neighbors(g, neighbors, min_node);
    for (i = 0; i < degree; i++)
    {
      int neighbor = neighbors[i];
      if (!visited[neighbor])
      {
        int new_cost = distance[min_node] + 1;
        if (distance[neighbor] == -1)
        {
          distance[neighbor] = new_cost;
          enqueue(queue, neighbor, new_cost);
        }
        else if (distance[neighbor] > new_cost)
        {
          distance[neighbor] = new_cost;
          updateQueue(queue, neighbor, new_cost);
        }
        if (prev[neighbor] > min_node || prev[neighbor] == -1)
          prev[neighbor] = min_node;
      }
    }
    free(neighbors);
  }
  free(queue);
  free(visited);
  return;
}

int *task4_solve(struct graph *g, int *radius, int *outages, int num_outages)
{
  int i, j;
  struct graph *new_graph = create_new_graph(g, outages, num_outages);
  assert(new_graph);
  int **distances = malloc(sizeof(int *) * g->numVertices);
  assert(distances);
  int **prevs = malloc(sizeof(int *) * g->numVertices);
  assert(prevs);
  for (i = 0; i < g->numVertices; i++)
  {
    distances[i] = malloc(sizeof(int) * g->numVertices);
    assert(distances[i]);
    prevs[i] = malloc(sizeof(int) * g->numVertices);
    assert(prevs[i]);
    dijkstra(new_graph, distances[i], prevs[i], i);
  }
  int max = 0;
  int start = g->numVertices, end = g->numVertices;
  for (i = 0; i < g->numVertices; i++)
  {
    for (j = 0; j < g->numVertices; j++)
    {
      if (distances[i][j] > max ||
          (distances[i][j] == max &&
           (i < start || (i == start && j < end))))
      {
        max = distances[i][j];
        start = i;
        end = j;
      }
    }
  }
  *radius = max;
  int *result = malloc(sizeof(int) * (max + 1));
  assert(result);
  int index = max;
  int num = end;
  while (index >= 0)
  {
    result[index] = num;
    num = prevs[start][num];
    index--;
  }
  for (i = 0; i < g->numVertices; i++)
  {
    free(distances[i]);
    free(prevs[i]);
  }
  free(distances);
  free(prevs);
  free(new_graph);
  return result;
}

int *get_critical_nodes(struct graph *g, int *num)
{
  int i;
  bool *visited = calloc(g->numVertices, sizeof(bool));
  assert(visited);
  int *ancestors = malloc(sizeof(int) * g->numVertices);
  assert(ancestors);
  int *order = malloc(sizeof(int) * g->numVertices);
  assert(order);
  int *hras = malloc(sizeof(int) * g->numVertices);
  assert(hras);
  bool *criticals = calloc(g->numVertices, sizeof(bool));
  assert(criticals);
  int *result = malloc(sizeof(int) * g->numVertices);
  assert(result);

  for (i = 0; i < g->numVertices; i++)
  {
    ancestors[i] = -1;
    order[i] = 0;
    hras[i] = i;
  }

  int nth_order;

  for (i = 0; i < g->numVertices; i++)
  {
    if (!visited[i])
      helper(g, visited, ancestors, order, hras, criticals, &nth_order, i);
  }
  int count = 0;
  for (i = 0; i < g->numVertices; i++)
  {
    if (criticals[i])
    {
      result[count] = i;
      count++;
    }
  }
  *num = count;
  free(criticals);
  free(hras);
  free(visited);
  free(order);
  return result;
}

void helper(struct graph *g, bool *visited, int *ancestors, int *order, int *hras, bool *criticals, int *nth_order, int u)
{
  int i, descendants = 0;
  visited[u] = true;
  order[u] = *nth_order;
  hras[u] = *nth_order;
  (*nth_order)++;
  int degree = get_degree(g, u);
  int *neighbors = malloc(sizeof(int) * degree);
  assert(neighbors);
  get_neighbors(g, neighbors, u);
  for (i = 0; i < degree; i++)
  {
    if (!visited[neighbors[i]])
    {
      descendants++;
      ancestors[neighbors[i]] = u;
      helper(g, visited, ancestors, order, hras, criticals, nth_order, neighbors[i]);
      hras[u] = min(hras[u], hras[neighbors[i]]);
      if (ancestors[u] == -1 && descendants >= 2)
        criticals[u] = true;
      if (ancestors[u] != -1 && hras[neighbors[i]] >= order[u])
        criticals[u] = true;
    }
    else if (ancestors[u] != neighbors[i])
      hras[u] = min(hras[u], order[neighbors[i]]);
  }
  free(neighbors);
}

int min(int a, int b)
{
  if (a <= b)
    return a;
  return b;
}

/* Finds:
  - Number of connected subnetworks (before outage) (Task 2)
  - Number of servers in largest subnetwork (before outage) (Task 3)
  - SIDs of servers in largest subnetwork (before outage) (Task 3)
  - Diameter of largest subnetworks (after outage) (Task 4)
  - Number of servers in path with largest diameter - should be one more than
    Diameter if a path exists (after outage) (Task 4)
  - SIDs in path with largest diameter (after outage) (Task 4)
  - Number of critical servers (before outage) (Task 7)
  - SIDs of critical servers (before outage) (Task 7)
*/

struct solution *graphSolve(struct graph *g, enum problemPart part,
                            int numServers, int numOutages, int *outages)
{
  struct solution *solution = (struct solution *)
      malloc(sizeof(struct solution));
  assert(solution);
  /* Initialise solution values */
  initaliseSolution(solution);
  int subnet_count = -1;
  int largest_subnet_count;
  int *largest_subnet = solve(g, &subnet_count, &largest_subnet_count);
  if (part == TASK_2)
  {
    /* IMPLEMENT TASK 2 SOLUTION HERE */
    solution->connectedSubnets = subnet_count;
  }
  else if (part == TASK_3)
  {
    /* IMPLEMENT TASK 3 SOLUTION HERE */
    solution->largestSubnet = largest_subnet_count;
    solution->largestSubnetSIDs = largest_subnet;
  }
  else if (part == TASK_4)
  {
    /* IMPLEMENT TASK 4 SOLUTION HERE */
    int radius;
    int *results = task4_solve(g, &radius, outages, numOutages);
    solution->postOutageDiameter = radius;
    solution->postOutageDiameterCount = radius + 1;
    solution->postOutageDiameterSIDs = results;
  }
  else if (part == TASK_7)
  {
    /* IMPLEMENT TASK 7 SOLUTION HERE */
    int count;
    int *result = get_critical_nodes(g, &count);

    solution->criticalServerCount = count;
    solution->criticalServerSIDs = result;
  }
  return solution;
}