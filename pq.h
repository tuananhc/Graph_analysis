/*
pq.h

Visible structs and functions for priority queues.

Skeleton written by Grady Fitzpatrick for COMP20007 Assignment 1 2021
*/
/* The priority queue. */
struct pq;

/* Get a new empty priority queue. */
struct pq *newPQ();

/* Add an item to the priority queue - cast pointer to (void *). */
void enqueue(struct pq *pq, int item, int priority);

/* Take the smallest item from the priority queue - cast pointer back to
  original type. */
int deletemin(struct pq *pq);

/* Returns 1 if empty, 0 otherwise. */
int empty(struct pq *pq);

/* Remove all items from priority queue (doesn't free) and free the queue. */
void freePQ(struct pq *pq);

void updateQueue(struct pq *pq, int item, int newPriority);

int get_queue_size(struct pq *pq);