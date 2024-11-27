#include <stdio.h>
#include <stdlib.h>
#include <canard.h>
#include "canardRxQueue.h"

struct CanardRxQueueItem* rxQueueHead = NULL;
struct CanardRxQueueItem* rxQueueTail = NULL;

void enqueueRxFrame(CanardCANFrame* rx_frame) {

  // dynamically allocate memory
  struct CanardRxQueueItem* new = (struct CanardRxQueueItem*)malloc(sizeof(struct CanardRxQueueItem));
  
  if (new == NULL) {
    printf("CanardRxQueueItem memory allocation failed");
    return;
  }

  new->next = NULL;
  new->frame = *rx_frame;

  if (rxQueueTail) {
    rxQueueTail->next = new;
  } else {
    rxQueueHead = new;
  }

  rxQueueTail = new;
}


struct CanardRxQueueItem* dequeueRxFrame() {

  if (!rxQueueHead) {
    return NULL; // queue is empty
  }

  struct CanardRxQueueItem* dequeuedItem  = rxQueueHead;
  rxQueueHead = rxQueueHead->next;

  if (!rxQueueHead) {
    rxQueueTail = NULL; // after dequeuing, the queue is empty
  }

  return dequeuedItem;
}

