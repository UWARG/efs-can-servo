  #include <stdlib.h>
  #include <canard.h>
  #include "canardRxQueue.h"
  #define MAX_QUEUE_SIZE 100

  static struct CanardRxQueueItem* rxQueueHead = NULL;
  static struct CanardRxQueueItem* rxQueueTail = NULL;
  uint8_t rxQueueSize = 0;

/*
  Add a CAN frame to the queue
  Will return EMPTYITEM, MALLOCFAIL, OVERFLOW, or SUCCESS

  RX_ENQUEUE_EMPTYITEM: the recieved rx frame is empty
  RX_ENQUEUE_MALLOCFAIL: memory was not allocated for the rx frame
  RX_ENQUEUE_OVERFLOW: queue is of max length. the oldest frame is removed (rxQueueHead) and the recieved frame is enqueued
  RX_ENQUEUE_SUCCESS: rx frame was successfully enqueued with no other consequences
  
*/
  enqueueRxReturnCode enqueueRxFrame(CanardCANFrame* rx_frame) {

    // recieved frame is empty
    if (rx_frame == NULL) {
      return RX_ENQUEUE_EMPTYITEM;
    }

    bool isOverflow = false; // is true if the queue is max size

    // If the queue is full, remove the oldest frame (rxQueueHead)
    if (rxQueueSize >= MAX_QUEUE_SIZE) {
      struct CanardRxQueueItem* oldHead  = rxQueueHead;
      rxQueueHead = rxQueueHead->next;
      free(oldHead);
      isOverflow = true;
    }

    // dynamically allocate memory
    struct CanardRxQueueItem* newQueueItem = (struct CanardRxQueueItem*)malloc(sizeof(struct CanardRxQueueItem));
    
    // new queue item was not created
    if (newQueueItem == NULL) {
      return RX_ENQUEUE_MALLOCFAIL;
    }

    newQueueItem->next = NULL;
    newQueueItem->frame = *rx_frame;

    // the queue is not empty
    if (rxQueueTail) {
      rxQueueTail->next = newQueueItem;
    } else { // the queue is empty
      rxQueueHead = newQueueItem;
    }

    rxQueueTail = newQueueItem;

    if (isOverflow) {
      return RX_ENQUEUE_OVERFLOW;
    }

    rxQueueSize++;
    return RX_ENQUEUE_SUCCESS;
  }


  struct dequeueRxReturnItem dequeueRxFrame() {

    struct dequeueRxReturnItem output;

    if (!rxQueueHead) { // queue is empty, return nothing
      output.isSuccess = false;
      return output;
    }

    // queue is not empty
    struct CanardRxQueueItem* dequeuedItem = rxQueueHead;
    output.isSuccess = true;
    output.frame = dequeuedItem->frame;
    rxQueueHead = rxQueueHead->next;

    if (!rxQueueHead) {
      rxQueueTail = NULL; // after dequeuing, the queue is empty
    }

    rxQueueSize--;
    free(dequeuedItem);

    return output;
  }

