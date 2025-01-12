#ifndef CANARD_RX_QUEUE_H
#define CANARD_RX_QUEUE_H

#include <canard.h>

struct CanardRxQueueItem {
    struct CanardRxQueueItem* next;
    CanardCANFrame frame;
};

extern struct CanardRxQueueItem* rxQueueHead;
extern struct CanardRxQueueItem* rxQueueTail;
extern uint8_t rxQueueSize;

typedef enum {
  RX_ENQUEUE_SUCCESS,
  RX_ENQUEUE_EMPTYITEM,
  RX_ENQUEUE_MALLOCFAIL,
  RX_ENQUEUE_OVERFLOW
} enqueueRxReturnCode;

struct dequeueRxReturnItem {
    bool isSuccess;
    CanardCANFrame frame;
};

enqueueRxReturnCode enqueueRxFrame(CanardCANFrame* rx_frame);
struct dequeueRxReturnItem dequeueRxFrame();

#endif