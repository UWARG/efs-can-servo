#ifndef CANARD_RX_QUEUE_H
#define CANARD_RX_QUEUE_H

#include <stdlib.h>
#include <canard.h>

struct CanardRxQueueItem {
    struct CanardRxQueueItem* next;
    CanardCANFrame frame;
};

extern struct CanardRxQueueItem* rxQueueHead;
extern struct CanardRxQueueItem* rxQueueTail;

void enqueueRxFrame(CanardCANFrame* rx_frame);
struct CanardRxQueueItem* dequeueRxFrame();

#endif