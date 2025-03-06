/*
 * canard_stm32_driver.c
 *
 *  Created on: 
 *      Author: 
 */

#include "canard.h"
#include "stm32l4xx_hal.h"
#include <string.h>
#include <stdio.h>
#include "canard_stm32_driver.h"
#include "node_settings.h"
#include "canardRxQueue.h"

uint32_t canMailbox; //CAN Bus Mail box variable

void init(InitTypeDef *initParams) {
  HAL_CAN_Start(initParams->hcan);
  HAL_CAN_ActivateNotification(initParams->hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
  canardInit(initParams->canard,
                initParams->mem_pool,
                initParams->mem_pool_size,
                initParams->on_recep,
                initParams->should_accept,
                NULL);

  if (NODE_ID > 0) {
      canardSetLocalNodeID(initParams->canard, NODE_ID);
  } else {
      printf("Node ID is 0, this node is anonymous and can't transmit most messaged. Please update this in node_settings.h\n");
  }
}

/**
 * @brief  Process CAN message from RxLocation FIFO into rx_frame
 * @param  hcan pointer to an CAN_HandleTypeDef structure that contains
 *         the configuration information for the specified FDCAN.
 * @param  RxLocation Location of the received message to be read.
 *         This parameter can be a value of @arg FDCAN_Rx_location.
 * @param  rx_frame pointer to a CanardCANFrame structure where the received CAN message will be
 * 		stored.
 * @retval ret == 1: OK, ret < 0: CANARD_ERROR, ret == 0: Check hcan->ErrorCode
 */
int16_t canardSTM32Recieve(CAN_HandleTypeDef *hcan, uint32_t RxLocation, CanardCANFrame *const rx_frame) {
	if (rx_frame == NULL) {
		return -CANARD_ERROR_INVALID_ARGUMENT;
	}

	CAN_RxHeaderTypeDef RxHeader;
	uint8_t RxData[8];

	if (HAL_CAN_GetRxMessage(hcan, RxLocation, &RxHeader, RxData) == HAL_OK) {

		//	printf("Received message: ID=%lu, DLC=%lu\n", RxHeader.StdId, RxHeader.DLC);
		//
		//	printf("0x");
		//	for (int i = 0; i < RxHeader.DLC; i++) {
		//		printf("%02x", RxData[i]);
		//	}
		//	printf("\n");

		// Process ID to canard format
		rx_frame->id = RxHeader.ExtId;

		if (RxHeader.IDE == CAN_ID_EXT) { // canard will only process the message if it is extended ID
			rx_frame->id |= CANARD_CAN_FRAME_EFF;
		}

		if (RxHeader.RTR == CAN_RTR_REMOTE) { // canard won't process the message if it is a remote frame
			rx_frame->id |= CANARD_CAN_FRAME_RTR;
		}

		rx_frame->data_len = RxHeader.DLC;
		memcpy(rx_frame->data, RxData, RxHeader.DLC);

		// assume a single interface
		rx_frame->iface_id = 0;

		return 1;
	}

	// Either no CAN msg to be read, or an error that can be read from hcan->ErrorCode
	return 0;
}

/**
 * @brief  Process tx_frame CAN message into Tx FIFO/Queue and transmit it
 * @param  hcan pointer to an CAN_HandleTypeDef structure that contains
 *         the configuration information for the specified FDCAN.
 * @param  tx_frame pointer to a CanardCANFrame structure that contains the CAN message to
 * 		transmit.
 * @retval ret == 1: OK, ret < 0: CANARD_ERROR, ret == 0: Check hcan->ErrorCode
 */
int16_t canardSTM32Transmit(CAN_HandleTypeDef *hcan, const CanardCANFrame* const tx_frame) {
	if (tx_frame == NULL) {
		return -CANARD_ERROR_INVALID_ARGUMENT;
	}

	if (tx_frame->id & CANARD_CAN_FRAME_ERR) {
		return -CANARD_ERROR_INVALID_ARGUMENT; // unsupported frame format
	}

	CAN_TxHeaderTypeDef TxHeader;
	uint8_t TxData[8];

	// Process canard id to STM FDCAN header format
	if (tx_frame->id & CANARD_CAN_FRAME_EFF) {
		TxHeader.IDE = CAN_ID_EXT;
		TxHeader.ExtId = tx_frame->id & CANARD_CAN_EXT_ID_MASK;
	} else {
		TxHeader.IDE = CAN_ID_STD;
		TxHeader.StdId = tx_frame->id & CANARD_CAN_STD_ID_MASK;
	}

	TxHeader.DLC = tx_frame->data_len;

	if (tx_frame->id & CANARD_CAN_FRAME_RTR) {
		TxHeader.RTR = CAN_RTR_REMOTE;
	} else {
		TxHeader.RTR = CAN_RTR_DATA;
	}

	TxHeader.TransmitGlobalTime = DISABLE;
	memcpy(TxData, tx_frame->data, TxHeader.DLC);

	if (HAL_CAN_AddTxMessage(hcan, &TxHeader, TxData, &canMailbox) == HAL_OK) {
//		printf("Successfully sent message with id: %lu \n", TxHeader.StdId);
		return 1;
	}

//	printf("Failed at adding message with id: %lu to Tx Fifo", TxHeader.StdId);
	// This might be for many reasons including the Tx Fifo being full, the error can be read from hcan->ErrorCode
	return 0;
}

/*
  get a 16 byte unique ID for this node, this should be based on the CPU unique ID or other unique ID
 */
void getUniqueID(uint8_t id[16]){
    uint32_t HALUniqueIDs[4];
    // Make Unique ID out of the 96-bit STM32 UID
    memset(id, 0, 16);
    HALUniqueIDs[0] = HAL_GetUIDw0();
    HALUniqueIDs[1] = HAL_GetUIDw1();
    HALUniqueIDs[2] = HAL_GetUIDw2();
    HALUniqueIDs[3] = HAL_GetUIDw1(); // repeating UIDw1 for this, no specific reason I chose this..
    memcpy(id, HALUniqueIDs, 16);
}

void processCanardRxQueue(CanardInstance *canard) {
    struct dequeueRxReturnItem dequeueRxReturnItem = dequeueRxFrame();
    const uint64_t timestamp = HAL_GetTick() * 1000ULL;

    if (dequeueRxReturnItem.isSuccess) {
        canardHandleRxFrame(canard, &(dequeueRxReturnItem.frame), timestamp);
    }
}

void processCanardTxQueue(CAN_HandleTypeDef *hcan, CanardInstance *canard) {
    // Transmitting

    for (const CanardCANFrame *tx_frame ; (tx_frame = canardPeekTxQueue(canard)) != NULL;) {
        const int16_t tx_res = canardSTM32Transmit(hcan, tx_frame);

        if (tx_res <= 0) {
            printf("Transmit error %d\n", tx_res);
        } else if (tx_res > 0) {
            printf("Successfully transmitted message\n");
        }

        // Pop canardTxQueue either way
        canardPopTxQueue(canard);
    }
}

void processTasks(CAN_HandleTypeDef *hcan, CanardInstance *canard, uint64_t *next_1hz_service) {
  processCanardRxQueue(canard);
  processCanardTxQueue(hcan, canard);

  uint64_t ts = HAL_GetTick();

  // 1 Hz Tasks
  if (ts >= *next_1hz_service){
    *next_1hz_service += 1000ULL;
    //Purge transfers that are no longer transmitted. This can free up some memory
    canardCleanupStaleTransfers(canard, ts);
  }
}

void canardSTM32RxFifo0MsgPending(CAN_HandleTypeDef *hcan) {
  // Receiving
    CanardCANFrame rx_frame;

    const int16_t rx_res = canardSTM32Recieve(hcan, CAN_RX_FIFO0, &rx_frame);

    if (rx_res < 0) {
        printf("Receive error %d\n", rx_res);
    }
    else if (rx_res > 0)        // Success - process the frame
    {
        enqueueRxReturnCode enqueueRxReturnCode = enqueueRxFrame(&rx_frame);

        switch (enqueueRxReturnCode) {
            case RX_ENQUEUE_EMPTYITEM:
                printf("Rx frame is empty");
                break;
            case RX_ENQUEUE_MALLOCFAIL:
                printf("CanardRxQueueItem memory allocation failed");
                break;
            case RX_ENQUEUE_OVERFLOW:
                printf("rxQueue is full, oldest frame has been removed");
                break;
            case RX_ENQUEUE_SUCCESS:
                break;
        }
    }
}
