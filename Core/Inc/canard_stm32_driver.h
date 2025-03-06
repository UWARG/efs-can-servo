/*
 * canard_stm32_driver.h
 *
 *  Created on: Jul 9, 2024
 *      Author: ronik
 */

#ifndef INC_CANARD_STM32_DRIVER_H_
#define INC_CANARD_STM32_DRIVER_H_

typedef struct {
  CAN_HandleTypeDef *hcan;
  CanardInstance *canard;
  void* mem_pool;
  size_t mem_pool_size;
  CanardOnTransferReception on_recep;
  CanardShouldAcceptTransfer should_accept;
} InitTypeDef;

void init(InitTypeDef *initParams);

int16_t canardSTM32Recieve(CAN_HandleTypeDef *hcan, uint32_t RxLocation, CanardCANFrame *const rx_frame);
int16_t canardSTM32Transmit(CAN_HandleTypeDef *hcan, const CanardCANFrame* const tx_frame);

void processCanardRxQueue(CanardInstance *canard);
void processCanardTxQueue(CAN_HandleTypeDef *hcan, CanardInstance *canard);
void processTasks(CAN_HandleTypeDef *hcan, CanardInstance *canard, uint64_t *next_1hz_service);

void getUniqueID(uint8_t id[16]);

void canardSTM32RxFifo0MsgPending();
void canardSTM32RxFifo1MsgPending();

void canardSTM32RxFifo0Full();
void canardSTM32RxFifo1Full();

void canardSTM32TxMailbox0Complete();
void canardSTM32TxMailbox1Complete();
void canardSTM32TxMailbox2Complete();

void canardSTM32TxMailbox0Abort();
void canardSTM32TxMailbox1Abort();
void canardSTM32TxMailbox2Abort();

#endif /* INC_CANARD_STM32_DRIVER_H_ */
