/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <canard.h>
#include <_timespec.h>
#include <time.h>
extern "C" {
#include "dronecan_msgs.h"
#include "canard_stm32_driver.h"
#include "node_settings.h"
#include "servo_control.hpp"
}
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CLOCK_MONOTONIC 1  // Define CLOCK_MONOTONIC for compatibility

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

CAN_FilterTypeDef canfil; //CAN Bus Filter, CHECK: Can maybe remove (from Hardy previous testing)

static CanardInstance canard;
static uint8_t memory_pool[1024];

static ServoController servo_controller;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static struct uavcan_protocol_NodeStatus node_status;

// Custom clock_gettime function for STM32
int clock_gettime(clockid_t clk_id, struct timespec *tp) {
    if (clk_id == CLOCK_MONOTONIC) {
        uint32_t millis = HAL_GetTick();  // Get the current tick in milliseconds
        tp->tv_sec = millis / 1000;       // Convert milliseconds to seconds
        tp->tv_nsec = (millis % 1000) * 1000000;  // Convert remaining milliseconds to nanoseconds
        return 0;  // Success
    }
    return -1;  // Unsupported clock ID
}


/*
  get a 16 byte unique ID for this node, this should be based on the CPU unique ID or other unique ID
 */
void getUniqueID(uint8_t id[16]){
	uint32_t HALUniqueIDs[3];
// Make Unique ID out of the 96-bit STM32 UID and fill the rest with 0s
	memset(id, 0, 16);
	HALUniqueIDs[0] = HAL_GetUIDw0();
	HALUniqueIDs[1] = HAL_GetUIDw1();
	HALUniqueIDs[2] = HAL_GetUIDw2();
	memcpy(id, HALUniqueIDs, 12);
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
	// Receiving
	CanardCANFrame rx_frame;

	const uint64_t timestamp = HAL_GetTick() * 1000ULL;
	const int16_t rx_res = canardSTM32Recieve(hcan, CAN_RX_FIFO0, &rx_frame);

	if (rx_res < 0) {
		printf("Receive error %d\n", rx_res);
	}
	else if (rx_res > 0)        // Success - process the frame
	{
		canardHandleRxFrame(&canard, &rx_frame, timestamp);
	}
}
// NOTE: All canard handlers and senders are based on this reference: https://dronecan.github.io/Specification/7._List_of_standard_data_types/
// Alternatively, you can look at the corresponding generated header file in the dsdlc_generated folder

// Canard Handlers ( Many have code copied from libcanard esc_node example: https://github.com/dronecan/libcanard/blob/master/examples/ESCNode/esc_node.c )

void handle_NodeStatus(CanardInstance *ins, CanardRxTransfer *transfer) {
	struct uavcan_protocol_NodeStatus nodeStatus;

	if (uavcan_protocol_NodeStatus_decode(transfer, &nodeStatus)) {
		return;
	}

	printf("Node health: %ud Node Mode: %ud\n", nodeStatus.health, nodeStatus.mode);

	printf("Node Health ");

	switch (nodeStatus.health) {
	case UAVCAN_PROTOCOL_NODESTATUS_HEALTH_OK:
		printf("OK\n");
		break;
	case UAVCAN_PROTOCOL_NODESTATUS_HEALTH_WARNING:
		printf("WARNING\n");
		break;
	case UAVCAN_PROTOCOL_NODESTATUS_HEALTH_ERROR:
		printf("ERROR\n");
		break;
	case UAVCAN_PROTOCOL_NODESTATUS_HEALTH_CRITICAL:
		printf("CRITICAL\n");
		break;
	default:
		printf("UNKNOWN?\n");
		break;
	}

	printf("Node Mode ");

	switch(nodeStatus.mode) {
	case UAVCAN_PROTOCOL_NODESTATUS_MODE_OPERATIONAL:
		printf("OPERATIONAL\n");
		break;
	case UAVCAN_PROTOCOL_NODESTATUS_MODE_INITIALIZATION:
		printf("INITIALIZATION\n");
		break;
	case UAVCAN_PROTOCOL_NODESTATUS_MODE_MAINTENANCE:
		printf("MAINTENANCE\n");
		break;
	case UAVCAN_PROTOCOL_NODESTATUS_MODE_SOFTWARE_UPDATE:
		printf("SOFTWARE UPDATE\n");
		break;
	case UAVCAN_PROTOCOL_NODESTATUS_MODE_OFFLINE:
		printf("OFFLINE\n");
		break;
	default:
		printf("UNKNOWN?\n");
		break;
	}
}

void handle_NotifyState(CanardInstance *ins, CanardRxTransfer *transfer) {
	struct ardupilot_indication_NotifyState notifyState;

	if (ardupilot_indication_NotifyState_decode(transfer, &notifyState)) {
		return;
	}

	uint32_t nl = notifyState.vehicle_state & 0xFFFFFFFF;  // ignoring the last 32 bits for printing since the highest vehicle_state value right now is 23 even though they're allowed to be up to 64bit unsigned integer

	printf("Vehicle State: %lu ", nl);

	if (notifyState.aux_data.len > 0) {
		printf("Aux Data: 0x");

		for (int i = 0; i < notifyState.aux_data.len; i++) {
			printf("%02x", notifyState.aux_data.data[i]);
		}
	}

	printf("\n");

}

/*
 * handle a servo ArrayCommand request
 */
static void handle_ArrayCommand(CanardInstance *ins, CanardRxTransfer *transfer)
{
	printf("entered array command\n");
    struct uavcan_equipment_actuator_ArrayCommand cmd;
    if (uavcan_equipment_actuator_ArrayCommand_decode(transfer, &cmd)) { //return true if decode is invalid
        return;
    }
    uint64_t tnow = HAL_GetTick() * 1000ULL;
    for (uint8_t i=0; i < cmd.commands.len; i++) {
        uint8_t actuator_id = cmd.commands.data[i].actuator_id;
		float actuator_pos = cmd.commands.data[i].command_value;

        switch (cmd.commands.data[i].command_type) {
        case UAVCAN_EQUIPMENT_ACTUATOR_COMMAND_COMMAND_TYPE_UNITLESS:
			servo_controller.update_servo_position(actuator_id, actuator_pos);
            break;
        case UAVCAN_EQUIPMENT_ACTUATOR_COMMAND_COMMAND_TYPE_PWM:
			servo_controller.update_servo_position(actuator_id, actuator_pos);
            break;
        }
        servo_controller.update_servo_last_update_us(actuator_id, tnow);

        //call a function to run the servos with the data set in this function
    }
}

/*
  handle a GetNodeInfo request
*/
// TODO: All the data in here is temporary for testing. If actually need to send valid data, edit accordingly.
void handle_GetNodeInfo(CanardInstance *ins, CanardRxTransfer *transfer) {
	printf("GetNodeInfo request from %d\n", transfer->source_node_id);

	uint8_t buffer[UAVCAN_PROTOCOL_GETNODEINFO_RESPONSE_MAX_SIZE];
	struct uavcan_protocol_GetNodeInfoResponse pkt;

	memset(&pkt, 0, sizeof(pkt));

	node_status.uptime_sec = HAL_GetTick() / 1000ULL;
	pkt.status = node_status;

	// fill in your major and minor firmware version
	pkt.software_version.major = 1;
	pkt.software_version.minor = 0;
	pkt.software_version.optional_field_flags = 0;
	pkt.software_version.vcs_commit = 0; // should put git hash in here

	// should fill in hardware version
	pkt.hardware_version.major = 1;
	pkt.hardware_version.minor = 0;

	// just setting all 16 bytes to 1 for testing
	getUniqueID(pkt.hardware_version.unique_id);

	strncpy((char*)pkt.name.data, "SERVONode", sizeof(pkt.name.data));
	pkt.name.len = strnlen((char*)pkt.name.data, sizeof(pkt.name.data));

	uint16_t total_size = uavcan_protocol_GetNodeInfoResponse_encode(&pkt, buffer);

	canardRequestOrRespond(ins,
						   transfer->source_node_id,
						   UAVCAN_PROTOCOL_GETNODEINFO_SIGNATURE,
						   UAVCAN_PROTOCOL_GETNODEINFO_ID,
						   &transfer->transfer_id,
						   transfer->priority,
						   CanardResponse,
						   &buffer[0],
						   total_size);
}

// Canard Senders

/*
  send the 1Hz NodeStatus message. This is what allows a node to show
  up in the DroneCAN GUI tool and in the flight controller logs
 */
void send_NodeStatus(void) {
    uint8_t buffer[UAVCAN_PROTOCOL_GETNODEINFO_RESPONSE_MAX_SIZE];

    node_status.uptime_sec = HAL_GetTick() / 1000UL;
    node_status.health = UAVCAN_PROTOCOL_NODESTATUS_HEALTH_OK;
    node_status.mode = UAVCAN_PROTOCOL_NODESTATUS_MODE_OPERATIONAL;
    node_status.sub_mode = 0;

    // put whatever you like in here for display in GUI
    node_status.vendor_specific_status_code = 1234;

    uint32_t len = uavcan_protocol_NodeStatus_encode(&node_status, buffer);

    // we need a static variable for the transfer ID. This is
    // incremeneted on each transfer, allowing for detection of packet
    // loss
    static uint8_t transfer_id;

    canardBroadcast(&canard,
                    UAVCAN_PROTOCOL_NODESTATUS_SIGNATURE,
                    UAVCAN_PROTOCOL_NODESTATUS_ID,
                    &transfer_id,
                    CANARD_TRANSFER_PRIORITY_LOW,
                    buffer,
                    len);
}

// Canard Util

bool shouldAcceptTransfer(const CanardInstance *ins,
                                 uint64_t *out_data_type_signature,
                                 uint16_t data_type_id,
                                 CanardTransferType transfer_type,
                                 uint8_t source_node_id)
{
	if (transfer_type == CanardTransferTypeRequest) {
	// check if we want to handle a specific service request
		switch (data_type_id) {
		case UAVCAN_PROTOCOL_GETNODEINFO_ID: {
			*out_data_type_signature = UAVCAN_PROTOCOL_GETNODEINFO_REQUEST_SIGNATURE;
			return true;
		}
		}
	}
	if (transfer_type == CanardTransferTypeResponse) {
		// check if we want to handle a specific service request
		switch (data_type_id) {
		}
	}
	if (transfer_type == CanardTransferTypeBroadcast) {
		// see if we want to handle a specific broadcast packet
		switch (data_type_id) {
		case UAVCAN_EQUIPMENT_ACTUATOR_ARRAYCOMMAND_ID: {
			*out_data_type_signature = UAVCAN_EQUIPMENT_ACTUATOR_ARRAYCOMMAND_SIGNATURE; //Change this
			return true;
		}
		case UAVCAN_PROTOCOL_NODESTATUS_ID: {
			*out_data_type_signature = UAVCAN_PROTOCOL_NODESTATUS_SIGNATURE;
			return true;
		}
		case ARDUPILOT_INDICATION_NOTIFYSTATE_ID: {
			*out_data_type_signature = ARDUPILOT_INDICATION_NOTIFYSTATE_SIGNATURE;
			return true;
		}
		}
	}
	// we don't want any other messages
	return false;
}

void onTransferReceived(CanardInstance *ins, CanardRxTransfer *transfer) {
	// switch on data type ID to pass to the right handler function
//	printf("Transfer type: %du, Transfer ID: %du \n", transfer->transfer_type, transfer->data_type_id);
//	printf("0x");
//		for (int i = 0; i < transfer->payload_len; i++) {
//			printf("%02x", transfer->payload_head[i]);
//		}
//
//		printf("\n");
	if (transfer->transfer_type == CanardTransferTypeRequest) {
		// check if we want to handle a specific service request
		switch (transfer->data_type_id) {
		case UAVCAN_PROTOCOL_GETNODEINFO_ID: {
			handle_GetNodeInfo(ins, transfer);
			break;
		}
		}
	}
	if (transfer->transfer_type == CanardTransferTypeResponse) {
		switch (transfer->data_type_id) {
		}
	}
	if (transfer->transfer_type == CanardTransferTypeBroadcast) {
		// check if we want to handle a specific broadcast message
		switch (transfer->data_type_id) {
		case UAVCAN_EQUIPMENT_ACTUATOR_ARRAYCOMMAND_ID:{
			handle_ArrayCommand(ins, transfer);
			break;
		}
		case UAVCAN_PROTOCOL_NODESTATUS_ID: {
			handle_NodeStatus(ins, transfer);
			break;
		}
		case ARDUPILOT_INDICATION_NOTIFYSTATE_ID: {
			handle_NotifyState(ins, transfer);
			break;
		}
		}
	}
}

void processCanardTxQueue(CAN_HandleTypeDef *hcan) {
	// Transmitting

	for (const CanardCANFrame *tx_frame ; (tx_frame = canardPeekTxQueue(&canard)) != NULL;) {
		const int16_t tx_res = canardSTM32Transmit(hcan, tx_frame);

		if (tx_res < 0) {
			printf("Transmit error %d\n", tx_res);
		} else if (tx_res > 0) {
			printf("Successfully transmitted message\n");
		}

		// Pop canardTxQueue either way
		canardPopTxQueue(&canard);
	}
}

/*
  This function is called at 1 Hz rate from the main loop.
*/
void process1HzTasks(uint64_t timestamp_usec) {
    /*
      Purge transfers that are no longer transmitted. This can free up some memory
    */
    canardCleanupStaleTransfers(&canard, timestamp_usec);

    /*
      Transmit the node status message
    */
    send_NodeStatus();
}

/*
  send servo status at 50Hz
*/
static void send_ServoStatus(void)
{
    // // send a separate status packet for each servo
    // for (uint8_t i=0; i<NUM_SERVOS; i++) {
    //     struct uavcan_equipment_actuator_Status pkt;
    //     memset(&pkt, 0, sizeof(pkt));
    //     uint8_t buffer[UAVCAN_EQUIPMENT_ACTUATOR_STATUS_MAX_SIZE];

    //     // make up some synthetic status data
    //     pkt.actuator_id = i;
    //     pkt.position = servos[i].position;
    //     pkt.force = 0;
    //     pkt.speed = 0; // m/s or rad/s
    //     pkt.power_rating_pct = 0;

    //     uint32_t len = uavcan_equipment_actuator_Status_encode(&pkt, buffer);

    //     // we need a static variable for the transfer ID. This is
    //     // incremeneted on each transfer, allowing for detection of packet
    //     // loss
    //     static uint8_t transfer_id;

    //     canardBroadcast(&canard,
    //                     UAVCAN_EQUIPMENT_ACTUATOR_STATUS_SIGNATURE,
    //                     UAVCAN_EQUIPMENT_ACTUATOR_STATUS_ID,
    //                     &transfer_id,
    //                     CANARD_TRANSFER_PRIORITY_LOW,
    //                     buffer,
    //                     len);
    // }
}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CAN1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */


	//CHECK: can maybe remove (prev code from Hardy Testing)
	canfil.FilterBank = 0;
	canfil.FilterMode = CAN_FILTERMODE_IDMASK;
	canfil.FilterFIFOAssignment = CAN_RX_FIFO0;
	canfil.FilterIdHigh = 0;
	canfil.FilterIdLow = 0;
	canfil.FilterMaskIdHigh = 0;
	canfil.FilterMaskIdLow = 0;
	canfil.FilterScale = CAN_FILTERSCALE_32BIT;
	canfil.FilterActivation = ENABLE;
	canfil.SlaveStartFilterBank = 14;

	HAL_CAN_ConfigFilter(&hcan1,&canfil);
  	HAL_CAN_Start(&hcan1);
  	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

	// configuring the pwm wave for servo module
	servo_controller.start_servos();

	canardInit(&canard,
			  	  memory_pool,
				  sizeof(memory_pool),
				  onTransferReceived,
				  shouldAcceptTransfer,
				  NULL);

	uint64_t next_1hz_service_at = HAL_GetTick();
	uint64_t next_50hz_service_at = HAL_GetTick();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  processCanardTxQueue(&hcan1);
	  const uint64_t ts = HAL_GetTick();

	  if (ts >= next_1hz_service_at){
		  next_1hz_service_at += 1000ULL;
		  process1HzTasks(ts);
	  }
	  if (ts >= next_50hz_service_at){
		  next_50hz_service_at += 1000ULL/50U;
		  send_ServoStatus();
	  }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 12;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
int __io_putchar(int ch) {
	ITM_SendChar(ch);
	return ch;
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
