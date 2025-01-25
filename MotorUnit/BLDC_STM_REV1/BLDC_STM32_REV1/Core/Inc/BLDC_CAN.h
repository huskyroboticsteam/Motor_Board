/*
 * BLDC_CAN.h
 *
 *  Created on: Jan 21, 2025
 *      Author: eunbecha
 */

#ifndef INC_BLDC_CAN_H_
#define INC_BLDC_CAN_H_

	#include "main.h"
	#include "can.h"

	// Define a constant for no new CAN packet
	#define NO_NEW_CAN_PACKET 0xFFFF

	// Function prototypes
	void NextStateFromCAN(CAN_RxHeaderTypeDef *rxHeader, uint8_t *rxData, CAN_TxHeaderTypeDef *txHeader, uint8_t *txData, uint32_t *txMailbox);
	void SendEncoderData(CAN_TxHeaderTypeDef *txHeader, uint8_t *txData, uint32_t *txMailbox);



#endif /* INC_BLDC_CAN_H_ */
