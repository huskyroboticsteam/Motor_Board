/*
 * Port.h
 *
 *  Created on: Nov 14, 2024
 *      Author: eunbecha
 */

#ifndef SRC_PORT_H_
#define SRC_PORT_H_

#pragma once

#include "CANPacket.h"


void InitCAN(int deviceGroup, int deviceAddress);

//TODO: define constants for these error codes
//Returns 0x0 for successful send
//returns 0x1 for generic error
//returns 0x2 all output buffers are full
//Reserve higher numbers for future error codes
int SendCANPacket(CANPacket *packetToSend);

//Returns 0x0 for SUCCESSFUL packet return
//Returns 0x1 for no message received
//Returns 0x2 for generic error
//Reserve higher numbers for future error codes
int PollAndReceiveCANPacket(CANPacket *receivedPacket);

uint8_t getLocalDeviceSerial();
uint8_t getLocalDeviceGroup();

//Returns constant
uint8_t getChipType();

//Chip type constants
//TODO: Find specific chip names
#define CHIP_TYPE_TEMPLATE              0x00
#define CHIP_TYPE_STM32Fxxx             0x01
#define CHIP_TYPE_PSOC_CY8C4248AZI_L485 0x02
#define CHIP_TYPE_AT90CANxxx            0x03
#define CHIP_TYPE_JETSON                0x04

//Error code constants
#define ERROR_NONE              0x00
#define ERROR_GENERIC_ERROR     0x01
#define ERROR_NULL_POINTER      0x02

#endif /* SRC_PORT_H_ */
