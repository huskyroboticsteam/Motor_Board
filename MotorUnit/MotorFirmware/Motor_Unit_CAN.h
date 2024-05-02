/* ========================================
 *
 * Copyright YOUR COMPANY, THE YEAR
 * All Rights Reserved
 * UNPUBLISHED, LICENSED SOFTWARE.
 *
 * CONFIDENTIAL AND PROPRIETARY INFORMATION
 * WHICH IS THE PROPERTY OF your company.
 *
 * ========================================
*/

#pragma once

#include <project.h>
#include "../HindsightCAN/CANLibrary.h"
void NextStateFromCAN(CANPacket *receivedPacket, CANPacket *packetToSend);
void SendEncoderData(CANPacket *packetToSend);
#define NO_NEW_CAN_PACKET 0xFFFF

/* [] END OF FILE */