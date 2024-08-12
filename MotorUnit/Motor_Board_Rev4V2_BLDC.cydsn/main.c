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

#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include "main.h"
#include "cyapicallbacks.h"
#include "../MotorFirmwareV2/CAN_Stuff.h"
#include "../MotorFirmwareV2/FSM_Stuff.h"
#include "../MotorFirmwareV2/MotorDrive.h"
#include "../MotorFirmwareV2/PIDControl.h"
#include "../HindsightCAN/CANLibrary.h"

// LED stuff
volatile uint8 CAN_time_LED = 0;
volatile uint8 ERROR_time_LED = 0;

// UART stuff
char txData[TX_DATA_SIZE];

// CAN stuff
CANPacket can_recieve;
CANPacket can_send;

int main() { 
    Initialize();
    
    for(;;)
    {
        int err = 0;
        
        if (!PollAndReceiveCANPacket(&can_recieve)) {
            LED_CAN_Write(ON);
            CAN_time_LED = 0;
            // PrintCanPacket(&can_recieve); // DEBUG
            err = ProcessCAN(&can_recieve, &can_send);
        }
        
        if (err) DisplayErrorCode(err);
        
        if (DBG_UART_SpiUartGetRxBufferSize()) {
            DebugPrint(DBG_UART_UartGetByte());
        }
        
        LED_DBG_Write(GetMode() == MODE_UNINIT); // turn on when initialized
        CyDelay(1);
    }
}

void Initialize(void) {
    CyGlobalIntEnable;
    
    StartCAN(ReadDIP());
    DBG_UART_Start();
    Timer_Periodic_Start();
    Timer_PID_Start();
    PWM_Motor_Start();
    QuadDec_Enc_Start();
    // SetEncOffset(0);
    ADC_Start();
    
    sprintf(txData, "Address: %x\r\n", GetAddress());
    Print(txData);
    
    isr_PID_StartEx(PID_Handler);
    isr_LED_StartEx(LED_Handler);
    isr_Drive_StartEx(Drive_Handler);
}

int ReadDIP() {
    return Status_Reg_DIP_Read() & 0x0F;
}

void DebugPrint(char input) {
    switch(input) {
        case 'm': // Mode
            sprintf(txData, "Address: %x", GetAddress());
            Print(txData);
            Print(" Mode: ");
            if (GetMode() == MODE_UNINIT) Print("UNINIT");
            else if (GetMode() == MODE_PWM_CTRL) Print("PWM");
            else if (GetMode() == MODE_PID_CTRL) Print("PID");
            break;
        // case 'a':
        //     SetMode(MODE_PWM_CTRL);
        //     SetPWM(200);
        //     break;
        // case 'd':
        //     SetMode(MODE_PWM_CTRL);
        //     SetPWM(-200);
        //     break;
        case 'p': // Position
            sprintf(txData, "Pos:%li PWM:%li", 
                GetPosition(), GetCurrentPWM());
            Print(txData);
            break;
        case 'o': // Raw sensor
            sprintf(txData, "Enc:%li Pot:%li Limits:", GetEncValue(), GetPotValue());
            Print(txData);
            PrintIntBin(GetLimitStatus());
            break;
        case 'i': // PID Config
            sprintf(txData, "Motor Target:%li P:%li I:%li D:%li",
                GetPIDState().target, GetPIDConfig().kP, GetPIDConfig().kI, GetPIDConfig().kD);
            Print(txData);
            break;
        case 'k': 
            ADC_StartConvert();
            ADC_IsEndConversion(ADC_WAIT_FOR_RESULT);
            PrintInt(ADC_GetResult16(0));
            // Print(" ");
            // PrintInt(ADC_GetResult16(1));
            // Print(" ");
            // PrintInt(ADC_GetResult16(2));
            break;
        case 'c': // Conversion
            sprintf(txData, "Motor1 TickMin:%li TickMax:%li mDegMin:%li mDegMax:%li",
                GetConversion().tickMin, GetConversion().tickMax,
                GetConversion().mDegMin, GetConversion().mDegMax);
            Print(txData);
            break;
        default:
            Print("what");
            break;
    }
    Print("\r\n");
}

void PrintCanPacket(CANPacket* packet) {
    sprintf(txData, "ID %X DLC %X DATA", packet->id, packet->dlc);
    Print(txData);
    for(int i = 0; i < packet->dlc; i++ ) {
        sprintf(txData," %02X", packet->data[i]);
        Print(txData);
    }
    Print("\r\n");
}

void DisplayErrorCode(uint8 code) {    
    ERROR_time_LED = 0;
    LED_ERR_Write(ON);
    
    sprintf(txData, "Error %X: ", code);
    Print(txData);

    switch(code) {
        case ERROR_INVALID_PACKET:
            Print("Packet type not recognized\r\n");
            break;
        case ERROR_PWM_NOT_ENABLED:
            Print("PWM is not enabled\r\n");
            break;
        case ERROR_LIMIT:
            Print("At limit\r\n");
            break;
        case ERROR_WRONG_MODE:
            Print("Wrong mode\r\n");
            break;
        case ERROR_MODE_CHANGE:
            Print("Failed to change mode\r\n");
            break;
        case ERROR_INVALID_TTC:
            Print("Cannot send that data type\r\n");
            break;
        case ERROR_ESTOP:
            Print("ESTOP\r\n");
            break;
        default:
            Print(":(\r\n");
            break;
    }
}

CY_ISR(LED_Handler) {
    CAN_time_LED++;
    ERROR_time_LED++;
    
    if (ERROR_time_LED >= 10)
        LED_ERR_Write(OFF);
    if (CAN_time_LED >= 2)
        LED_CAN_Write(OFF);
}

/* [] END OF FILE */
