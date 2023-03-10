#include <main.h>
#include "Motor_Unit_Debug.h"
#include "cyapicallbacks.h"
#include "MotorDrive.h"
#include "MotorBoardCAN.h"
#include "MotorBoardFSM.h"
#include "PID.h"
#include <project.h>
#include <math.h>

#ifdef RGB_LED_ARRAY
#include "LED_Array.h"
extern const uint32 StripLights_CLUT[ ];
#endif

//LED
uint8_t CAN_time_LED = 0;
uint8_t ERRORTimeLED = 0;

int32_t millidegreeTarget = 0;

//Uart variables
char txData[TX_DATA_SIZE];

//drive variable
int16 nextPWM = 0;
extern uint8 invalidate;
uint8_t ignoreLimSw = 0;
uint8_t encoderTimeOut = 0;

// Motor Unit Variables
uint8_t bound_set1;
uint8_t bound_set2;
int32_t enc_lim_1;
int32_t enc_lim_2;

uint8 address = 0;

//Status and Data Structs
volatile uint8 drive = 0;
uint8_t CAN_check_delay = 0;
CANPacket can_recieve;
CANPacket can_send;

CY_ISR(Period_Reset_Handler) {
    int timer = Timer_PWM_ReadStatusRegister();
    invalidate++;
    CAN_time_LED++;
    CAN_check_delay ++;
    ERRORTimeLED++;
    encoderTimeOut++;
    if (encoderTimeOut >= 2){
        encoderTimeOut = 0;
        SendEncoderData(&can_send);
    }
    if (invalidate >= 20){
        set_PWM(0, 0, 0);   
    }
    if (ERRORTimeLED >= 3) {
        #ifdef ERROR_LED
        ERROR_LED_Write(LED_OFF);
        #endif
        #ifdef DEBUG_LED1   
        DEBUG_LED_1_Write(LED_OFF);
        #endif
        #ifdef DEBUG_LED2
        DEBUG_LED_2_Write(LED_OFF);
        #endif
    }
    if (CAN_time_LED >= 3){
        #ifdef CAN_LED
        CAN_LED_Write(LED_OFF);
        #endif
    }
}
  
CY_ISR(Pin_Limit_Handler){
    #ifdef PRINT_LIMIT_SW_TRIGGER
    sprintf(txData,"LimitSW triggerd Stat: %x \r\n", Status_Reg_Switches_Read() & 0b11);
    UART_UartPutString(txData);
    #endif
    
    set_PWM(GetCurrentPWM(), ignoreLimSw, Status_Reg_Switches_Read());
    
    #ifdef CAN_TELEM_SEND
    AssembleLimitSwitchAlertPacket(&can_send, DEVICE_GROUP_JETSON, 
        DEVICE_SERIAL_JETSON, Status_Reg_Switches_Read() & 0b11);
    SendCANPacket(&can_send);
    #endif
    
    char num[256];
    sprintf(num,"LimitSW enc lim 1: %i \r\n", enc_lim_1);
    UART_UartPutString(num);
    
    char num2[256];
    sprintf(num2,"LimitSW enc lim 2: %i \r\n", enc_lim_2);
    UART_UartPutString(num2);
            

    if (bound_set1 && Pin_Limit_1_Read()) {
        QuadDec_SetCounter((int)round(GetkPPJR() / (360*1000.0) * enc_lim_1));
        UART_UartPutString("Change limit 1\n\r");
    }
    if (bound_set2 && Pin_Limit_2_Read()){
        UART_UartPutString("Change limit 2\n\r");
        QuadDec_SetCounter((int)round(GetkPPJR() / (360*1000.0) * enc_lim_2));
    }
}

int main(void)
{ 
    Initialize();
  
    #ifdef RGB_LED_ARRAY
    StripLights_DisplayClear(StripLights_BLACK);
    #endif
    
    for(;;)
    {
        switch(GetState()) {
            case(UNINIT):
                //idle animation
                SetStateTo(CHECK_CAN);
                break;
            case(SET_PWM):
                set_PWM(nextPWM, ignoreLimSw, Status_Reg_Switches_Read());
                SetStateTo(CHECK_CAN);
                break;
            case(CALC_PID):
                SetPosition(millidegreeTarget);   
                SetStateTo(CHECK_CAN);
                break;
            case(QUEUE_ERROR):
                SetStateTo(CHECK_CAN);
                break;
            case(CHECK_CAN):
                NextStateFromCAN(&can_recieve, &can_send);
                #ifdef PRINT_CAN_PACKET
                PrintCanPacket(can_recieve);
                #endif
                break;
            default:
                //Should Never Get Here
                //TODO: ERROR
                GotoUninitState();
                break;

        }
        #ifdef PRINT_FSM_STATE_MODE
        sprintf(txData, "Mode: %x State:%x \r\n", GetMode(), GetState());
        UART_UartPutString(txData);
        #endif
        #ifdef PRINT_SET_PID_CONST
        sprintf(txData, "P: %d I: %d D: %d PPJ: %d Ready: %d \r\n", GetkPosition(), GetkIntegral() 
        ,GetkDerivative(), GetkPPJR(), PIDconstsSet());
        UART_UartPutString(txData);
        #endif
        #ifdef PRINT_ENCODER_VALUE
        sprintf(txData, "Encoder Value: %d  \r\n", QuadDec_GetCounter());
        UART_UartPutString(txData);
        #endif
    }
}
 
void Initialize(void) {
     // Enable global interrupts. LED arrays need this first
    CyGlobalIntEnable;
    
    #ifdef RGB_LED_ARRAY
    initalize_LEDs(LOW_LED_POWER);
    #endif
    
    Status_Reg_Switches_InterruptEnable();
    
    address = Can_addr_Read();
    
    UART_Start();
    sprintf(txData, "Dip Addr: %x \r\n", address);
    UART_UartPutString(txData);
    
    #ifdef ERROR_LED
    ERROR_LED_Write(~(address >> 3 & 1));
    #endif
    #ifdef DEBUG_LED2
    DEBUG_LED_2_Write(~(address >> 2) & 1);
    #endif
    #ifdef DEBUG_LED1
    DEBUG_LED_1_Write(~(address >> 1) & 1);
    #endif 
    #ifdef CAN_LED
    CAN_LED_Write(~address & 1);
    #endif
    
    InitCAN(0x4, (int)address);
    Timer_PWM_Start();
    QuadDec_Start();
    PWM_Motor_Start();  
    isr_Limit_1_StartEx(Pin_Limit_Handler);
    isr_period_PWM_StartEx(Period_Reset_Handler);
}

// FIXME: receivedPacket should really be a refernce to the can packet to avoid the copy
void PrintCanPacket(CANPacket receivedPacket){
    for(int i = 0; i < receivedPacket.dlc; i++) {
        sprintf(txData, "Byte%d %x   ", i+1, receivedPacket.data[i]);
        UART_UartPutString(txData);
    }

    sprintf(txData,"ID:%x %x %x\r\n",receivedPacket.id >> 10, 
        (receivedPacket.id >> 6) & 0xF , receivedPacket.id & 0x3F);
    UART_UartPutString(txData);
}

uint16_t ReadCAN(CANPacket *receivedPacket){
    volatile int error = PollAndReceiveCANPacket(receivedPacket);
    if(!error && GetPacketID(receivedPacket) != 0xF6){
        #ifdef CAN_LED
        CAN_LED_Write(LED_ON);
        #endif
        CAN_time_LED = 0;
        return receivedPacket->data[0];
    }
    return NO_NEW_CAN_PACKET; //Means no Packet
}

void DisplayErrorCode(uint8_t code) 
{    
    #ifdef ERROR_LED
    ERROR_LED_Write(LED_OFF);
    #endif
    #ifdef DEBUG_LED1   
    DEBUG_LED_1_Write(LED_OFF);
    #endif
    #ifdef DEBUG_LED2
    DEBUG_LED_2_Write(LED_OFF);
    #endif
    
    ERRORTimeLED = 0;
    ERROR_LED_Write(LED_ON);
    
    #ifdef PRINT_MOTOR_ERROR
        //TODO: PRINT ERROR
    #endif

    switch(code)
    {   
        //case0: CAN Packet ERROR
        case 1://Mode Error
            #ifdef DEBUG_LED1
            DEBUG_LED_1_Write(LED_ON);
            #endif
            break;
        case 2:
            #ifdef DEBUG_LED2
            DEBUG_LED_2_Write(LED_ON);
            #endif
            break;
        case 3:
            #ifdef DEBUG_LED1
            DEBUG_LED_1_Write(LED_ON);
            #endif
            #ifdef DEBUG_LED2
            DEBUG_LED_2_Write(LED_ON);
            #endif
            break;
        default:
            //some error
            break;
    }

}