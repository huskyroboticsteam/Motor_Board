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

//PID varaibles
#include <project.h>
#include "PositionPID.h"
#include "MotorDrive.h"
#include "FSM_Stuff.h"

PID_Config PID = {.maxIntegral=500, .maxPWM=1023};
volatile PID_State PID_state = {};

uint8 PID_enable;

int StartPID() {
    int err = 0;
    
    if (PID.kP_set && PID.kI_set && PID.kD_set && GetConversion().ratio_set) {
        PID_state.integral = 0;
        PID_state.last_error = 0;
        PID_enable = 1;
    } else err = 1;
    
    return err;
}

void StopPID() {
    PID_enable = 0;
    PID_state.target_set = 0;
    SetPWM(0);
}

void SetkPosition(int32 kP) {
    PID.kP = kP;
    PID.kP_set = 1;
}
void SetkIntegral(int32 kI) {
    PID.kI = kI;
    PID.kI_set = 1;
    
}
void SetkDerivative(int32 kD) {
    PID.kD = kD;
    PID.kD_set = 1;
}

void SetPIDMaxPWM( uint16 maxPWM) {
    PID.maxPWM = maxPWM;
}

PID_Config GetPIDConfig() {
    return PID;
}
PID_State GetPIDState() {
    return PID_state;
}


void SetPIDTarget(int32 mDegs) {
    PID_state.target = mDegs;
    PID_state.target_set = 1;
}

int PID_Update() {
    int error;
    volatile PID_State* state;
    PID_Config* pid; 
    error = PID_state.target - GetPosition();
    
    state = &PID_state;
    pid = &PID;
    
    int integral = state->integral;
    integral += error;
    
    //integral clamp
    if (integral >  pid->maxIntegral) integral =  pid->maxIntegral;
    if (integral < -pid->maxIntegral) integral = -pid->maxIntegral;
    
    int derivative = error - state->last_error;
    int32 new_pwm = error*pid->kP/10 + integral*pid->kI/10 + derivative*pid->kD/10;
    state->last_error = error;
    state->integral = integral;
    
    //Max Power clamp
    if(new_pwm > pid->maxPWM){
        new_pwm = pid->maxPWM;
    } else if(new_pwm < -pid->maxPWM) {
        new_pwm = -pid->maxPWM;
    }
    
    return SetPWM(new_pwm);
}

CY_ISR(PID_Handler) {
    if (PID_enable && PID_state.target_set) PID_Update();
}

/* [] END OF FILE */
