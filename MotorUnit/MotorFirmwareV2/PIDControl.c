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

#include <project.h>
#include "PIDControl.h"
#include "MotorDrive.h"
#include "FSM_Stuff.h"

PID_Config PID = {.maxIntegral=-1, .maxPWM=1023};
volatile PID_State PID_state = {};

uint8 PID_enable;
uint8 ctrl_type;

// starts PID control for position (0) or velocity (1)
int StartPID(int control_type) {
    int err = 0;
    
    if (PID.kP_set && PID.kI_set && PID.kD_set && GetConversion().ratio_set) {
        PID_state.integral = 0;
        PID_state.last_error = 0;
        PID_enable = 1;
        ctrl_type = control_type;
    } else err = 1;
    
    return err;
}

void StopPID() {
    PID_enable = 0;
    PID_state.target_set = 0;
    SetPWM(0);
}

void SetkProportional(int32 kP) {
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

void SetPIDMaxPWM(uint16 maxPWM) {
    PID.maxPWM = maxPWM;
}

PID_Config GetPIDConfig() {
    return PID;
}
PID_State GetPIDState() {
    return PID_state;
}


void SetPIDTarget(int32 target) {
    PID_state.target = target;
    PID_state.target_set = 1;
}

int PID_Update() {
    int error;
    volatile PID_State* state;
    PID_Config* pid; 

    if (ctrl_type == 0) {
        error = PID_state.target - GetPosition();
    } else {
        error = PID_state.target - GetVelocity();
    }
    
    int integral = PID_state.integral;
    integral += error;
    
    //integral clamp
    if (integral >  PID.maxIntegral) integral =  PID.maxIntegral;
    if (integral < -PID.maxIntegral) integral = -PID.maxIntegral;
    
    int derivative = error - PID_state.last_error;
    int32 new_pwm = error*PID.kP/10 + integral*PID.kI/10 + derivative*PID.kD/10;
    PID_state.last_error = error;
    PID_state.integral = integral;
    
    //Max Power clamp
    if(new_pwm > PID.maxPWM){
        new_pwm = PID.maxPWM;
    } else if(new_pwm < -PID.maxPWM) {
        new_pwm = -PID.maxPWM;
    }
    
    return SetPWM(new_pwm);
}

CY_ISR(PID_Handler) {
    if (PID_enable && PID_state.target_set) PID_Update();
}

/* [] END OF FILE */
