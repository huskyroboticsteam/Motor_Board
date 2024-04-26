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
#include <stdint.h>

typedef struct PID_config {
    uint8 kP_set, kI_set, kD_set;
    int32 kP, kI, kD, maxPWM, maxIntegral;
} PID_Config;

typedef struct PID_state {
    int8 target_set;
    int32 target;
    int32 integral;
    int32 last_error;
} PID_State;

int StartPID();
void StopPID();
void SetkPosition(int32 kP);
void SetkIntegral(int32 kI);
void SetkDerivative(int32 kD);
void SetPIDMaxPWM(uint16 maxPWM);
void SetPIDTarget(int32 mDegs);
PID_State GetPIDState();
PID_Config GetPIDConfig();

CY_ISR(PID_Handler);

/* [] END OF FILE */
