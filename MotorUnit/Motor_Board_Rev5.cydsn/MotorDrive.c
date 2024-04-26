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
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include "main.h"
#include "MotorDrive.h"
#include "CAN_Stuff.h"

int16 min_pwm = 20;

uint8  PWM_enable = 0;

int32 PWM_value = 0;

volatile uint8  PWM_invalidate = 0;

volatile int32 position = 0;
volatile int32 enc_value = 0;
volatile int32 pot_value = 0;

Conversion conv1 = {};
Conversion conv2 = {};

uint8 enc_dir = FORWARD;

volatile uint8 limit_status = 0;

uint8 bound_set1;
uint8 bound_set2;
int32 enc_lim_1;
int32 enc_lim_2;

int StartPWM() {
    PWM_enable = 1;
    return 0;
}

void StopPWM(int motor) {
    PWM_enable = 0;
    //Set PWM Motor Compare (i.e (PWM_NAME_WriteCompare(0)))
}

// Sends PWM and Direction to the motor driver
// Also checks limits and sets PWM variables
int SetPWM(int16 pwm) {
    int err = 0;
    if (PWM_enable) {
        PWM_invalidate = 0;
        
        //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        //IMPLEMENT DIR_WRITE() FOR PIN MOTOR
        //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        if (pwm < 0) {
            Pin_Motor2_Dir_Write(BACKWARD);
            // if (pot_value <= 0) {
            //     err = ERROR_LIMIT;
            //     pwm = 0;
            // }
        } else if (pwm > 0) {
            Pin_Motor2_Dir_Write(FORWARD);
            // if (pot_value >= 4095) {
            //     err = ERROR_LIMIT;
            //     pwm = 0;
            // }
        }
        
        if (abs(pwm) < min_pwm) pwm = 0;
        
        PWM_value = pwm;
        PWM_Motor2_WriteCompare(abs(pwm));
    } else err = ERROR_PWM_NOT_ENABLED;
    return err;
}
    
//not fixed
int UpdateConversion(int motor) {
    if (motor & MOTOR1) {
        if (conv1.min_set && conv1.max_set && conv1.mDegMax != conv1.mDegMin) {
            conv1.ratio = (double) (conv1.tickMax-conv1.tickMin)/(conv1.mDegMax-conv1.mDegMin);
            conv1.ratio_set = 1;
        } else {
            return 1;
        }
    }
    if (motor & MOTOR2) {
        if (conv2.min_set && conv2.max_set && conv2.mDegMax != conv2.mDegMin) {
            conv2.ratio = (double) (conv2.tickMax-conv2.tickMin)/(conv2.mDegMax-conv2.mDegMin);
            conv2.ratio_set = 1;
        } else {
            return 1;
        }
    }
    return 0;
}

//not fixed
Conversion GetConversion(int motor) {
    if (motor == MOTOR1) return conv1;
    if (motor == MOTOR2) return conv2;
    return (Conversion) {};
}

//not fixed
uint8 GetConversionReady(int motor) {
    if (motor == MOTOR1) return conv1.ratio_set;
    if (motor == MOTOR2) return conv2.ratio_set;
    return 0;
}

//not fixed
void SetConvRatio(int motor, float ratio) {
    if (motor & MOTOR1) {
        conv1.ratio = ratio;
        conv1.ratio_set = 1;
    }
    if (motor & MOTOR2) {
        conv1.ratio = ratio;
        conv1.ratio_set = 1;
    }
}

//not fixed
int SetConvMin(int motor, int32 tickMin, int32 mDegMin) {
    if (motor & MOTOR1) {
        conv1.tickMin = tickMin;
        conv1.mDegMin = mDegMin;
        conv1.min_set = 1;
    }
    if (motor & MOTOR2) {
        conv2.tickMin = tickMin;
        conv2.mDegMin = mDegMin;
        conv2.min_set = 1;
    }
    return UpdateConversion(motor);
}

//not fixed
void SetConvMax(int motor, int32 tickMax, int32 mDegMax) {
    if (motor & MOTOR1) {
        conv1.tickMax = tickMax;
        conv1.mDegMax = mDegMax;
        conv1.max_set = 1;
    }
    if (motor & MOTOR2) {
        conv2.tickMax = tickMax;
        conv2.mDegMax = mDegMax;
        conv2.max_set = 1;
    }
    UpdateConversion(motor);
}

void SetEncOffset(int32 tick_offset) {
    // QuadDec_Enc_WriteCounter(tick_offset);
}

void SetEncDir(uint8 dir) {
    enc_dir = dir;
}

int UpdateEncValue() {
    uint32 val = 0; // QuadDec_Enc_ReadCounter();
    enc_value = enc_dir ? val : -val;
    return 0;
}

int UpdatePotValue() {
    int32 n = 1000;
    int32 sum = 0;
    for (int i = 0; i < n; i++) {
        ADC_StartConvert();
        ADC_IsEndConversion(ADC_WAIT_FOR_RESULT);
        sum += ADC_GetResult16(0);
    }
    pot_value = sum/n;
    return 0;
}


uint8 GetLimitStatus() { return limit_status; }
int32 GetPotValue() { return pot_value; }
int32 GetEncValue() { return enc_value; }
int32 GetPosition() {
    return position;
}
int32 GetCurrentPWM() {
    return PWM_value;
}

//not fixed
void UpdatePosition(int motor) {
    if (motor & MOTOR1) {
        if (conv1.ratio_set)
            position1 = (enc_value-conv1.tickMin) * conv1.ratio + conv1.mDegMin;
    }
    if (motor & MOTOR2) {
        if (conv2.ratio_set)
            position2 = (pot_value-conv2.tickMin) * conv2.ratio + conv2.mDegMin;
    }
}

//not fixed
void SetEncBound(uint8 lim_num, int32 value) {
    if (lim_num == 0) {
        bound_set1 = 1;
        enc_lim_1 = value;
    } else if (lim_num == 1) {
        bound_set2 = 1;
        enc_lim_2 = value;
    }
}    

//not fixed
CY_ISR(Limit_Handler) {
    limit_status = StatusReg_Limit_Read();
    
    if (limit_status & 0b01) {
        SetPWM(MOTOR1, 0);
        SendLimitAlert(limit_status);
        if (bound_set1) SetEncOffset(MOTOR1, enc_lim_1);
    }
    if (limit_status & 0b10) {
        SetPWM(MOTOR1, 0);
        SendLimitAlert(limit_status);
        if (bound_set2) SetEncOffset(MOTOR1, enc_lim_2);
    }
}

//not fixed
CY_ISR(Drive_Handler) {
    if (PWM1_invalidate == 20) SetPWM(MOTOR1, 0);
    else PWM1_invalidate++;
    
    if (PWM2_invalidate == 20) SetPWM(MOTOR2, 0);
    else PWM2_invalidate++;
    
    UpdatePotValue();
    UpdateEncValue();
    UpdatePosition(MOTOR_BOTH);
}

/* [] END OF FILE */
