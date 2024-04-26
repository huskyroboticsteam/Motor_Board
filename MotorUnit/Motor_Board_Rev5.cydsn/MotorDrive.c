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

Conversion conv = {};

uint8 enc_dir = FORWARD;

volatile uint8 limit_status = 0;
uint8 using_pot = 0;

uint8 bound_set1;
uint8 bound_set2;
int32 enc_lim_1;
int32 enc_lim_2;

int StartPWM() {
    PWM_enable = 1;
    return 0;
}

void StopPWM() {
    PWM_enable = 0;
    PWM_Motor_WriteCompare(0);
}

// Sends PWM and Direction to the motor driver
// Also checks limits and sets PWM variables
int SetPWM(int16 pwm) {
    int err = 0;
    if (PWM_enable) {
        PWM_invalidate = 0;
        
        if (pwm < 0) {
            Pin_Motor_Dir_Write(BACKWARD);
            // if (pot_value <= 0) {
            //     err = ERROR_LIMIT;
            //     pwm = 0;
            // }
        } else if (pwm > 0) {
            Pin_Motor_Dir_Write(FORWARD);
            // if (pot_value >= 4095) {
            //     err = ERROR_LIMIT;
            //     pwm = 0;
            // }
        }
        
        if (abs(pwm) < min_pwm) pwm = 0;
        
        PWM_value = pwm;
        PWM_Motor_WriteCompare(abs(pwm));
    } else err = ERROR_PWM_NOT_ENABLED;
    
    return err;
}

int UpdateConversion() {
    if (conv.min_set && conv.max_set && conv.mDegMax != conv.mDegMin) {
        conv.ratio = (double) (conv.tickMax-conv.tickMin)/(conv.mDegMax-conv.mDegMin);
        conv.ratio_set = 1;
    } else {
        return 1;
    }
    
    return 0;
}

Conversion GetConversion() { return conv; }

void SetConvRatio(float ratio) {
    conv.ratio = ratio;
    conv.ratio_set = 1;
}

int SetConvMin(int32 tickMin, int32 mDegMin) {
    conv.tickMin = tickMin;
    conv.mDegMin = mDegMin;
    conv.min_set = 1;
    
    return UpdateConversion();
}

void SetConvMax(int32 tickMax, int32 mDegMax) {
    conv.tickMax = tickMax;
    conv.mDegMax = mDegMax;
    conv.max_set = 1;
    UpdateConversion();
}

void SetEncOffset(int32 tick_offset) {
    QuadDec_Enc_WriteCounter(tick_offset);
}

void SetEncDir(uint8 dir) {
    enc_dir = dir;
}

int UpdateEncValue() {
    uint32 val = QuadDec_Enc_ReadCounter();
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

int32 GetPotValue() { return pot_value; }
int32 GetEncValue() { return enc_value; }
int32 GetPosition() { return position; }
int32 GetCurrentPWM() { return PWM_value; }
uint8 GetLimitStatus() { return limit_status; }

void UpdatePosition() {
    if (conv.ratio_set) {
        if (using_pot) {  
            position = (enc_value-conv.tickMin) * conv.ratio + conv.mDegMin;
        } else {
            position = (pot_value-conv.tickMin) * conv.ratio + conv.mDegMin;
        }
    }
}

void SetUsingPot(uint8_t pot) { using_pot = pot; }

void SetEncBound(uint8 lim_num, int32 value) {
    if (lim_num == 0) {
        bound_set1 = 1;
        enc_lim_1 = value;
    } else if (lim_num == 1) {
        bound_set2 = 1;
        enc_lim_2 = value;
    }
}    

CY_ISR(Limit_Handler) {
    limit_status = StatusReg_Limit_Read();
    
    if (limit_status & 0b01) {
        SetPWM(0);
        SendLimitAlert(limit_status);
        if (bound_set1) SetEncOffset(enc_lim_1);
    }
    if (limit_status & 0b10) {
        SetPWM(0);
        SendLimitAlert(limit_status);
        if (bound_set2) SetEncOffset(enc_lim_2);
    }
}

CY_ISR(Drive_Handler) {
    if (PWM_invalidate == 20) SetPWM(0);
    else PWM_invalidate++;
    
    if (using_pot) UpdatePotValue();
    else UpdateEncValue();
    UpdatePosition();
}

/* [] END OF FILE */
