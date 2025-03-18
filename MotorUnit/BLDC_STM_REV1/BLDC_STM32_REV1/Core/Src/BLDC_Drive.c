/*
 * BLDC_Drive.c
 *
 *  Created on: Jan 11, 2025
 *      Author: eunbecha
 */

#include <stdint.h>
#include <stdio.h>
#include "main.h"
#include "BLDC_Drive.h"
extern TIM_HandleTypeDef htim1;

// Constants
#define MAX_RPM 1000

// Rotor state definitions
uint8_t rotorStates[6] = {0b001001,
                          0b011000,
                          0b010010,
                          0b000110,
                          0b100100,
                          0b100001};

// Global variables
int32_t currentSpeed = 0;
int8_t currentDir = 0;
int8_t currentRotorState = 0;
float delay_ms = 0;

// Placeholder for limit switch state, replace with actual GPIO reads
uint8_t ReadLimitSwitch() {
    // Replace with GPIO input logic for limit switches
    return 0;
}

// Function to set motor speed and direction
void set_speed(int16_t speed, uint8_t disable_limit) {
    uint8_t limitSW = disable_limit ? 0 : ReadLimitSwitch();

    if (speed < 0 && !(limitSW & 0b10)) {
        currentDir = 0;
        currentSpeed = -speed;
    } else if (speed > 0 && !(limitSW & 0b01)) {
        currentDir = 1;
        currentSpeed = speed;
    } else {
        currentSpeed = 0;
    }

    if (currentSpeed != 0) {
        delay_ms = (float)(1 << 15) * 10000 / currentSpeed / MAX_RPM;
        // Start the rotor delay timer
        __HAL_TIM_SET_AUTORELOAD(&htim1, (uint32_t)delay_ms);
        HAL_TIM_Base_Start_IT(&htim1);
    } else {
        delay_ms = 0;
        // Stop the motor
        UH_Write(0);
        UL_Write(0);
        VH_Write(0);
        VL_Write(0);
        WH_Write(0);
        WL_Write(0);
        HAL_TIM_Base_Stop_IT(&htim1);
    }
}

// Function to get the current speed
int16_t GetCurrentSpeed() {
    return currentSpeed;
}

// Function to update rotor state
void updateRotorState() {
    currentRotorState += currentDir;

    if (currentRotorState <= -1) {
        currentRotorState = 5;
    } else if (currentRotorState >= 6) {
        currentRotorState = 0;
    }

    uint8_t state = rotorStates[currentRotorState];

    // Write to GPIOs to energize motor phases
    UH_Write((state & (1 << 0)) != 0);
    UL_Write((state & (1 << 1)) != 0);
    VH_Write((state & (1 << 2)) != 0);
    VL_Write((state & (1 << 3)) != 0);
    WH_Write((state & (1 << 4)) != 0);
    WL_Write((state & (1 << 5)) != 0);
}

// Interrupt callback for the timer (HAL)
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM1) { // Replace TIM1 with the timer you're using
        updateRotorState();
    }
}
