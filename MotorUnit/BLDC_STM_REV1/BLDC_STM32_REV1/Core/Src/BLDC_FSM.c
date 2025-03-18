/*
 * BLDC_FSM.c
 *
 *  Created on: Jan 11, 2025
 *      Author: eunbecha
 */

#include "BLDC_FSM.h"
#include "main.h"

/* Control FSM */

#include "BLDC_FSM.h"
#include "BLDC_Drive.h"

/* Drive mode
0xFF = un-init
0x0 = PWM
0x1 = PID */
uint8_t motorUnitMode   = 0xFF;
uint8_t motorUnitState  = UNINIT; // Initial motor state
uint8_t PIDConstSetReg  = 0;

// GotoUninitState: Halts the motor and clears PID progress
void GotoUninitState() {
    // Halt motor (specific motor halt implementation required)
    ClearPIDProgress(); // Clear PID internal progress/state
    motorUnitMode = 0xFF;
    motorUnitState = UNINIT;
}

// Set the state of the motor
void SetStateTo(uint8_t state) {
    motorUnitState = state;
}

// Set the mode of the motor
void SetModeTo(uint8_t mode) {
    motorUnitMode = mode;
}

// Get the current state of the motor
uint8_t GetState() {
    return motorUnitState;
}

// Get the current mode of the motor
uint8_t GetMode() {
    return motorUnitMode;
}

// The following functions set PID constants (flags in a register)
void PositionConstIsSet() {
    PIDConstSetReg |= 0b100;
}

void IntegralConstIsSet() {
    PIDConstSetReg |= 0b10;
}

void DerivativeConstIsSet() {
    PIDConstSetReg |= 0b1;
}

void PPJRConstIsSet() {
    PIDConstSetReg |= 0b1000;
}

void MaxJointRevIsSet() {
    // Implementation for max joint revolution constant setting (if needed)
}

// Check if all PID constants are set
uint8_t PIDconstsSet() {
    return PIDConstSetReg == 0b1111;
}

// Clear all PID constants
void ClearPIDconst() {
    PIDConstSetReg = 0;
}
