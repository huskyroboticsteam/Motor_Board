/*
 * BLDC_Drive.h
 *
 *  Created on: Jan 11, 2025
 *      Author: eunbecha
 */

#ifndef INC_BLDC_DRIVE_H_
#define INC_BLDC_DRIVE_H_

#include <stdio.h>
#ifndef MotorDrive
    #define MotorDrive
#endif

// State of U, V, and W inductors (H-L)
struct CoilState
{
    uint8_t UH : 1;
    uint8_t UL : 1;
    uint8_t VH : 1;
    uint8_t VL : 1;
    uint8_t WH : 1;
    uint8_t WL : 1;
};

// State of U, V, and W inductors (H-L)
struct HallState
{
    uint8_t A : 1;
    uint8_t B : 1;
    uint8_t C : 1;
};

void set_speed(int16_t speed, uint8_t disable_limit);
int16_t GetCurrentSpeed();
void updateRotorState();
void commutateMotor(uint8_t hallState);

#endif /* INC_BLDC_DRIVE_H_ */
