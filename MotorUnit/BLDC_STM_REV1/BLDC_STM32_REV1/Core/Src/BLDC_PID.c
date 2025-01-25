/*
 * BLDC_PID.c
 *
 *  Created on: Jan 14, 2025
 *      Author: eunbecha
 */

#include "BLDC_PID.h"

int32_t delayTime = 1000;
int32_t kPosition = 0, kIntegral = 0, kDerivative = 0;
int32_t tickMax = 0, tickMin = 0, mDegMax = 0, mDegMin = 0;
double mDegPerTick = 0.0;

int integral = 0;     // Needs to be reset upon mode change
int lastError = 0;    // Needs to be reset upon mode change
int integralClamp = 5000;

uint8_t enabledPID = 0;
uint32_t maxSpeed = 32768;

extern UART_HandleTypeDef huart1; // Assuming UART1 is used

void ClearPIDProgress() {
    integral = 0;
    lastError = 0;
}

void DisablePID() {
    enabledPID = 0;
}

void EnablePID() {
    enabledPID = 1;
}

uint8_t PIDIsEnabled() {
    return enabledPID;
}

void InitializePID() {
    ClearPIDProgress();
    DisablePID();
    lastError = 0;
}

double UpdateConversion() {
    if (mDegMin == mDegMax) return 0;
    mDegPerTick = (double)(mDegMax - mDegMin) / (tickMax - tickMin);
    return mDegPerTick;
}


int32_t GetPositionmDeg() {
    if (mDegPerTick == 0.0)
        return 0;
    // TODO: Implement encoder reading and position conversion
    return 0;
}

void SetPosition(int32_t mDegs) {
    int32_t speed = Position_PID(mDegs);
    // TODO: Implement set_speed(speed, ignoreLimSw);
}

int32_t Position_PID(int32_t targetmDeg) {
    if (!PIDIsEnabled()) {
        return 0;
    }

    int32_t current = GetPositionmDeg();
    int32_t error = targetmDeg - current;

    if (error <= 5 && error >= -5) {
        return 0;
    }

    integral += error;

    if (integral > integralClamp) {
        integral = integralClamp;
    }
    if (integral < -integralClamp) {
        integral = -integralClamp;
    }

    int derivative = error - lastError;
    int PWMOut = error * kPosition / 10 + integral * kIntegral / 10 + derivative * kDerivative / 10;
    lastError = error;

    char txData[100];
    snprintf(txData, sizeof(txData), "c:%ld, P:%ld, I:%d, D:%d, Out:%d\n\r", current, error, integral, derivative, PWMOut);
    HAL_UART_Transmit(&huart1, (uint8_t *)txData, strlen(txData), HAL_MAX_DELAY);

    return PWMOut;
}
