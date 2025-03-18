/*
 * BLDC_PID.h
 *
 *  Created on: Jan 14, 2025
 *      Author: eunbecha
 */

#ifndef INC_BLDC_PID_H_
#define INC_BLDC_PID_H_

#include "main.h"
#include <stdint.h>

void SetPosition(int32_t mDegs) ;
int32_t MiliDegreesToTicks(int32_t miliDegrees);
int32_t Position_PID(int32_t targetmDeg);
int32_t GetPositionmDeg();

void SetkPosition(int32_t kP);
void SetkIntegral(int32_t kI);
void SetkDerivative(int32_t kD);
void SetConversion(double conv);
double UpdateConversion();
// void SetEncoderDir(uint8_t flip);
// void setEncoderAtLimit(int enc_limit);

//used for mode change
void ClearPIDProgress();
void InitializePID();

void DisablePID();
void EnablePID();
uint8_t PIDIsEnabled();

void SetEncoderDirDefault();
void SetEncoderDirReverse();
void SetMaxPIDPWM(uint16_t setValue);
void setUsingPot(uint8_t pot);
void setTickMin(int32_t val);
void setTickMax(int32_t val);
void setmDegMin(int32_t val);
void setmDegMax(int32_t val);

int32_t GetMaxPIDPWM();
int32_t GetkPosition();
int32_t GetkIntegral();
int32_t GetkDerivative();
double GetConversion();
int32_t GetTickMax();
int32_t GetTickMin();
int32_t GetmDegMax();
int32_t GetmDegMin();

#endif /* INC_BLDC_PID_H_ */
