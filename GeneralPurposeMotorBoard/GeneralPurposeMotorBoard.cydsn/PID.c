#include "PID.h"
#include "MotorDrive.h"
#include "main.h"
#include <project.h>
#include <math.h>

int32_t kPosition = 0;
int32_t kIntegral = 0;
int32_t kDerivative = 0;

int32_t kPPJR = 0;

int32_t PWM = 0;
extern uint8_t ignoreLimSw;

extern char txData[TX_DATA_SIZE];
int8 flipEncoder = 1;
uint8_t usingPot = 0;

// integral and lastPosition need to be reset upon mode change
int integral = 0;
int lastPosition = 0;
int integralClamp = 500;

double ratio;
uint8 complete = 0;
int32 maxPWM = 32767;

uint8_t enabledPID = 0;

void ClearPIDProgress() {
    integral = 0;
    lastPosition = 0;
}

void DisablePID() {
    enabledPID = 0;
}

void EnablePID() {
    enabledPID = 1;
}

uint8_t PIDIsEnabled() {
    return(enabledPID);
}

void InitializePID() {
    set_PWM(0, 0, 0);
    ClearPIDProgress();
    DisablePID();
    lastPosition = GetEncoderValWithFlip();
}

void SetkPosition(int32_t kP) {
    kPosition = kP;
}

void SetkIntegral(int32_t kI) {
    kIntegral = kI;
}

void SetkDerivative(int32_t kD) {
    kDerivative = kD;
}
void SetkPPJR(uint32_t kppjr) {
    kPPJR = kppjr;
}

int32_t GetkPosition() {
    return kPosition;
}

int32_t GetkIntegral() {
    return kIntegral;
}

int32_t GetkDerivative() {
    return kDerivative;
}

uint32_t GetkPPJR() {
    return kPPJR;
}

void SetMaxPIDPWM(uint16_t setValue) {
    maxPWM = setValue;
}

int32_t GetMaxPIDPWM() {
    return maxPWM;
}

int32_t GetEncoderValWithFlip() {
    return flipEncoder * QuadDec_GetCounter();
}

void SetEncoderDirDefault() {
    flipEncoder = 1;
}

void SetEncoderDirReverse() {
    flipEncoder = -1;
}

int32_t CurrentPositionMiliDegree() {
    if(kPPJR == 0){
        return(0);
}            
    return  (int)round((360*1000.0) / kPPJR * GetEncoderValWithFlip());
}

void SetPosition(int32 miliDegrees) {
        //TODO: Make Potentiometer Compatible
        PWM = Position_PID(MiliDegreesToTicks(miliDegrees));
        
        //Max Power clamp
        if(PWM > GetMaxPIDPWM()){
            set_PWM(GetMaxPIDPWM(), ignoreLimSw, Status_Reg_Switches_Read());   
        } else if(PWM < -GetMaxPIDPWM()) {
            set_PWM(-GetMaxPIDPWM(), ignoreLimSw, Status_Reg_Switches_Read());
        } else {
            set_PWM(PWM, ignoreLimSw, Status_Reg_Switches_Read());   
        }
}

int32_t MiliDegreesToTicks(int32_t miliDegrees) {
    int32_t ticks = miliDegrees * kPPJR/(360*1000);// make float
    return(ticks);
}

int32_t Position_PID(int32 targetTick) {
    if(!PIDIsEnabled()){
        return(0);
    }
    
    //TODO: Make Potentoimeter Compatible
    volatile int32 current =  GetEncoderValWithFlip();
    int32 position = targetTick - current;
    
    // if within tolerance exit
    if(position <= 5 && position >= -5) {
      return(0);
    }
    
    integral = integral + position;
    
    //integral clamp
    if (integral > integralClamp) {
        integral = integralClamp;   
    }
    if (integral < -integralClamp) {
        integral = -integralClamp;
    }
    
    int derivative = position - lastPosition;
    int PWMOut = position*kPosition/10 + integral*kIntegral/10 + derivative*kDerivative/10;
    lastPosition = position;
    
    #ifdef PRINT_PID_DEBUG
        sprintf(txData,"c:%d, P:%d, I%d, D:%d, Out:%d", current, position, integral, derivative, PWMOut);
        UART_UartPutString(txData);   
    #endif
 
    return (PWMOut);
}