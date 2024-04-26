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

#include "FSM_Stuff.h"
#include "MotorDrive.h"
#include "project.h"
#include "PositionPID.h"

int mode

int SetMode(int new_mode) {
    int err = 0;
    
    if (new_mode == MODE_UNINIT) {
        StopPID();
        StopPWM();
    } else if (new_mode == MODE_PWM_CTRL) {
        StopPID();
        err = StartPWM();
    } else if (new_mode == MODE_PID_CTRL) {
        err = StartPID();
        if (!err)
            err = StartPWM();
    }
    
    if (err) return ERROR_MODE_CHANGE;
    mode = new_mode;
    return 0;
}

int GetMode(int motor) {
    return mode;
}

/* [] END OF FILE */