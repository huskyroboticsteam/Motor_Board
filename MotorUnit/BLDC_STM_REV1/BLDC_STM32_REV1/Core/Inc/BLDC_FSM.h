/*
 * BLDC_FSM.h
 *
 *  Created on: Jan 11, 2025
 *      Author: eunbecha
 */

#ifndef INC_BLDC_FSM_H_
#define INC_BLDC_FSM_H_

#include <stdint.h>

#define UNINIT          0xFF
#define SET_PWM         0x0
#define CALC_PID        0x1
#define CHECK_CAN       0x2
#define QUEUE_ERROR     0x3

void GotoUninitState(void);
void SetStateTo(uint8_t state);
void SetModeTo(uint8_t mode);
uint8_t GetState(void);
uint8_t GetMode(void);
void PositionConstIsSet(void);
void IntegralConstIsSet(void);
void DerivativeConstIsSet(void);
void PPJRConstIsSet(void);
void MaxJointRevIsSet(void);
uint8_t PIDconstsSet(void);
void ClearPIDconst(void);



#endif /* INC_BLDC_FSM_H_ */
