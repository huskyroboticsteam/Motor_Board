#ifndef MOTORDRIVE_H
#define MOTORDRIVE_H

#include <stdio.h>

void set_PWM(int16_t compare, uint8_t disable_limit, uint8_t limitSW);

int16_t GetCurrentPWM();

#endif // MOTORDRIVE_H