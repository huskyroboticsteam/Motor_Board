/*
 * BLDC_SPI.h
 *
 *  Created on: Jan 11, 2025
 *      Author: eunbecha
 */

#ifndef INC_BLDC_SPI_H_
#define INC_BLDC_SPI_H_

#include "main.h"
extern SPI_HandleTypeDef hspi1;

uint32_t read_spi(uint8_t address, uint32_t dataToSend);


#endif /* INC_BLDC_SPI_H_ */
