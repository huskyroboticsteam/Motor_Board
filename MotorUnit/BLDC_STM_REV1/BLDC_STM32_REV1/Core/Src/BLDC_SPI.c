/*
 * BLDC_SPI.c
 *
 *  Created on: Jan 11, 2025
 *      Author: eunbecha
 */

#include "BLDC_SPI.h"

// read fault
uint32_t read_spi(uint8_t address, uint32_t dataToSend) {
    uint8_t txData[5]; // Buffer for transmitted data
    uint8_t rxData[4]; // Buffer for received data
    uint32_t result = 0;

    // Prepare the data to send: address and data
    txData[0] = address;
    txData[1] = (dataToSend & 0xFF000000) >> 24; // Send the MSB first
    txData[2] = (dataToSend & 0x00FF0000) >> 16;
    txData[3] = (dataToSend & 0x0000FF00) >> 8;
    txData[4] = (dataToSend & 0x000000FF); // Send the LSB last

    // Perform SPI transmission and reception simultaneously
    if (HAL_SPI_TransmitReceive(&hspi1, txData, rxData, 5, HAL_MAX_DELAY) != HAL_OK) {
        // Error handling
        Error_Handler();
    }

    // Combine the received bytes into a 32-bit result
    result |= (rxData[0] << 24);
    result |= (rxData[1] << 16);
    result |= (rxData[2] << 8);
    result |= rxData[3];

    return result;
}

