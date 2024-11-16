#include "main.h"
#include <stm32f3xx_hal_can.h>
#include <CANPacket.h>

// Define CAN baud rates
#define CAN_1000_BAUD 1000000
#define CAN_500_BAUD 500000
#define CAN_250_BAUD 250000
#define CAN_125_BAUD 125000
#define CAN_100_BAUD 100000

// Globals for device identification
uint8_t devGrp = 0; // Device group
uint8_t devSer = 0; // Device serial

// CAN handle
CAN_HandleTypeDef hcan;

/**
 * @brief Configures the CAN filter.
 */
void CAN_FilterConfig(void) {
    CAN_FilterTypeDef sFilterConfig;

    sFilterConfig.FilterBank = 0; // Use filter bank 0
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK; // Identifier mask mode
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT; // 32-bit scale
    sFilterConfig.FilterIdHigh = 0x0000; // Identifier high bits
    sFilterConfig.FilterIdLow = 0x0000; // Identifier low bits
    sFilterConfig.FilterMaskIdHigh = 0x0000; // Mask high bits
    sFilterConfig.FilterMaskIdLow = 0x0000; // Mask low bits
    sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0; // Assign to FIFO 0
    sFilterConfig.FilterActivation = ENABLE; // Enable the filter

    if (HAL_CAN_ConfigFilter(&hcan, &sFilterConfig) != HAL_OK) {
        // Filter configuration error
        Error_Handler();
    }
}

/**
 * @brief Initializes the CAN peripheral.
 * @param baudRate Baud rate for CAN communication
 * @param deviceGroup Device group identifier
 * @param deviceSerial Device serial number
 */
void InitCAN(uint32_t baudRate, uint16_t deviceGroup, uint16_t deviceSerial) {
    // Save device group and serial
    devGrp = deviceGroup;
    devSer = deviceSerial;

    // Initialize the CAN handle
    hcan.Instance = CAN1;
    hcan.Init.Prescaler = HAL_RCC_GetPCLK1Freq() / (baudRate * 16);
    hcan.Init.Mode = CAN_MODE_NORMAL; // Normal mode
    hcan.Init.SyncJumpWidth = CAN_SJW_1TQ; // Synchronization Jump Width
    hcan.Init.TimeSeg1 = CAN_BS1_13TQ; // Time Segment 1
    hcan.Init.TimeSeg2 = CAN_BS2_2TQ; // Time Segment 2
    hcan.Init.TimeTriggeredMode = DISABLE;
    hcan.Init.AutoBusOff = ENABLE;
    hcan.Init.AutoWakeUp = DISABLE;
    hcan.Init.AutoRetransmission = ENABLE;
    hcan.Init.ReceiveFifoLocked = DISABLE;
    hcan.Init.TransmitFifoPriority = DISABLE;

    if (HAL_CAN_Init(&hcan) != HAL_OK) {
        // Initialization error
        Error_Handler();
    }

    // Configure CAN filter
    CAN_FilterConfig();

    // Start CAN communication
    if (HAL_CAN_Start(&hcan) != HAL_OK) {
        // Start error
        Error_Handler();
    }

    // Enable CAN RX interrupts
    if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) {
        // Notification activation error
        Error_Handler();
    }
}

/**
 * @brief Sends a CAN packet.
 * @param packetToSend Pointer to the packet to send
 * @return Status of the transmission (0 for success, 0x02 for no mailbox available)
 */
int SendCANPacket(CANPacket *packetToSend) {
    CAN_TxHeaderTypeDef TxHeader;
    uint32_t TxMailbox;
    uint8_t data[8];

    TxHeader.StdId = packetToSend->id; // Standard identifier
    TxHeader.ExtId = 0; // Extended identifier (not used)
    TxHeader.RTR = CAN_RTR_DATA; // Data frame
    TxHeader.IDE = CAN_ID_STD; // Standard ID
    TxHeader.DLC = packetToSend->dlc; // Data length code
    TxHeader.TransmitGlobalTime = DISABLE;

    // Copy data to transmission buffer
    for (uint8_t i = 0; i < packetToSend->dlc; i++) {
        data[i] = packetToSend->data[i];
    }

    if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, data, &TxMailbox) != HAL_OK) {
        // Transmission error
        return 0x02; // No Tx Mailbox available
    }

    return 0; // Success
}

/**
 * @brief Polls and receives a CAN packet.
 * @param receivedPacket Pointer to the packet to store received data
 * @return Status of the reception (0 for success, 0x01 if no message, 0x02 for error)
 */
int PollAndReceiveCANPacket(CANPacket *receivedPacket) {
    CAN_RxHeaderTypeDef RxHeader;
    uint8_t data[8];

    if (HAL_CAN_GetRxFifoFillLevel(&hcan, CAN_RX_FIFO0) == 0) {
        // No message available
        return 0x01;
    }

    if (HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO0, &RxHeader, data) != HAL_OK) {
        // Reception error
        return 0x02;
    }

    // Fill the received packet structure
    receivedPacket->id = RxHeader.StdId;
    receivedPacket->dlc = RxHeader.DLC;
    for (uint8_t i = 0; i < RxHeader.DLC; i++) {
        receivedPacket->data[i] = data[i];
    }

    return 0; // Success
}

/**
 * @brief Retrieves the local device group.
 * @return The device group identifier
 */
uint8_t getLocalDeviceGroup() {
    return devGrp;
}

/**
 * @brief Retrieves the local device serial.
 * @return The device serial identifier
 */
uint8_t getLocalDeviceSerial() {
    return devSer;
}
