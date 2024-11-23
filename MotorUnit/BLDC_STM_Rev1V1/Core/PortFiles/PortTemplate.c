///*
// * Documentation: https://huskyroboticsteam.slite.com/app/channels/iU0BryG7M9/collections/aXvWTcIR6c/notes/4otlSFsSp2
// */
//
//#if CHIP_TYPE == CHIP_TYPE_TEMPLATE // Replace with the target chip type
//
//#include "../Inc/Port.h"
//
///**
// * @brief Initializes the CAN communication for the given device group and address.
// * @param deviceGroup The device group ID.
// * @param deviceAddress The device address ID.
// */
//void InitCAN(int deviceGroup, int deviceAddress) {
//    // TODO: Implement hardware initialization for CAN communication
//    // This should include configuring receive filters and other hardware-specific settings
//    // Example:
//    // - Configure receive mailboxes
//    // - Set up filtering for CAN messages based on the device group and address
//}
//
///**
// * @brief Sends a CAN packet.
// * @param packetToSend Pointer to the CAN packet to be sent.
// * @return Status code indicating success or failure.
// */
//int SendCANPacket(CANPacket *packetToSend) {
//    // TODO: Implement CAN packet sending and queuing
//    // This could involve filling a CAN transmit buffer and triggering a transmission
//    // Example:
//    // - Prepare the CAN packet structure (ID, data, etc.)
//    // - Queue the packet for transmission
//    // - Trigger the CAN controller to send the packet
//}
//
///**
// * @brief Polls and receives a CAN packet from the receive buffer.
// * @param receivedPacket Pointer to the structure where the received packet will be stored.
// * @return Status code indicating success or failure (e.g., no message, error).
// */
//int PollAndReceiveCANPacket(CANPacket *receivedPacket) {
//    // TODO: Implement CAN packet reception from the receive buffer or registers
//    // This will likely involve checking the CAN controller's status and retrieving received data
//    // Example:
//    // - Check if a new CAN message is available
//    // - If available, copy the received message into the provided structure
//    // - Return success or failure
//}
//
///**
// * @brief Gets the local device's serial number.
// * @return Device serial number. This might be set via DIP switches or hardcoded.
// */
//uint8_t getLocalDeviceSerial() {
//    // The serial number may be board-specific or hardcoded
//    // Example:
//    // - Read from DIP switches or other onboard settings
//    return DEVICE_SERIAL_MOTOR_CHASSIS_FR; // Example value (can be board-specific)
//}
//
///**
// * @brief Gets the local device's group ID.
// * @return Device group ID.
// */
//uint8_t getLocalDeviceGroup() {
//    // The device group might be board-specific and can vary
//    // Example:
//    // - Set the group ID based on the board or configuration
//    return DEVICE_GROUP_MOTOR_CONTROL; // Example value (can be board-specific)
//}
//
///**
// * @brief Gets the chip type for the current port.
// * @return Chip type.
// */
//uint8_t getChipType() {
//    //return CHIP_TYPE; // Should be consistent across all ports for a given chip
//    // This value can be used for debugging or identifying the target chip
//}
//
//#endif // CHIP_TYPE == CHIP_TYPE_TEMPLATE
