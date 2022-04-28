/**
 * @file can_adapter.cpp
 * 
 * @author Joseph Telaak
 * 
 * @brief CAN adapter control
 * 
 * @version 0.1
 * 
 * @date 2022-04-26
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "can_adapter.h"

// --------- CAN

/** @brief Setup the CAN transceiver */
void CAN_Adapter::setupCAN(int CS_PIN, uint32_t id) {
    #ifdef DEBUG
        // Log init
        Serial.println("CAN Transceiver: Init Starting");
    #endif

    // Init
    can_module = new MCP2515(CS_PIN);

    // Reset and set
    can_module -> reset();
    can_module -> setBitrate(CAN_125KBPS);
    can_module -> setNormalMode();

    // ID
    m_can_id = id;

    #ifdef DEBUG
        // Log done
        Serial.println("CAN Transceiver: Done");
    #endif

}

/**
 * @brief Get the CAN message if the id is correct
 * 
 * @return true If message is valid and id's match
 * @return false If message is invalid or id's do not match
 */

bool CAN_Adapter::getCANMessage() {
    if (can_module -> readMessage(&can_msg_in) == MCP2515::ERROR_OK) {
        if (can_msg_in.can_id == m_can_id) {
            #ifdef DEBUG
                printReceivedCANMessage();
            #endif

            return true;

        } 
    }

    return false;

}

/**
 * @brief Send a message of the can bus
 * 
 * @param id ID of the CAN device to send message to
 * @param m_data Data to send to the CAN device
 */

void CAN_Adapter::sendCANMessage(uint32_t id, uint8_t m_data[8]) {
    // Assign ID
    can_msg_out.can_id = id;

    // Assign dlc
    can_msg_out.can_dlc = m_can_dlc;

    // Map data
    for (int i = 0; i < m_can_dlc; i++) {
        can_msg_out.data[i] = m_data[i];

    }

    #ifdef DEBUG
        printOutgoingCANMessage();
    
    #endif

    // Send message
    can_module -> sendMessage(&can_msg_out);

}

/**
 * @brief Send a message of the can bus
 * 
 * @param id ID of the CAN device to send message to
 * @param m_data Data to send to the CAN device
 */

void CAN_Adapter::sendCANMessage() {
    #ifdef DEBUG
        printOutgoingCANMessage();
    
    #endif

    // Send message
    can_module -> sendMessage(&can_msg_out);

}

#ifdef DEBUG

    /** @brief Print out the outgoing message */
    void CAN_Adapter::printOutgoingCANMessage() {
        // Start log
        Serial.print("CAN-TX: (" + String(can_msg_out.can_id) + ") ");

        // Print data
        for (int i = 0; i < can_msg_out.can_dlc; i++) {
            Serial.print(String(can_msg_out.data[i]) + " ");

        }

        // New Line
        Serial.println();

    }

    /** @brief Print out the received can frame*/
    void CAN_Adapter::printReceivedCANMessage() {
        // Start log
        Serial.print("CAN-RX: (" + String(can_msg_in.can_id) + ") ");

        // Print data
        for (int i = 0; i < can_msg_in.can_dlc; i++) {
            Serial.print(String(can_msg_in.data[i]) + " ");

        }

        // New Line
        Serial.println();

    }

#endif

/**
 * @brief Set the new can address in eeprom
 * 
 * @param new_can_addr New can address
 */

void CAN_Adapter::setCANAddress(uint32_t new_can_addr) {  m_can_id = new_can_addr; }

/**
 * @brief Check the condition and use to prepare a CAN message
 * 
 * @param condition Condition to check
 * 
 * @return uint8_t (0x01 - True) or (0x02 - False)
 */

uint8_t getCANBoolean(bool condition) { return condition ? 0x01 : 0x02; }

/** @brief Convert an 8 bit integer into a standard integer  */
int convertToInt(uint8_t incoming_int) {
    int new_int = incoming_int << 8;

    if (new_int < 0) { new_int = new_int * -1; }
    if (new_int > 255) { new_int = 255; }

    return new_int;

}

/** @brief Convert two 8 bit integers into a standard integer  */
int convertToInt(uint8_t int_1, uint8_t int_2) {
    int new_int = (int_1 << 8) | int_2;

}