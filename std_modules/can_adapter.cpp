#include "can_adapter.h"

// --------- CAN

/** @brief Setup the CAN transceiver */
void setupCAN(int CS_PIN) {
    // Get address from eeprom
    Serial.println("CAN Transceiver: Loading CAN Address");
    
    if (m_can_id == 0x000) {
        uint32_t new_addr = getCANAddress();

        if (new_addr == 0x000) {


        } else {
            m_can_id = new_addr;
            
        }
    }

    // Log init
    Serial.println("CAN Transceiver: Init Starting");

    // Init
    mcp2515 = new MCP2515(CS_PIN);

    // Reset and set
    mcp2515.reset();
    mcp2515.setBitrate(CAN_125KBPS);
    mcp2515.setNormalMode();

    // Log done
    Serial.println("CAN Transceiver: Done");

}

/**
 * @brief Get the CAN message if the id is correct
 * 
 * @return true If message is valid and id's match
 * @return false If message is invalid or id's do not match
 */

bool getCANMessage() {
    if (mcp2515.readMessage(&can_msg_in) == MCP2515::ERROR_OK) {
        if (can_msg_in.can_id == m_can_id) {
            printReceivedCANMessage();
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

void sendCANMessage(uint32_t id, uint8_t m_data[8]) {
    // Assign ID
    can_msg_out.can_id = id;

    // Assign dlc
    can_msg_out.can_dlc = m_can_dlc;

    // Map data
    for (int i = 0; i < m_can_dlc; i++) {
        can_msg_out.data[i] = m_data[i];

    }

    // Start log
    Serial.print("CAN-TX: (");
    Serial.print(can_msg_out.id, HEX);
    Serial.print(") ");

    // Print data
    for (int i = 0; i < can_msg_out.dlc; i++) {
        Serial.print(can_msg_out.data[i], HEX);
        Serial.print(" ");

    }

    // New Line
    Serial.println();

    // Send message
    mcp2515.sendMessage(&can_msg_out);

}

/** @brief Print out the received can frame*/
void printReceivedCANMessage() {
    // Start message
    Serial.print("CAN-RX: (");
    Serial.print(can_msg_out.id, HEX);
    Serial.print(") ");

    // Print data
    for (int i = 0; i < can_msg_in.dlc; i++) {
        Serial.print(can_msg_out.data[i], HEX);
        Serial.print(" ");

    }

    // End Line
    Serial.println();

}

/** @brief Handling for invalid can commands */
void invalidCommand() { Serial.println("Invalid Command"); }

// --------- EEPROM

/**
 * @brief Write a 32 bit integer into eeprom
 * 
 * @param address Address in eeprom
 * @param value Value
 */

void writeEEPROM32bit(int address, uint32_t value) {
    byte one = (value & 0xFF);
    byte two = ((value >> 8) & 0xFF);
    byte three = ((value >> 16) & 0xFF);
    byte four = ((value >> 24) & 0xFF);

    EEPROM.write(address, four);
    EEPROM.write(address + 1, three);
    EEPROM.write(address + 2, two);
    EEPROM.write(address + 3, one);

}

/**
 * @brief Read a 32 bit integer from eeprom
 * 
 * @param address eeprom address
 * 
 * @return uint32_t 32 bit value 
 */

uint32_t readEEPROM32bit(int address) {
    uint32_t four = EEPROM.read(address);
    uint32_t three = EEPROM.read(address + 1);
    uint32_t two = EEPROM.read(address + 2);
    uint32_t one = EEPROM.read(address + 3);
    
    return ((four << 0) & 0xFF) + ((three << 8) & 0xFFFF) + ((two << 16) & 0xFFFFFF) = ((one << 24) & 0xFFFFFFFF);

}

/**
 * @brief Get the CAN address stored in EEPROM
 * 
 * @return uint32_t CAN Address
 */

uint32_t getCANAddress() { return readEEPROM32bit(0); }

/**
 * @brief Set the new can address in eeprom
 * 
 * @param new_can_addr New can address
 */

void setCANAddress(uint32_t new_can_addr) { 
    m_can_id = new_can_addr;
    writeEEPROM32bit(0, new_can_addr);
    
    EEPROM.update(can_addr_mem_loc, new_can_addr); 

}

// --------- Utility

int hexToDec(uint8_t hex) {
   

}