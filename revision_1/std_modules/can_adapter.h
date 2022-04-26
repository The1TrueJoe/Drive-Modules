/**
 * @file can_adapter.h
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

// --------- Lib
#include <Arduino.h>

// Can Lib
#include "arduino-mcp2515/mcp2515.h"

// CAN Pins
#define Default_CAN_CS 10
#define Default_CAN_INT 2

//  Message Buffer
struct can_frame can_msg_in;
struct can_frame can_msg_out;

// CAN Info
uint32_t m_can_id = 0x000;
uint8_t m_can_dlc = 8;

// Adapter
MCP2515* can_adapter;

// EEPROM
#ifdef USES_EEPROM
    #include <EEPROM.h>
#endif

// CAN
#ifdef DEBUG
    void printReceivedCANMessage();
#endif

void setCANAddress(uint32_t new_can_addr);
uint8_t getCANBoolean(bool condition);

// EEPROM
#ifdef USES_EEPROM
    void writeEEPROM32bit(int address, uint32_t value);
    uint32_t readEEPROM32bit(int address);
#endif

// Utils
int convertToInt(uint8_t incoming_int);
int convertToInt(uint8_t int_1, uint8_t int_2);

// Setup CAN
void setupCAN(int CS_PIN = Default_CAN_CS, uint32_t id = 0x000);

// Get message
bool getCANMessage();

// Send message
void sendCANMessage(uint32_t id, uint8_t m_data[8]);