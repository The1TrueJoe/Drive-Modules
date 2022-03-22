// --------- Lib
#include <Arduino.h>

// --------- Can Interface
#include "mcp2515.h"
#define Default_CAN_CS 10

struct can_frame can_msg_in;
struct can_frame can_msg_out;

uint32_t m_can_id = 0x000;
uint8_t m_can_dlc = 8;

MCP2515* mcp2515;

// --------- EEPROM
#include <EEPROM.h>

private:
    void printReceivedCANMessage();
    void writeEEPROM32bit(int address, uint32_t value);
    uint32_t readEEPROM32bit(int address);
    uint32_t getCANAddress();
    void setCANAddress(uint32_t new_can_addr);

public:
    void setupCAN(int CS_PIN = Default_CAN_CS);
    bool getCANMessage();
    void sendCANMessage(uint32_t id, uint8_t m_data[8]);
    void invalidCommand();
    int hexToDec(uint8_t hex);
    
