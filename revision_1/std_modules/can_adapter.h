// --------- Lib
#include <Arduino.h>

// Can Lib
#include "mcp2515.h"

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
MCP2515* mcp2515;

// EEPROM
#ifdef USES_EEPROM
    #include <EEPROM.h>
#endif

private:
    // CAN
    void printReceivedCANMessage();
    void setCANAddress(uint32_t new_can_addr);
    uint8_t getCANBoolean(bool condition);

    // EEPROM
    #ifdef USES_EEPROM
        void writeEEPROM32bit(int address, uint32_t value);
        uint32_t readEEPROM32bit(int address);
        uint32_t getCANAddress();
    #endif
    
    // Utils
    int convertToInt(uint8_t incoming_int);
    int convertToInt(uint8_t int_1, uint8_t int_2);

public:
    // Setup CAN
    void setupCAN(int CS_PIN = Default_CAN_CS, uint32_t id);

    // Get message
    bool getCANMessage();

    // Send message
    void sendCANMessage(uint32_t id, uint8_t m_data[8]);