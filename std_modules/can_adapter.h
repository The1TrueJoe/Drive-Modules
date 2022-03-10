// --------- Lib
#include <Arduino.h>

// --------- Can Interface
#include <SPI.h>
#include <mcp2515.h>

struct can_frame can_msg_in;
struct can_frame can_msg_out;

uint32_t m_can_id = 0x000;
uint8_t m_can_dlc = 8;

uint32_t master_can_id = 0x001;

MCP2515 mcp2515(10);

// --------- EEPROM
#include <EEPROM.h>

private:
    void printReceivedCANMessage();
    void writeEEPROM32bit(int address, uint32_t value);
    uint32_t readEEPROM32bit(int address);
    uint32_t getCANAddress();
    void setCANAddress(uint32_t new_can_addr);

public:
    void setupCAN();
    bool getCANMessage();
    void sendCANMessage(uint32_t id, uint8_t m_data[8]);
    void invalidCommand();
    int hexToDec(uint8_t hex);
    
