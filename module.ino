/**
 * @file module.ino
 * 
 * @author Joseph Telaak
 * 
 * @brief Code for the simple motor controller module
 * 
 * @version 0.1
 * @date 2022-02-12
 * 
 * @copyright Copyright (c) 2022
 * 
 */

// --------- Pin Definitions

// Motor Controller 1
#define CTRL1_L_PWM 9
#define CTRL1_R_PWM 6
#define CTRL1_ENABLE 8

// Motor Controller 2
#define CTRL2_L_PWM 5
#define CTRL2_R_PWM 3
#define CTRL2_ENABLE 7

// Indentification Light
#define ID_LIGHT_RED A0
#define ID_LIGHT_GREEN A1
#define ID_LIGHT_BLUE A2

// Can Interface
#include <SPI.h>
#include <mcp2515.h>

struct can_frame can_msg_in;
struct can_frame can_msg_out;

uint32_t m_can_id = 0x00;
uint8_t m_can_dlc = 8;

MCP2515 mcp2515(10);

// EEPROM
#include <EEPROM.h>

// --------- Setup Functions

/**
 * @brief Arduino default setup function
 * 
 */

void setup() {
    // Init serial port
    Serial.begin(115200);

    // Setup the id light
    setupIDLight();

    if (m_can_id == 0x00) {
        errorLight();
        Serial.println("Error: No CAN Address in EEPROM");

        while ()


    } else {
        m_can_id = getCANAddress();

    }


    // Setup the can bus transceiver
    setupCAN();

    // Setup the motor controllers
    setupMotorControllers();

    

}

// --------- Primary Loop

/**
 * @brief Arduino main loop
 * 
 */

void loop() {
    if (mcp2515.readMessage(&can_msg_in) == MCP2515::ERROR_OK) {
        printReceivedCANMessage()

    }
}

// --------- Motor Controllers

/**
 * @brief 
 * 
 */

void setupMotorControllers() {
    // Motor Controller 1 PinMode
    Serial.println("Motor Controller 1: Setting Pin Mode");
    pinMode(CTRL1_ENABLE, OUTPUT);
    pinMode(CTRL1_L_PWM, OUTPUT);
    pinMode(CTRL1_R_PWM, OUTPUT);

    // Reset controller
    resetMotorController1();

    // Motor Controller 2 PinMode
    Serial.println("Motor Controller 2: Setting Pin Mode");
    pinMode(CTRL2_ENABLE, OUTPUT);
    pinMode(CTRL2_L_PWM, OUTPUT);
    pinMode(CTRL2_R_PWM, OUTPUT);

    // Reset controller
    resetMotorController2();

    // Post update
    Serial.println("Motor Controllers: Setup Complete");

}

/** @brief Enables motor controller by setting the enable pin high */
void enableMotorController1(int i) {  digitalWrite(CTRL1_ENABLE, HIGH); }

/**
 * @brief Writes all pins low
 * 
 */

void resetMotorController1() {
    // Write Controller 1 Low
    Serial.println("Motor Controller 1: Writing all pins low");
    digitalWrite(CTRL1_ENABLE, LOW);
    digitalWrite(CTRL1_L_PWM, LOW);
    digitalWrite(CTRL1_R_PWM, LOW);

}

/** @brief Enables motor controller 2 by setting the enable pin high */
void enableMotorController2() { digitalWrite(CTRL2_ENABLE, HIGH); }

/**
 * @brief Writes all pins low
 * 
 */

void resetMotorController2() {
    // Write Controller 2 Low
    Serial.println("Motor Controller 2: Writing all pins low");
    digitalWrite(CTRL2_ENABLE, LOW);
    digitalWrite(CTRL2_L_PWM, LOW);
    digitalWrite(CTRL2_R_PWM, LOW);

}

// --------- Light

/**
 * @brief Enables the ID light pins
 * 
 */

void setupIDLight() {
    Serial.println("ID Light: Setting Pin Mode")
    pinMode(ID_LIGHT_RED, OUTPUT);
    pinMode(ID_LIGHT_GREEN, OUTPUT);
    pinMode(ID_LIGHT_BLUE, OUTPUT);

}

/**
 * @brief Sets the color on the led light
 * 
 * @param red red value
 * @param green green value
 * @param blue blue value
 */

void setIDLightColor(int red, int green, int blue) {
    Serial.println("ID Light: Set R: " str(red) + " G: " = str(green) + " B: " + str(blue));
    analogWrite(ID_LIGHT_RED, red);
    analogWrite(ID_LIGHT_GREEN, green);
    analogWrite(ID_LIGHT_BLUE, blue);

}

/** @brief Set the light to red to show an error */
void errorLight() { setIDLightColor(255, 0, 0); }

/** @brief Set the light to off */
void offLight() { setIDLightColor(0, 0, 0); }

// --------- CAN

/**
 * @brief 
 * 
 */

void setupCAN() {
    // Log intit
    Serial.println("CAN Transciever: Init Starting");

    // Reset and set
    mcp2515.reset();
    mcp2515.setBitrate(CAN_125KBPS);
    mcp2515.setNormalMode();

    // Log done
    Serial.println("CAN Transciever: Done");

}

/**
 * @brief 
 * 
 * @param m_data 
 */

void sendCANMessage(uint8_t m_data[8]) {
    // Assign ID
    can_msg_out.can_id = m_can_id;

    // Assign dlc
    can_msg_out.can_dlc = m_can_dlc;

    // Map data
    for (int i = 0; i < m_can_dlc; i++) {
        can_msg_out.data[i] = m_data[i];

    }

    // Start log
    Serial.print("CAN Transceiver: Send Message ID: ");
    Serial.print(can_msg_out.id, HEX);
    Serial.print(" DLC: ");
    Serial.print(can_msg_out.dlc, HEX);
    Serial.print(" Msg: ");

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
    Serial.print("CAN Transceiver: Received Message ID: ") +
    Serial.print(can_msg_in.id, HEX);
    Serial.print(" DLC: ");
    Serial.print(can_msg_in.dlc, HEX);
    Serial.print(" Msg: ");

    // Print data
    for (int i = 0; i < can_msg_in.dlc; i++) {
        Serial.print(can_msg_out.data[i], HEX);
        Serial.print(" ");

    }

    // End Line
    Serial.println();

}

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

int hexToDec(String hex) {
   

}

