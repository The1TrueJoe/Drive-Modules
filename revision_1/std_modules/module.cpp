/**
 * @file module.cpp
 * 
 * @author Joseph Telaak
 * 
 * @brief Standard module helper methods
 * 
 * @version 0.1
 * 
 * @date 2022-04-26
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include <module.h>

/**
 * @brief Standard module setup
 * 
 */

void standardModuleSetup(int CAN_CS_PIN) {
    // Init serial port
    Serial.begin(115200);

    #ifdef HAS_LIGHT
        // Setup the id light
        setupIDLight();
    #endif

    // Setup the can bus transceiver
    setupCAN(CAN_CS_PIN);

}

/**
 * @brief Tells the drive computer that the device is ready
 * 
 */

void ready() {
    uint8_t message[8] = { 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA };

    #ifdef DEBUG
        Serial.println("Module Ready")
    #endif

    sendCANMessage(master_can_id, message);

}

/**
 * @brief Holds the module until receiveing an enable message. NOTE: Disable interupts first, enable interupts afterwards
 * 
 */

void holdTillEnabled() {
    bool enabled = false;
    uint8_t enabled_message[8] = { 0xAA, 0xAB, 0xAC, 0xAD, 0xAF, 0xA0, 0xA1, 0xA2 };

    #ifdef DEBUG
        Serial.println("Awaiting Enable Message!");
    #endif

    ready();
    getCANMessage();

    while (!enabled) {
        bool equal_message = true; 

        for (int i = 0; i < sizeof(enabled_message); i++) {
            if (enabled_message[i] != can_msg_in.data[i]) {
                equal_message = false;
                break;

            }
        }

        if (!equal_message) {
            delay(100);

            #ifdef DEBUG
                Serial.println("Awaiting Enable Message!");
            #endif

            ready();
            getCANMessage();

        } else {
            #ifdef DEBUG
                Serial.println("Enable Message Received!");
            #endif

            enabled = true;

        }
    }

    #ifdef DEBUG
        Serial.println("Hold Ended");
    #endif

}

/**
 * @brief Standard module loop. Reads message and determines the course of action
 * 
 */

void standardModuleLoopHead() {
    
}

/**
 * @brief Standard module loop. Reads message and determines the course of action
 * 
 */

void standardModuleLoopTail() {
    
}