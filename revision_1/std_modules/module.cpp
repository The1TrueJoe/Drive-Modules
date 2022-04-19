#include <module.h>

/**
 * @brief Standard module setup
 * 
 */

void standardModuleSetup(int CAN_CS_PIN) {
    // Init serial port
    Serial.begin(115200);

    // Setup the id light
    setupIDLight();

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

    ready()
    getCANMessage();

    while (!enabled) {
        bool equal_message = true; 

        for (int i = 0; i < len(enabled_message); i++) [
            if (enabled_message[i] != can_msg_in[i]) {
                equal_message = false;
                break;

            }
        }

        if (!equal_message) {
            delay(100);

            #ifdef DEBUG
                Serial.println("Awaiting Enable Message!");
            #endif

            ready()
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
    switch (can_msg_in.data[0]) {
        case 0x0F:
            switch (can_msg_in.data[1]) {
                case 0x0A:
                    setIDLightColor(
                        convertToInt(can_msg_in.data[2]), 
                        convertToInt(can_msg_in.data[3]), 
                        convertToInt(can_msg_in.data[4]) 
                    );

                    break;
            
                default:
                    break;
            }

            break;

        default:
            break;

    }
}