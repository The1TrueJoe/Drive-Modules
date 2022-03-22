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

    Serial.println("Module Ready")
    sendCANMessage(master_can_id, message);

}

/**
 * @brief Holds the module until receiveing an enable message. NOTE: Disable interupts first, enable interupts afterwards
 * 
 */

void holdTillEnabled() {
    bool enabled = false;
    uint8_t enabled_message[8] = { 0xAA, 0xAB, 0xAC, 0xAD, 0xAF, 0xA0, 0xA1, 0xA2 };

    Serial.println("Awaiting Enable Message!");
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

            Serial.println("Awaiting Enable Message!");
            getCANMessage();

        } else {
            Serial.println("Enable Message Received!");
            enabled = true;

        }
    }

    Serial.println("Hold Ended");

}

/**
 * @brief Standard module loop. Reads message and determines the course of action
 * 
 */

void standardModuleLoopHead() {
    switch (can_msg_in.data[0]) {
        case 0x0A:
            switch (can_msg_in.data[1]) {
                case 0x0A:
                    setIDLightColor(hexToDec(can_msg_in.data[2]), hexToDec(can_msg_in.data[3]), hexToDec(can_msg_in.data[4]))
                    break;
            
                default:
                    break;
            }

            break;

        default:
            break;

    }
}

/**
 * @brief Standard module loop. Reads message and determines the course of action
 * 
 */

void standardModuleLoopTail() {

}