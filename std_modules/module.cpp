#include <module.h>

/**
 * @brief Standard module setup
 * 
 */

void standardModuleSetup() {
    // Init serial port
    Serial.begin(115200);

    // Setup the id light
    setupIDLight();

    // Setup the can bus transceiver
    setupCAN();

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