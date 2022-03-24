/**
 * @file speed_control_module.ino
 * 
 * @author Joseph Telaak
 * 
 * @brief Code for the speed control module
 * 
 * @version 0.1
 * 
 * @date 2022-03-23
 * 
 * @copyright Copyright (c) 2022
 * 
 */

// Libraries
#include <module.h>
#include <MCP4131.h>

// Accelerator Input
#define ACCEL_INPUT_SEL 7

// Activity Control
#define ACT_SW 4
#define ACT_SEL 3

// Direction Control
#define FWD_REV_SEL 5
volatile bool buzzer_enabled = false;
uint32_t accessory_control_address = accessory_module_default_address;

// Digital Potentiometer
#define SPEED_CTRL_CS 9
MCP4131 mcp4131(SPEED_CTRL_CS);

// CAN
#define CAN_CS 10

// --------- Arduino

/**
 * @brief Arduino setup function
 * 
 */

void setup() {
    // CAN ID
    m_can_id = speed_module_default_address;

    // Standard module setup
    standardModuleSetup(CAN_CS);

    // Announce Ready
    ready();
    holdTillEnabled();

    // Setup Interupts
    attachInterupt(digitalPinToInterrupt(CAN_INT), canLoop, FALLING);

    // Relay setup
    setupDirectionSelector();
    setupAcceleratorSelector();
    setupActivitySwitch();

}

/**
 * @brief Arduino main periodic loop
 * 
 */

void loop() {
    postAccelSetting();
    postDirection();
    postMovementEnabled();

    delay(10000);

}

/**
 * @brief CAN Message handling
 * 
 */

void canLoop() {
    // Get message
    if (!getCANMessage()) { return: }

    standardModuleLoopHead();

    switch (can_msg_in.data[0]) {
        case 0x0A:
            switch (can_msg_in.data[1]) {
                case 0x0A:
                    switch (can_msg_in.data[2]) {
                        case 0xA:
                            setAccelPos(can_msg_in.data[3]);
                            break;

                        case 0x0D:
                            switch (can_msg_in.data[3]) {
                                case 0x01:
                                    enableManualAccelInput();
                                    break;

                                case 0x02:
                                    disableManualAccelInput();
                                    break;

                                default:
                                    break;
                            }

                            break;

                        case 0x0E:
                            switch (can_msg_in.data[3]) {
                                case 0x01:
                                    enableMovement();
                                    break;

                                case 0x02:
                                    disableMovement();
                                    break;

                                default:
                                    break;
                            }

                            break;

                        default:
                            break;
                    }

                    break;

                case 0x0D:
                    switch (can_msg_in.data[2]) {
                        case 0x01:
                            forward();
                            break;

                        case 0x02:
                            reverse();
                            break;

                        case 0x0B:
                            switch (can_msg_in.data[3]) {
                                case 0x01:
                                    enableBuzzerCtrl();
                                    break;

                                case 0x0@:
                                    disableBuzzerCtrl();
                                    break;

                                default:
                                    break;
                            }

                            break;

                        default:
                            break;
                    }

                    break;

                default:
                    break;
            }

            break;

        case 0x0B:
            switch (can_msg_in.data[1]) {
                case 0x0A:
                    switch (can_msg_in.data[2]) {
                        case 0x01:
                            incAccelPos();
                            break;

                        case 0x02:
                            decAccelPos();
                            break;

                        default:
                            break;
                    }

                    break;

                default:
                    break;
            }

            break;

        case 0x0C:
            switch (can_msg_in.data[1]) {
                case 0x0A:
                    switch (can_msg_in.data[2]) {
                        case 0x0A:
                            postAccelSetting();
                            break;

                        case 0x0D:
                            postManualAccelInput();
                            break;

                        case 0x0E:
                            postMovementEnabled();
                            break;

                        default:
                            break;
                    }

                    break;

                case 0x0B:
                    postBuzzerEnable();
                    break;

                case 0x0D:
                    postDirection();
                    break;

                default:
                    break;
            }

            break;

        default:
            break;
    }

     standardModuleLoopTail();

}

// --------- Forward / Reverse

/** @brief Setup the direction selector switch */
void setupDirectionSelector() {
    pinMode(FWD_REV_SEL, OUTPUT); 
    forward();

}

/** @brief Set the direction to forwards */
void forward() { 
    // Set direction
    digitalWrite(FWD_REV_SEL, LOW); 
    postDirection();

    // Buzzer control
    if (buzzer_enabled) {
        uint8_t message[8] = { 0x0A, 0x06, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00 };
        sendCANMessage(accessory_control_address, message);

    }
    
}

/** @brief Set the direction to revers */
void reverse() {
    // Set direction switch
    digitalWrite(FWD_REV_SEL, HIGH);
    postDirection();

    // Buzzzer control
    if (buzzer_enabled) {
        uint8_t message[8] = { 0x0A, 0x06, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00 };
        sendCANMessage(accessory_control_address, message);

    }

}

/** @brief Check if the direction is forwards */
bool isForwards() { return digitalRead(FWD_REV_SEL) == LOW; }

/** @brief Post the current direction of the drive system */
void postDirection() {
    // Build Message
    uint8_t message[8] = { 0x0C, 0x0C, 0x0D, getCANBoolean(isForwards()), 0x00, 0x00, 0x00, 0x00};

    // Send Message
    sendCANMessage(m_can_id, message);

}

/** @brief Disable buzzzer activation */
void disableBuzzerCtrl() { buzzer_enabled = false; }

/** @brief Enable buzzer activation */
void enableBuzzerCtrl() { buzzer_enabled = true; }

/** @brief Report the buzzer enable statis*/
void postBuzzerEnable() {
    // Build Message
    uint8_t message[8] = { 0x0C, 0x0C, 0x0B, getCANBoolean(buzzer_enabled), 0x00, 0x00, 0x00, 0x00};
    sendCANMessage(m_can_id, message);

}

// --------- Speed Control

/** @brief Set up the accelerator selector */
void setupAcceleratorSelector() {
    pinMode(ACT_SEL, OUTPUT);
    pinMode(ACCEL_INPUT_SEL, OUTPUT);

    enableManualAccelInput();

}

/** @brief Disable manual accelerator input and enable auto speed control */
void disableManualAccelInput() {
    digitalWrite(ACT_SEL, HIGH);
    digitalWrite(ACCEL_INPUT_SEL, HIGH);

}

/** @brief Enable manual selector input  */
void enableManualAccelInput() {
    digitalWrite(ACT_SEL, LOW);
    digitalWrite(ACCEL_INPUT_SEL, LOW);
    
}

/** @brief Check if manual accelerator input */
bool isManualAccelInput() { return digitalRead(ACT_SEL) == LOW; }

/** @brief Post manual acclerator status */
bool postManualAccelInput() {
    int condition = isManualAccelInput();

    // Build Message
    uint8_t message[8] = { 0x0C, 0x0C, 0x0A, 0x0D, getCANBoolean(condition), 0x00, 0x00, 0x00};
    sendCANMessage(m_can_id, message);

    // Return condition
    return condition;

} 

/** @brief Set the accelerator digital pot position */
void setAccelPos(uint8_t pos) {
    mcp4131.writeWiper(pos);
    postAccelSetting();

}

/** @brief Increment the accelerator digital pot position */
void incAccelPos() { 
    mcp4131.incrementWiper();
    postAccelSetting();

}

/** @brief Decrement the accelerator digital pot position */
void decAccelPos() { 
    mcp4131.incrementWiper();
    postAccelSetting();

}

/** @brief Report the accelerator digital pot position */
uint8_t postAccelSetting() {
    // Collect wiper data
    uint8_t wiper_pos = mcp4131.readWiper();

    // Build Message
    uint8_t message[8] = {0x0C, 0x0C, 0x0A, 0x0A, wiper_pos, 0x00, 0x00, 0x00};
    sendCANMessage(m_can_id, message);

    // Return value
    return wiper_pos;

}

// --------- Motor Activity

/** @brief Setup the enable switch */
void setupActivitySwitch() {
    pinMode(ACT_SW, OUTPUT);

    disableMovement();

}

/** @brief Disable the drive motor controller */
void disableMovement() { 
    digitalWrite(ACT_SW, HIGH);
    postMovementEnabled();
    
}

/** @brief Enable the drive motor controller */
void enableMovement() {
    digitalWrite(ACT_SW, LOW);
    postMovementEnabled();

}

/** @brief Check if movement is enabled */
bool isMovementEnabled() { return digitalRead(ACT_SW) == LOW; }

/** @brief post if movement is enabled */
bool postMovementEnabled() {
    int condition = isMovementEnabled();

    // Build Message
    uint8_t message[8] = {0x0C, 0x0C, 0x0A, 0x0E, getCANBoolean(condition), 0x00, 0x00, 0x00};
    sendCANMessage(m_can_id, message);

    // Return condition
    return condition;

}