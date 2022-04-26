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
#include "module.h"
#include "port_config.h"
#include "arduino-mcp4xxx/mcp4xxx.h"

// Settings
#define NO_DIGIPOT_READ
#define RESET_BY_DECREMENT
//#define HOLD
//#define DEBUG

// DigiPot Namespace
using namespace icecave::arduino;

// Buzzer
volatile bool buzzer_enabled = false;

// Digital Potentiometer
volatile bool manual_accel = false;
MCP4XXX* digi_pot;

#ifdef NO_DIGIPOT_READ
    volatile uint8_t wiper_pos = 0;
    
#endif

// Timing
long time_since_update = 0;
#define UPDATE_INTERVAL 5000

// --------- Arduino

/**
 * @brief Arduino setup function
 * 
 */

void setup() {
    // CAN ID
    can_adapter -> m_can_id = drive_module_address;

    // Standard module setup
    standardModuleSetup(CAN_CS, 0xFF6);

    // Announce Ready
    #ifdef HOLD
        ready();
        holdTillEnabled();
    #endif

    // Setup Interupts
    attachInterrupt(digitalPinToInterrupt(CAN_INT), canLoop, FALLING);
    attachInterrupt(digitalPinToInterrupt(PEDAL_SW), pedalPressed, RISING);

    // Setup
    setupAccelerator();
    setupDirectionSelector();
    setupActivitySwitch();
    setupPedal();

}

/**
 * @brief Arduino main periodic loop
 * 
 */

void loop() {
    if (manual_accel) {
        setAccelPos((postPedalPos() / 4) >> 8);
        delay(10);

    }

    if (millis() - time_since_update > UPDATE_INTERVAL) {
        postAccelSetting();
        postDirection();
        postMovementEnabled();

        if (!manual_accel) {
            postPedalPos();
            delay(100);

        }
    }
}

/**
 * @brief CAN Message handling
 * 
 */

void canLoop() {
    // Get message
    if (!can_adapter -> getCANMessage()) { return; }

    standardModuleLoopHead();

    switch (can_adapter -> can_msg_in.data[0]) {
        case 0x0A:
            switch (can_adapter -> can_msg_in.data[1]) {
                case 0x0A:
                    switch (can_adapter -> can_msg_in.data[2]) {
                        case 0xA:
                            setAccelPos(can_adapter -> can_msg_in.data[3]);
                            break;

                        case 0x0D:
                            switch (can_adapter -> can_msg_in.data[3]) {
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
                            switch (can_adapter -> can_msg_in.data[3]) {
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
                    switch (can_adapter -> can_msg_in.data[2]) {
                        case 0x01:
                            forward();
                            break;

                        case 0x02:
                            reverse();
                            break;

                        case 0x0B:
                            switch (can_adapter -> can_msg_in.data[3]) {
                                case 0x01:
                                    enableBuzzerCtrl();
                                    break;

                                case 0x02:
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
            switch (can_adapter -> can_msg_in.data[1]) {
                case 0x0A:
                    switch (can_adapter -> can_msg_in.data[2]) {
                        case 0x01:
                            incAccelPos();
                            break;

                        case 0x02:
                            decAccelPos();
                            break;

                        case 0x0D:
                            zeroAccelPos();
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
            switch (can_adapter -> can_msg_in.data[1]) {
                case 0x0A:
                    switch (can_adapter -> can_msg_in.data[2]) {
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
        can_adapter -> sendCANMessage(accessory_module_address, message);

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
        can_adapter -> sendCANMessage(accessory_module_address, message);

    }

}

/** @brief Check if the direction is forwards */
bool isForwards() { return digitalRead(FWD_REV_SEL) == LOW; }

/** @brief Post the current direction of the drive system */
void postDirection() {
    // Build Message
    uint8_t message[8] = { 0x0C, 0x0C, 0x0D, can_adapter -> getCANBoolean(isForwards()), 0x00, 0x00, 0x00, 0x00};

    // Send Message
    can_adapter -> sendCANMessage(can_adapter -> m_can_id, message);

}

/** @brief Disable buzzzer activation */
void disableBuzzerCtrl() { buzzer_enabled = false; }

/** @brief Enable buzzer activation */
void enableBuzzerCtrl() { buzzer_enabled = true; }

/** @brief Report the buzzer enable statis*/
void postBuzzerEnable() {
    // Build Message
    uint8_t message[8] = { 0x0C, 0x0C, 0x0B, can_adapter -> getCANBoolean(buzzer_enabled), 0x00, 0x00, 0x00, 0x00};
    can_adapter -> sendCANMessage(can_adapter -> m_can_id, message);

}

// --------- Speed Control

/** @brief Setup the accelerator */
void setupAccelerator() {
    digi_pot = new MCP4XXX(SPEED_CTRL_CS);
    zeroAccelPos();

}

/** @brief Disable manual accelerator input and enable auto speed control */
void disableManualAccelInput() { manual_accel = false; }

/** @brief Enable manual selector input  */
void enableManualAccelInput() { manual_accel = true; }

/** @brief Check if manual accelerator input */
bool isManualAccelInput() { return manual_accel; }

/** @brief Post manual acclerator status */
bool postManualAccelInput() {
    int condition = isManualAccelInput();

    // Build Message
    uint8_t message[8] = { 0x0C, 0x0C, 0x0A, 0x0D, can_adapter -> getCANBoolean(condition), 0x00, 0x00, 0x00};
    can_adapter -> sendCANMessage(can_adapter -> m_can_id, message);

    // Return condition
    return condition;

} 

/** @brief Zeroes the accelerator position */
void zeroAccelPos() {
    #ifdef RESET_BY_DECREMENT
        #ifdef DEBUG
            Serial.println("Reseting Wiper Pos by Decrementing");

        #endif

        // Reset Wiper
        for (int i = 0; i < 260; i++) {
            digi_pot->decrement();
            
        }

    #else
        digi_pot->write(0);
    
    #endif

    postAccelSetting();

}

/** @brief Set the accelerator digital pot position */
void setAccelPos(uint8_t pos) {
    #ifdef RESET_BY_DECREMENT
        uint8_t current_pos = getAccelSetting();

        if (current_pos > pos) {
            for (int i = current_pos; i < pos; i++) {
                digi_pot->increment();

                #ifdef NO_DIGIPOT_READ
                    wiper_pos++;
                #endif

            }

        } else if (current_pos < pos) {
            for (int i = current_pos; i > pos; i--) {
                digi_pot->decrement();

                #ifdef NO_DIGIPOT_READ
                    wiper_pos--;
                #endif

            }
        }

    #else
        digi_pot->write(pos);

    #endif

    postAccelSetting();

}

/** @brief Increment the accelerator digital pot position */
void incAccelPos() { 
    digi_pot -> increment();

    #ifdef NO_DIGIPOT_READ
        wiper_pos++;

    #endif

    postAccelSetting();

}

/** @brief Decrement the accelerator digital pot position */
void decAccelPos() { 
    digi_pot->increment();

    #ifdef NO_DIGIPOT_READ
        wiper_pos--;

    #endif

    postAccelSetting();

}

/** @brief Get the acclerator wiper pos*/
uint8_t getAccelSetting() {
    #ifdef NO_DIGIPOT_READ
        return wiper_pos;

    #else
        return digi_pot->readWiper();

    #endif

}

/** @brief Report the accelerator digital pot position */
uint8_t postAccelSetting() {
    // Build Message
    uint8_t message[8] = {0x0C, 0x0C, 0x0A, 0x0A, getAccelSetting(), 0x00, 0x00, 0x00};
    can_adapter -> sendCANMessage(can_adapter -> m_can_id, message);

    // Return value
    return wiper_pos;

}

// --------- Manual Accelerator Pedal

/** @brief Setup the pedal */
void setupPedal() {
    pinMode(PEDAL_IN, INPUT);
    pinMode(PEDAL_SW, INPUT);

}

/** @brief Read the pedal pos */
int readPedalPos() { return analogRead(PEDAL_IN); }

/** @brief Post the pedal pos*/
int postPedalPos() {
    // Build Message
    int pedal_pos = readPedalPos();
    uint8_t data[2] = { (pedal_pos >> 8), (pedal_pos & 0xFF)};
    uint8_t message[8] = {};
    can_adapter -> sendCANMessage(can_adapter -> m_can_id, message);

    // Return value
    return pedal_pos;

}

/** @brief The Accelerator Pedal is Pressed */
void pedalPressed() {
    // Build Message
    uint8_t message[8] = {};
    can_adapter -> sendCANMessage(can_adapter -> m_can_id, message);

    // Enable the movement relay if the pedal is pressed
    if (manual_accel) {
        enableMovement();

    }

    // Attach Interupt
    attachInterrupt(digitalPinToInterrupt(PEDAL_SW), pedalPressed, RISING);

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
    uint8_t message[8] = {0x0C, 0x0C, 0x0A, 0x0E, can_adapter -> getCANBoolean(condition), 0x00, 0x00, 0x00};
    can_adapter -> sendCANMessage(can_adapter -> m_can_id, message);

    // Return condition
    return condition;

}