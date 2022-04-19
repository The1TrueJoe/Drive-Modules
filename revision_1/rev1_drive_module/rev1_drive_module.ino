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
#include <mcp4xxx.h>

using namespace icecave::arduino;

// Activity Control
#define ACT_SW 4

// Direction Control
#define FWD_REV_SEL 5
volatile bool buzzer_enabled = false;
uint32_t accessory_control_address = accessory_module_default_address;

// Digital Potentiometer
volatile bool manual_accel = false;
mcp4xxx* mcp4151(SPEED_CTRL_CS);
#define SPEED_CTRL_CS 9
#define NO_DIGIPOT_READ
#define RESET_BY_DECREMENT

#ifdef NO_DIGIPOT_READ
    volatile uint8_t wiper_pos = 0;
    
#endif

// Accelerator Pedal
#define PEDAL_IN A3
#define PEDAL_SW 3

// CAN
#define CAN_CS 10
#define CAN_INT 2

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
    m_can_id = speed_module_default_address;

    // Standard module setup
    standardModuleSetup(CAN_CS);

    // Announce Ready
    ready();
    holdTillEnabled();

    #ifdef RESET_BY_DECREMENT
        #ifdef DEBUG
            Serial.println("Reseting Wiper Pos by Decrementing");

        #endif

        // Reset Wiper
        for (int i = 0; i < 260; i++) {
            mcp4151->decrement();
            
        }

    #else
        mcp4151->write(0);
    
    #endif

    // Setup Interupts
    attachInterupt(digitalPinToInterrupt(CAN_INT), canLoop, FALLING);
    attachInterupt(digitalPinToInterrupt(PEDAL_SW), pedalPressed, RISING);

    // Setup
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
    uint8_t message[8] = { 0x0C, 0x0C, 0x0A, 0x0D, getCANBoolean(condition), 0x00, 0x00, 0x00};
    sendCANMessage(m_can_id, message);

    // Return condition
    return condition;

} 

/** @brief Set the accelerator digital pot position */
void setAccelPos(uint8_t pos) {
    #ifdef RESET_BY_DECREMENT
        uint8_t current_pos = getAccelSetting();

        if (current_pos > pos) {
            for (int i = current_pos; i < pos; i++) {
                mcp4151->increment();

                #ifdef NO_DIGIPOT_READ
                    wiper_pos++;
                #endif

            }

        } else if (current_pos < pos) {
            for (int i = current_pos; i > pos; i--) {
                mcp4151->decrement();

                #ifdef NO_DIGIPOT_READ
                    wiper_pos--;
                #endif

            }
        }

        for (int i = )

    #else
        mcp4151->write(pos);

    #endif

    postAccelSetting();

}

/** @brief Increment the accelerator digital pot position */
void incAccelPos() { 
    mcp4151->increment();

    #ifdef NO_DIGIPOT_READ
        wiper_pos++;

    #endif

    postAccelSetting();

}

/** @brief Decrement the accelerator digital pot position */
void decAccelPos() { 
    mcp4151->increment();

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
        return mcp4151->readWiper();

    #endif

}

/** @brief Report the accelerator digital pot position */
uint8_t postAccelSetting() {
    // Build Message
    uint8_t message[8] = {0x0C, 0x0C, 0x0A, 0x0A, getAccelSetting(), 0x00, 0x00, 0x00};
    sendCANMessage(m_can_id, message);

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
    sendCANMessage(m_can_id, message);

    // Return value
    return pedal_pos;

}

/** @brief The Accelerator Pedal is Pressed */
void pedalPressed() {
    // Build Message
    uint8_t message[8] = {};
    sendCANMessage(m_can_id, message);

    // Enable the movement relay if the pedal is pressed
    if (manual_accel) {
        enableMovement();

    }

    // Attach Interupt
    attachInterupt(digitalPinToInterrupt(PEDAL_SW), pedalPressed, RISING);

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