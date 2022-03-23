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

// --------- Definitions

// Steering Motor Ctrl
#define STR_L_PWM 9
#define STR_R_PWM 6
#define STR_ENABLE 4

// Steering Linear Actuator Potentiometer
#define STR_POT A5

// Steering Wheel Input Potentiometer
#define STR_WHL_POT A4

// Steering Wheel ID (From CAN Messages)
const uint8_t str_id = 0x01;

// Manual Mode
bool manual_mode_eng = true;
int manual_mode_steering_speed = 128;

// Brake Motor Ctrl
#define BRK_L_PWM 5
#define BRK_R_PWM 10
#define BRK_ENABLE 7

// Brake Motor Encoder
#define BRK_ENC 2

// Brake ID (From CAN Messages)
const uint8_t brk_id = 0x02;

// Brake Encoder Current Ticks
volatile long brk_enc_ticks = 0;

// CAN
#define CAN_CS 8
#define CAN_INT 3

// --------- Lib

#include <module.h>

// --------- Setup Functions

/** @brief Arduino default setup function (Holds until this module is enabled)  */
void setup() {
    // CAN ID
    m_can_id = direction_control_default_address;

    // Standard module setup
    standardModuleSetup(CAN_CS);

    // Announce Ready
    ready();
    holdTillEnabled();

    // Setup Interupts
    attachInterupt(CAN_INT, canLoop, FALLING);
    attachInterupt(BRK_ENC, incBrakeTicks, FALLING);

    // Setup Motor Controllers
    setupBrakeMotor();
    setupSteeringMotor();

}

// --------- Primary Loop

/** @brief Arduino main loop */
void loop() {
    // Post the positions
    postSteeringPos();
    postSteeringWheelPos();

    // Manual Mode (Takes input from the wheel)
    if (manual_mode_eng) {
        int pos = postSteeringWheelPos();
        turnToPos(manual_mode_steering_speed, pos);

        delay(10);

    } else {
        delay(100); // Longer delay while in normal mode

    }
}

/** @brief CAN Message Handling (Runs on interupt) */
void canLoop() {
    standardModuleLoopHead();

    switch (can_msg_in.data[0]) {
        case 0x0A:
            switch (can_msg_in.data[1]) {
                case 0x01:
                    switch (can_msg_in.data[2]) {
                        case 0x0A:
                            switch (can_msg_in.data[3]) {
                                case 0x01:
                                    enableSteeringMotor();
                                    break;

                                case 0x02:
                                    resetSteeringMotor();
                                    break;

                                default:
                                    break;

                            }
                            
                            break;
                            

                        case 0x0C:
                            switch (can_msg_in.data[3]) {
                                case 0x01:
                                    turnLeft((can_msg_in.data[4] << 8) | can_msg_in.data[5]);
                                    break;

                                case 0x02:
                                    turnRight((can_msg_in.data[4] << 8) | can_msg_in.data[5]);
                                    break;

                                default:
                                    break;

                            }
                            
                            break;

                        case 0x0D:
                            switch (can_msg_in.data[3]) {
                                case 0x01:
                                    enableManualMode(); 
                                    break;S

                                case 0x02:
                                    disableManualMode(); 
                                    break;

                                case 0x0C:
                                    setManualModeSteeringSpeed(int(can_msg_in.data[4]));
                                    break;

                                default:
                                    break;

                            }
                            
                            break;

                        default:
                            break;

                    }

                    break;

                case 0x02:
                    switch (can_msg_in.data[2]) {
                        case 0x0A:
                            switch (can_msg_in.data[3]) {
                                case 0x01:
                                    enableBrakeMotor();
                                    break;

                                case 0x02:
                                    resetBrakeMotor();
                                    break;

                                default:
                                    break;

                            }

                            break;

                        case 0x0C:
                            switch (can_msg_in.data[3]) {
                                case 0x01:
                                    pullBrakes((can_msg_in.data[4] << 8) | can_msg_in.data[5]);
                                    break;

                                case 0x02;
                                    reverseBrakes((can_msg_in.data[4] << 8) | can_msg_in.data[5]);
                                    break;

                                default:
                                    break;

                            }

                            break;

                        case 0x0F:
                            resetBrakeTicks();
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
                case 0x01:
                    turnWheelsToPos((can_msg_in.data[2] << 8) | can_msg_in.data[2]);
                    break;

                default:
                    break;
            
            }

            break;
            

        case 0x0C:
            switch (can_msg_in.data[1]) {
                case 0x01:
                    switch (can_msg_in.data[2]) {
                        case 0x0A:
                            postSteeringEnabled();
                            break;

                        case 0x0D;
                            postSteeringMode();
                            break;

                        case 0x0E:
                            postSteeringWheelPos();
                            break;


                        case 0x0F:
                            postSteeringPos();
                            break;

                        default:
                            break;

                    }

                    break;

                case 0x02:
                    switch (can_msg_in.data[2]) {
                        case 0x0A:
                            postBrakeEnabled();
                            break;

                        case 0x0F:
                            postBrakeTicks();
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
        

    standardModuleLoopTail();
}

// --------- Brakes

/** @brief Sets up the brake motor controller */
void setupBrakeMotor() {
    // Brake Motor Ctrl PinMode
    Serial.println("Brake Motor Ctrl: Setting Pin Mode");
    pinMode(BRK_ENABLE, OUTPUT);
    pinMode(BRK_L_PWM, OUTPUT);
    pinMode(BRK_R_PWM, OUTPUT);

    // Reset Control
    resetBrakeMotor();

    // Post Status
    postBrakeEnabled();
    postBrakeTicks();

}

/** @brief Resets the brake motor controller (Sets all pins low) */
void resetBrakeMotor() {
    // Write Controller 2 Low
    digitalWrite(BRK_ENABLE, LOW);
    digitalWrite(BRK_L_PWM, LOW);
    digitalWrite(BRK_R_PWM, LOW);

    // Report
    Serial.println("Brake Motor Ctrl: Reset");
    postBrakeEnabled();

}

/** @brief Enable the brake motor controller */
void enableBrakeMotor() {
    // Write pin high
    digitalWrite(BRK_ENABLE, HIGH);

    // Report
    Serial.println("Brake Motor Ctrl: Enabled");
    postBrakeEnabled();
    
}

/** @brief Check if brake motor controller is enabled */
bool isBrakeEnabled() { return digitalRead(BRK_ENABLE) == HIGH; }

/** @brief Report the brake motor controller enable status to the bus */
bool postBrakeEnabled() {
    // Check status
    bool brakes = isBrakeEnabled();

    // Build message
    if (brakes) {
        Serial.println("Brakes Are Enabled");
        uint8_t status = 0x01;

    } else {
        uint8_t status = 0x02;

    }

    // Send Message
    uint8_t message[8] = { 0x0C, 0x0C, 0x02, 0x0A, status, 0x00, 0x00, 0x00 };
    sendCANMessage(m_can_id, message);

    // Return status
    return brakes;

}

/** @brief Increment the brake motor encoder tick count */
void incBrakeTicks() { brk_enc_ticks++; }

/** @brief Reset the brake motor encoder tick counter */
void resetBrakeTicks() { brk_enc_ticks = 0; }

/** @brief Post the brake tick count to the bus */
int postBrakeTicks() {
    // Build mesage
    uint8_t data[2] = { (brk_enc_ticks >> 8), (brk_enc_ticks&0xFF)};
    uint8_t message[8] = { 0x0C, 0x0C, 0x02, 0x0F, data[0], data[1], 0x00, 0x00 };

    // Send message
    sendCANMessage(m_can_id, message);

    // Return tick count
    return brk_enc_ticks;

}

/**
 * @brief Pull the brake cord to engage the brakes
 * 
 * @param duty_cycle Motor duty cycle
 */

void pullBrakes(int duty_cycle) {
    // Check enabled
    if (!isBrakeEnabled()) { 
        if (manual_mode_eng) {
            enableBrakeMotor();

        } else {
            return;
            
        }
    }

    // Write to controllers
    analogWrite(BRK_L_PWM, 0);
    analogWrite(BRK_R_PWM, duty_cycle);

    // Report
    Serial.println("Pulling Brakes: " + String(duty_cycle));

}

/**
 * @brief Reverse the brake cord
 * 
 * @param duty_cycle Motor duty cycle
 */

void reverseBrakes(int duty_cycle) {
    // Check enable
    if (!isBrakeEnabled()) { 
        if (manual_mode_eng) {
            enableBrakeMotor();

        } else {
            return;
            
        }
    }

    // Write to controllers
    analogWrite(BRK_R_PWM, 0);
    analogWrite(BRK_L_PWM, duty_cycle);

    // Report
    Serial.println("Reversing Brakes: " + String(duty_cycle));

}

// --------- Steering

/** @brief Setup the steering motor */
void setupSteeringMotor() {
    // Steering Motor Ctrl PinMode
    Serial.println("Steering Motor Ctrl: Setting Pin Mode");
    pinMode(STR_ENABLE, OUTPUT);
    pinMode(STR_L_PWM, OUTPUT);
    pinMode(STR_R_PWM, OUTPUT);

    // Reset Controller
    resetSteeringMotor();

    // Set Potentiometer
    postSteeringPos();
    postSteeringEnabled();

}

/** @brief Resets the steering motor controller (Sets all pins low) */
void resetSteeringMotor() {
    // Write Controller 1 Low
    digitalWrite(STR_ENABLE, LOW);
    digitalWrite(STR_L_PWM, LOW);
    digitalWrite(STR_R_PWM, LOW);

    // Report
    Serial.println("Steering Motor Ctrl: Reset");
    postSteeringEnabled();

}

/** @brief Enable the steering controller */
void enableSteeringMotor() {
    // Write pin high
    digitalWrite(STR_ENABLE, HIGH);

    // Report
    Serial.println("Steering Motor Ctrl: Enabled");
    postSteeringEnabled();

}

/** @brief Check if the steering motor controller is enabled */
bool isSteeringEnabled() { return digitalRead(STR_ENABLE) == 1; }

/**  @brief Report if the steeting motor controller is enabled to the bus */
bool postSteeringEnabled() {
    // Check the status
    bool steering = isSteeringEnabled();

    // Build the message
    if (steering) {
        Serial.println("Steering Is Enabled");
        uint8_t status = 0x01;

    } else {
        uint8_t status = 0x02;
        
    }

    // Send message
    uint8_t message[8] = { 0x0C, 0x0C, 0x01, 0x0A, status, 0x00, 0x00, 0x00 };
    sendCANMessage(m_can_id, message);

    // Return the status
    return steering;

}

/** @brief Check if the steering is in manual mode */
bool isManualMode() { return manual_mode_eng; }

/** @brief Enable the manual steering mode (input from the wheel potentiometer) */
void enableManualMode() { 
    manual_mode_eng = true; 
    postSteeringMode();
    
}

/** @brief Disable the manual steering mode (input from the wheel potentiometer)  */
void disableManualMode() { 
    manual_mode_eng = false; 
    postSteeringMode();

}

/** @brief Set the manual steering speed */
void setManualModeSteeringSpeed(int speed) { manual_mode_steering_speed = speed; }

/** @brief Report the steering mode to the bus */
bool postSteeringMode() {
    // Check if in the manual mode
    bool is_man = isManualMode();

    // Build message
    if (is_man) {
        Serial.println("Manual Mode Enabled");
        uint8_t status = 0x01;

    } else {
        Serial.println("Manual Mode Disabled");
        uint8_t status = 0x02;

    }

    // Send message
    uint8_t message[8] = { 0x0C, 0x0C, 0x01, 0x0D, status, 0x00, 0x00, 0x00 };
    sendCANMessage(m_can_id, message);

    // Return manual
    return is_man;

}

/** @brief Check the steering wheel potentiometer position */
int checkSteeringWheelPos() { return analogRead(STR_WHL_POT); }

/** @brief Report the steeting wheel position to the bus  */
int postSteeringWheelPos() {
    // Build message
    int pot_value = checkSteeringWheelPos();
    uint8_t data[2] = { (pot_value >> 8), (pot_value&0xFF)};
    uint8_t message[8] = { 0x0C, 0x0C, 0x01, 0x0E, data[0], data[1], 0x00, 0x00 };

    // Send message
    sendCANMessage(m_can_id, message);

    // Return the potentiometer value
    return pot_value;

}

/** @brief Check the steering linear actuator position */
int checkSteeringPos() { return analogRead(STR_POT); }

/** @brief Post the steering linear actuator positon to the bus */
int postSteeringPos() {
    // Build message
    int pot_value = checkSteeringPos();
    uint8_t data[2] = { (pot_value >> 8), (pot_value&0xFF)};
    uint8_t message[8] = { 0x0C, 0x0C, 0x01, 0x0F, data[0], data[1], 0x00, 0x00 };

    // Send message
    sendCANMessage(m_can_id, message);

    // Return the potentiometer value
    return pot_value;

}

/**
 * @brief Turn wheels left at a given speed
 * 
 * @param duty_cycle Speed
 */

void turnLeft(int duty_cycle) {
    // Check if enabled
    if (!isSteeringEnabled()) { 
        if (manual_mode_eng) {
            enableSteeringMotor();
            postSteeringEnabled();

        } else {
            return;

        }
    }

    // Set the motor controller
    analogWrite(STR_L_PWM, 0);
    analogWrite(STR_R_PWM, duty_cycle);

    // Report
    Serial.println("Steering Left: " + String(duty_cycle));

}

/**
 * @brief Turn wheels left at a given speed
 * 
 * @param duty_cycle Speed
 */

void turnRight(int duty_cycle) {
    // Check if enabled
    if (!isSteeringEnabled()) { 
        if (manual_mode_eng) {
            enableSteeringMotor();
            postSteeringEnabled();

        } else {
            return;
            
        }
    }

    // Set the motor controller
    analogWrite(STR_R_PWM, 0);
    analogWrite(STR_L_PWM, duty_cycle);

    // Report
    Serial.println("Steering Right: " + String(duty_cycle));

}

/**
 * @brief Turn the wheels to the linear actuator position
 * 
 * @param duty_cycle Speed
 * @param pot_pos Desired potentiometer position
 */

void turnWheelsToPos(int duty_cycle, int pot_pos) {
    // Get the current position
    int current_pos = postSteeringPos();
    int cnt;

    // If the two positions are not equal
    if (current_pos != pot_pos) {
        // Decide the direction to tuen
        if (pot_pos > current_pos) {
            turnLeft(duty_cycle);

        } else {
            turnRight(duty_cycle);

        }

        // Loop to check
        while (current_pos != pot_pos) { 
            // Wait
            delay(1); 
            cnt++;

            // Periodic posts
            if (cnt % 10 == 0) { current_pos = postSteeringPos(); }
            
            // Timeout
            if (cnt > 10000) { 
                Serial.println("Turn Timeout");

                uint8_t message[8] = {  };
                sendCANMessage(m_can_id, message);

                return; 
            
            }
            
        }

        // Rest the motor controller
        resetSteeringMotor();

    }
}