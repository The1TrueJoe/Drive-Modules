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

#define STR_POT A5

const uint8_t str_id = 0x01;

// Brake Motor Ctrl
#define BRK_L_PWM 5
#define BRK_R_PWM 10
#define BRK_ENABLE 7

#define BRK_ENC 2

#define BRK_PEDAL A4

const uint8_t brk_id = 0x02;

volatile long brk_enc_ticks = 0;

// CAN
#define CAN_CS 8
#define CAN_INT 3

// --------- Lib

#include <module.h>



// --------- Setup Functions

/**
 * @brief Arduino default setup function
 * 
 */

void setup() {
    // Standard module setup
    standardModuleSetup(CAN_CS);
    attachInterupt(CAN_INT, canLoop, FALLING);

}

// --------- Primary Loop

/**
 * @brief Arduino main loop
 * 
 */

void loop() {
    // Periodic
    if (checkBrakes()) { postBrakeEngaged(); }

}

void canLoop() {
    standardModuleLoopHead();
        

    standardModuleLoopTail();
}

// --------- Brakes

/**
 * @brief 
 * 
 */

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
    postBrakeEngaged();
    postBrakeTicks();

}

/**
 * @brief 
 * 
 */

void resetBrakeMotor() {
    // Write Controller 2 Low
    Serial.println("Brake Motor Ctrl: Reset");
    digitalWrite(BRK_ENABLE, LOW);
    digitalWrite(BRK_L_PWM, LOW);
    digitalWrite(BRK_R_PWM, LOW);

}

/**
 * @brief 
 * 
 */

void enableBrakeMotor() {
    Serial.println("Brake Motor Ctrl: Enabled");
    digitalWrite(BRK_ENABLE, HIGH);
    
}

/**
 * @brief 
 * 
 * @return true 
 * @return false 
 */

bool isBrakeEnabled() { return digitalRead(BRK_ENABLE) == 1; }

/**
 * @brief 
 * 
 * @return true 
 * @return false 
 */

bool postBrakeEnabled() {
    bool brake = isBrakeEnabled();

    bool brakes = checkBrakes();

    if (brakes) { Serial.println("Brakes Are Enabled"); }
    uint8_t message[8] = {  };

    sendCANMessage(master_can_id, message);

    return brakes;

}

/**
 * @brief 
 * 
 */

void incBrakeTicks() { brk_enc_ticks++; }

/**
 * @brief 
 * 
 */

void resetBrakeTicks() { brk_enc_ticks = 0; }

/**
 * @brief 
 * 
 * @return int 
 */

int postBrakeTicks() {
    uint8_t data[2] = { (brk_enc_ticks >> 8), (brk_enc_ticks&0xFF)};
    uint8_t message[8] = {  };

    sendCANMessage(master_can_id, message);

    return brk_enc_ticks;

}

/**
 * @brief 
 * 
 * @return true 
 * @return false 
 */

bool checkBrakes() { return digitalRead(BRK_PEDAL) == 1; }

/**
 * @brief 
 * 
 * @return true 
 * @return false 
 */

bool postBrakeEngaged() {
    bool brakes = checkBrakes();

    if (brakes) { Serial.println("Brakes Are Engaged"); }
    uint8_t message[8] = {  };

    sendCANMessage(master_can_id, message);

    return brakes;

}

// --------- Steering

/**
 * @brief 
 * 
 */

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

/**
 * @brief 
 * 
 */

void resetSteeringMotor() {
    // Write Controller 1 Low
    Serial.println("Steering Motor Ctrl: Reset");
    digitalWrite(STR_ENABLE, LOW);
    digitalWrite(STR_L_PWM, LOW);
    digitalWrite(STR_R_PWM, LOW);

}

void enableSteeringMotor() {
    Serial.println("Steering Motor Ctrl: Enabled");
    digitalWrite(STR_ENABLE, HIGH);

}

/**
 * @brief 
 * 
 * @return true 
 * @return false 
 */

bool isSteeringEnabled() { return digitalRead(STR_ENABLE) == 1; }

/**
 * @brief 
 * 
 * @return true 
 * @return false 
 */

bool postSteeringEnabled() {
    bool steering = isSteeringEnabled();

    if (steering) { Serial.println("Steering Is Enabled"); }
    uint8_t message[8] = {  };

    sendCANMessage(master_can_id, message);

    return steering;

}

/**
 * @brief 
 * 
 * @return int 
 */

int checkSteeringPos() { return analogRead(STR_POT); }

/**
 * @brief 
 * 
 * @return int 
 */

int postSteeringPos() {
    int pot_value = checkSteeringPos();
    uint8_t data[2] = { (pot_value >> 8), (pot_value&0xFF)};
    uint8_t message[8] = {  };

    sendCANMessage(master_can_id, message);

    return pot_value;

}

/**
 * @brief 
 * 
 * @param duty_cycle 
 */

void turnLeft(int duty_cycle) {
    if (!isSteeringEnabled()) { return; }

    analogWrite(STR_L_PWM, 0);
    analogWrite(STR_R_PWM, duty_cycle);

    Serial.println("Steering Left: " + str(duty_cycle));

}

/**
 * @brief 
 * 
 * @param duty_cycle 
 */

void turnRight(int duty_cycle) {
    if (!isSteeringEnabled()) { return; }

    analogWrite(STR_R_PWM, 0);
    analogWrite(STR_L_PWM, duty_cycle);

    Serial.println("Steering Right: " + str(duty_cycle));

}

/**
 * @brief 
 * 
 * @param duty_cycle 
 * @param pot_pos 
 */

void turnToPos(int duty_cycle, int pot_pos) {
    int current_pos = postSteeringPos();
    int cnt;

    if (current_pos != pot_pos) {
        if (pot_pos > current_pos) {
            turnLeft(duty_cycle);

        } else {
            turnRight(duty_cycle);

        }

        while (current_pos != pot_pos) { 
            delay(1); 
            cnt++;

            if (cnt % 10 == 0) { current_pos = postSteeringPos(); }
            
            if (cnt > 10000) { 
                Serial.println("Turn Timeout");

                uint8_t message[8] = {  };
                sendCANMessage(master_can_id, message);

                return; 
            
            }
            
        }

        resetSteeringMotor();

    }
}