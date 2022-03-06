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

// --------- Lib

#include <module.h>

// --------- Setup Functions

/**
 * @brief Arduino default setup function
 * 
 */

void setup() {
    // Standard module setup
    standardModuleSetup();

    // Setup the motor controllers
    setupMotorControllers();

    

}

// --------- Primary Loop

/**
 * @brief Arduino main loop
 * 
 */

void loop() {
    if (getCANMessage()) {
        standardModuleLoopHead();

        switch (can_msg_in.data[0]) {
            case 0x0C:
                int id = can_msg_in.data[1];

                switch (can_msg_in.data[2]) {
                    case 0:
                        resetMotorController(id);
                        break;

                    case 1:
                        enableMotorController(id);
                        break;

                    case 2:
                        postMotorControllerStatus(id);
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

/**
 * @brief Enables motor controller by setting the enable pin high
 * 
 * @param i 
 * 
 * @return true If motor controller is enabled
 * @return false If motor controller is not enabled
 */

bool enableMotorController(int i) { 
    if (i == 1) {
        Serial.println("Motor Controller 1: Enabled");
        digitalWrite(CTRL1_ENABLE, HIGH);
        return true; 
        
    } else if (i == 2) {
        Serial.println("Motor Controller 2: Enabled");
        digitalWrite(CTRL2_ENABLE, HIGH);
        return true;
        
    } else {
        Serial.println("Error: Invalid Motor ID While Enabling: " + str(i));
        return false;

    }
}

/**
 * @brief Resets motor controllers
 * 
 * @param i Motor controller to reset
 * 
 * @return true If motor controller is reset
 * @return false If motor controller is not reset
 */

bool resetMotorController(int i) {
     if (i == 1) {
        // Write Controller 1 Low
        Serial.println("Motor Controller 1: Reset");
        digitalWrite(CTRL1_ENABLE, LOW);
        digitalWrite(CTRL1_L_PWM, LOW);
        digitalWrite(CTRL1_R_PWM, LOW);

        return true; 
        
    } else if (i == 2) {
        // Write Controller 2 Low
        Serial.println("Motor Controller 2: Reset");
        digitalWrite(CTRL2_ENABLE, LOW);
        digitalWrite(CTRL2_L_PWM, LOW);
        digitalWrite(CTRL2_R_PWM, LOW);

        return true;
        
    } else {
        Serial.println("Error: Invalid Motor ID While Resetting: " + str(i));
        return false;

    }
}

/**
 * @brief Get the Motor Controller Status
 * 
 * @param i Motor ID
 * 
 * @return true Motor is enabled
 * @return false Motor is disabled
 */

bool getMotorControllerStatus(int i) {
    if (i == 1) {
        if (digitalRead(CTRL1_ENABLE) == 1) {
            Serial.printn("Motor Controller 1: Current Status is Enabled");
            return true;
            
        } else {
            Serial.printn("Motor Controller 1: Current Status is Disabled");
            return false;

        }

    } else if (i == 2) {
        if (digitalRead(CTRL2_ENABLE) == 1) {
            Serial.printn("Motor Controller 2: Current Status is Enabled");
            return true;
            
        } else {
            Serial.printn("Motor Controller 2: Current Status is Disabled");
            return false;

        }
        
    } else {
        Serial.println("Error: Invalid Motor ID While Checking Status: " + str(i));
        return false;

    }
}