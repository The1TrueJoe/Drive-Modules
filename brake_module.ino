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
            // 
            case 0x0C:
                motorControlSequence(); break;
                
        
            default:
                break;

        }

        standardModuleLoopTail();

    }
}

/**
 * @brief Motor control can message parsing sequence
 * 
 */

void motorControlSequence() {
    uint8_t controller_id = can_msg_in.data[1];

    switch (can_msg_in.data[2]) {
        case 0x0A:
            switch (can_msg_in.data[3]) {
                case 0x00:
                    resetMotorController(controller_id); break;
                case 0x01:
                    enableMotorController(controller_id); break;
                case 0x02:
                    postMotorControllerStatus(controller_id); break;
                default:
                    invalidCommand(); break;

            }
            
            break;

        case 0x0C:
            switch (can_msg_in.data[3]) {
                case 0x00:
                    runForward(controller_id, hexToDec(can_msg_in.data[4])); break;
                case 0x01:
                    runBackward(controller_id, hexToDec(can_msg_in.data[4])); break;
                default:
                    invalidCommand(); break;

            }

            break;

        default:
            invalidCommand(); break;
    }

}

// --------- Motor Controllers

/**
 * @brief Setup the motor controllers
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
 * @param i Motor controller to enable
 * 
 * @return true If motor controller is enabled
 * @return false If motor controller is not enabled
 */

bool enableMotorController(uint8_t i) { 
    if (i == 0x01) {
        Serial.println("Motor Controller 1: Enabled");
        digitalWrite(CTRL1_ENABLE, HIGH);
        return true; 
        
    } else if (i == 0x02) {
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

bool resetMotorController(uint8_t i) {
     if (i == 0x01) {
        // Write Controller 1 Low
        Serial.println("Motor Controller 1: Reset");
        digitalWrite(CTRL1_ENABLE, LOW);
        digitalWrite(CTRL1_L_PWM, LOW);
        digitalWrite(CTRL1_R_PWM, LOW);

        return true; 
        
    } else if (i == 0x02) {
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

bool getMotorControllerStatus(uint8_t i) {
    if (i == 0x01) {
        if (digitalRead(CTRL1_ENABLE) == 1) {
            Serial.printn("Motor Controller 1: Current Status is Enabled");
            return true;
            
        } 

    } else if (i == 0x02) {
        if (digitalRead(CTRL2_ENABLE) == 1) {
            Serial.printn("Motor Controller 2: Current Status is Enabled");
            return true;
            
        }
        
    } else {
        Serial.println("Error: Invalid Motor ID While Checking Status: " + str(i));
        return false;

    }

    Serial.printn("Motor Controller " + str(i) + ": Current Status is Disabled");
    return false;

}

/**
 * @brief Post motor controller status to the bus
 * 
 * @param controller Controller id
 */

void postMotorControllerStatus(uint8_t controller) {
    uint8_t status = getMotorControllerStatus(controller) ? 0x01 : 0x00;
    uint8_t message = { 0x0C, controller, 0x0A, 0x02, status, 0x00, 0x00, 0x00 };

    sendCANMessage(master_can_id, message);

}

/**
 * @brief Runs the motor controllers forwards
 * 
 * @param controller Controller id
 * @param duty_cycle Duty cycle
 */

void runForward(uint8_t controller, int duty_cycle) {
    if (controller == 1) {
        Serial.println("Motor Controller 1: Forward Speed " + str(duty_cycle));
        analogWrite(CTRL1_R_PWM, duty_cycle);

    } else if (controller == 2) {
        Serial.println("Motor Controller 2: Forward Speed " + str(duty_cycle));
        analogWrite(CTRL2_R_PWM, duty_cycle);
        
    } else {
        Serial.println("Error: Invalid Motor ID While Setting Speed: " + str(i));

    }
}

/**
 * @brief Runs the motor controllers backwards
 * 
 * @param controller Controller id
 * @param duty_cycle Duty cycle
 */

void runBackward(int controller, int duty_cycle) {
    if (controller == 1) {
        Serial.println("Motor Controller 1: Reverse Speed " + str(duty_cycle));
        analogWrite(CTRL1_L_PWM, duty_cycle);

    } else if (controller == 1) {
        Serial.println("Motor Controller 2: Reverse Speed " + str(duty_cycle));
        analogWrite(CTRL2_L_PWM, duty_cycle);
        
    } else {
        Serial.println("Error: Invalid Motor ID While Setting Speed: " + str(i));

    }
}