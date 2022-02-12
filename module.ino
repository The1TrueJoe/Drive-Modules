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
#define CTRL2_L_PWM 
#define CTRL2_R_PWM 
#define CTRL2_ENABLE 

// Indentification Light
#define ID_LIGHT_RED
#define ID_LIGHT_GREEN
#define ID_LIGHT_BLUE

// Can Interface

// --------- Setup Functions

/**
 * @brief Arduino default setup function
 * 
 */

void setup() {

}

// --------- Primary Loop

/**
 * @brief Arduino main loop
 * 
 */

void loop() {

}

// --------- Motor Controllers

void setupMotorControllers() {
    // Motor Controller 1 PinMode
    pinMode(CTRL1_ENABLE, OUTPUT);
    pinMode(CTRL1_L_PWM, OUTPUT);
    pinMode(CTRL1_R_PWM, OUTPUT);

    // Write Controller 1 Low
    digitalWrite(CTRL1_ENABLE, LOW);
    digitalWrite(CTRL1_L_PWM, LOW);
    digitalWrite(CTRL1_R_PWM, LOW);

    // Motor Controller 2 PinMode
    pinMode(CTRL2_ENABLE, OUTPUT);
    pinMode(CTRL2_L_PWM, OUTPUT);
    pinMode(CTRL2_R_PWM, OUTPUT);

    // Write Controller 2 Low
    digitalWrite(CTRL2_ENABLE, LOW);
    digitalWrite(CTRL2_L_PWM, LOW);
    digitalWrite(CTRL2_R_PWM, LOW);

}

/**
 * @brief Sets the motor controller enable pins high for both controllers
 * 
 */

void enableMotorControllers() {
    enableMotorController1();
    enableMotorController2();

}

/**
 * @brief Enables motor controller 1 by setting the enable pin high
 * 
 */

void enableMotorController1() {
    digitalWrite(CTRL1_ENABLE, HIGH);

}

/**
 * @brief Enables motor controller 2 by setting the enable pin high
 * 
 */

void enableMotorController2() {
    digitalWrite(CTRL2_ENABLE, HIGH);
    
}

// --------- Light

/**
 * @brief Enables the ID light pins
 * 
 */

void setupIDLight() {
    pinMode(ID_LIGHT_RED, OUTPUT);
    pinMode(ID_LIGHT_GREEN, OUTPUT);
    pinMode(ID_LIGHT_BLUE, OUTPUT);

}

/**
 * @brief Sets the color on the led light
 * 
 * @param red red value
 * @param green green value
 * @param blue blue value
 */

void setIDLightColor(int red, int green, int blue) {
    analogWrite(ID_LIGHT_RED, red);
    analogWrite(ID_LIGHT_GREEN, green);
    analogWrite(ID_LIGHT_BLUE, blue);

}