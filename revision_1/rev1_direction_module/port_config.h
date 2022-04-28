/**
 * @file port_config.h
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

// Steering Motor Ctrl
#define STR_L_PWM 9
#define STR_R_PWM 6
#define STR_ENABLE 4

// Steering Linear Actuator Potentiometer
#define STR_POT A0

// Steering Wheel Input Encoder
#define STR_WHL_ENC 3
#define STR_WHL_ENC2 A2

// Brake Motor Ctrl
#define BRK_L_PWM 5
#define BRK_R_PWM 10
#define BRK_ENABLE 7

// Brake Actuator Potentiometer
#define BRK_POT A1

// CAN
#define CAN_CS 8
#define CAN_INT 2