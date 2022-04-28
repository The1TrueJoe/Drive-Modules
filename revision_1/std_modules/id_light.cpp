/**
 * @file id_light.cpp
 * 
 * @author Joseph Telaak
 * 
 * @brief Identification light control
 * 
 * @version 0.1
 * 
 * @date 2022-04-26
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "id_light.h"

// --------- Light

/**
 * @brief Enables the ID light pins
 * 
 */

void setupIDLight() {
    #ifdef DEBUG
        Serial.println("ID Light: Setting Pin Mode")
    #endif

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
    #ifdef DEBUG
        Serial.println("ID Light: Set R: " String(red) + " G: " = String(green) + " B: " + String(blue));
    #endif
    
    analogWrite(ID_LIGHT_RED, red);
    analogWrite(ID_LIGHT_GREEN, green);
    analogWrite(ID_LIGHT_BLUE, blue);

}

/** @brief Set the light to red to show an error */
void errorLight() { setIDLightColor(255, 0, 0); }

/** @brief Set the light to off */
void offLight() { setIDLightColor(0, 0, 0); }