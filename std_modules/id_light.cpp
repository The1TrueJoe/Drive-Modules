#include "id_light.h"

// --------- Light

/**
 * @brief Enables the ID light pins
 * 
 */

void ID_LIGHT::setupIDLight() {
    Serial.println("ID Light: Setting Pin Mode")
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

void ID_LIGHT::setIDLightColor(int red, int green, int blue) {
    Serial.println("ID Light: Set R: " String(red) + " G: " = String(green) + " B: " + String(blue));
    analogWrite(ID_LIGHT_RED, red);
    analogWrite(ID_LIGHT_GREEN, green);
    analogWrite(ID_LIGHT_BLUE, blue);

}

/** @brief Set the light to red to show an error */
void ID_LIGHT::errorLight() { setIDLightColor(255, 0, 0); }

/** @brief Set the light to off */
void ID_LIGHT::offLight() { setIDLightColor(0, 0, 0); }