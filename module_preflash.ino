/**
 * @file module_preflash.ino
 * 
 * @author Joseph Telaak
 * 
 * @brief Resets the EEPROM on the Arduino along
 * 
 * @version 0.1
 * @date 2022-03-05
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include <EEPROM.h>

// LED Pin
#define ARDUINO_LED 13

// Module LED
#include "id_light.h"

/**
 * @brief Runs the preflash sequence
 * 
 */

void setup() {
    // Serial Debugging
    Serial.begin(115200);
    Serial.println("Beginning New Module Preflash Sequence............");

    // Setup Lights
    setupIDLight();
    pinMode(ARDUINO_LED, OUTPUT);

    // Clear EEPROM
    clearEEPROM();

}

/**
 * @brief Clears eeprom
 * 
 */

void clearEEPROM() {
    // Clear EEPROM
    Serial.println("Clearing EEPROM.....");
    
    for (int i = 0; i < EEPROM.length(); i++) {
        EEPROM.write(i, 0);

    }

    Serial.println("EEPROM Cleared!");

}

/**
 * @brief Flashes led when completed
 * 
 */

void loop() {
    // Signal Done
    Serial.println("Preflash Complete! Power off Arduino and Flash Module Software")
    digitalWrite(ARDUINO_LED, HIGH);
    setIDLightColor(0, 255, 0);

    // Wait
    delay(500);

    // Flash LEDs
    digitalWrite(ARDUINO_LED, LOW);
    offLight();

    // Wait
    delay(500);

}