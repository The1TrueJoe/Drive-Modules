// To Test

#include <Arduino.h>

//#define TEST_FWDREV
//#define TEST_ACCL_ACT
//#define TEST_ACCL
#define TEST_PEDAL_SW
#define TEST_PEDAL_POT

#ifdef TEST_ACCL
    #include "mcp4xxx.h"
    using namespace icecave::arduino;
    MCP4XXX* accel;
    #define ACCEL_CS 9
#endif

#ifdef TEST_FWDREV
    #define FWD_REV_SEL 5
#endif

#ifdef TEST_ACCL_ACT
    #define ACT_SW 4
#endif

#ifdef TEST_PEDAL_SW
    #define PEDAL_SW 3
    volatile bool pressed;
#endif

#ifdef TEST_PEDAL_POT
    #define PEDAL_POT A3
#endif 

void setup() {
    Serial.begin(115200);
    Serial.println("Testing Hardware.... DO NOT CONNECT TO LIVE CART!!");
    delay(2000);

    #ifdef TEST_FWDREV
        Serial.println("Testing Foward/Reverse Selector");
        pinMode(FWD_REV_SEL, OUTPUT);

        for (int i = 0; i < 4; i++) {
            Serial.println("HIGH");
            digitalWrite(FWD_REV_SEL, HIGH);

            delay(1000);

            Serial.println("LOW");
            digitalWrite(FWD_REV_SEL, LOW);

            delay(1000);

        }

        Serial.println("Done");
    #endif

    #ifdef TEST_ACCL_ACT
        Serial.println("Testing Accelerator Switch");
        pinMode(ACT_SW, OUTPUT);

        for (int i = 0; i < 4; i++) {
            Serial.println("HIGH");
            digitalWrite(ACT_SW, HIGH);

            delay(1000);

            Serial.println("LOW");
            digitalWrite(ACT_SW, LOW);

            delay(1000);

        }

        Serial.println("Done");
    #endif

    #ifdef TEST_ACCL
        Serial.println("Testing Accelerator");
        Serial.println("Zeroing");
        
        accel = new MCP4XXX(ACCEL_CS);

        for (int i = 0; i < 260; i++) {
            accel -> decrement();

        }

        for (int i = 0; i < 256; i++) {
            Serial.println("Increment " + String(i));
            accel -> increment();

            delay(100);
        }

        for (int i = 0; i < 256; i++) {
            Serial.println("Decrement " + String(i));
            accel -> decrement();

            delay(100);
        }

        Serial.println("Done");
    #endif

    #ifdef TEST_PEDAL_SW
        Serial.println("Tesing Pedal Switch");
        pinMode(PEDAL_SW, INPUT);
        pressed = false;
        attachInterrupt(digitalPinToInterrupt(PEDAL_SW), pedal_released, RISING);

        while(!pressed) {
            Serial.println("Press and Hold Pedal");
            delay(1000);
            
        }

        while(pressed) {
            Serial.println("Release Pedal");
            delay(1000);
            
        }

        Serial.println("Done");
    #endif

    #ifdef TEST_PEDAL_POT
        Serial.println("Tesing Pedal Potentiometer");

        #ifdef TEST_PEDAL_SW
            noInterrupts();
        #endif

        pinMode(PEDAL_POT, INPUT);

        for (int i = 0; i < 100; i++) {
            Serial.println("Pot Reading (" + String(i) + ") " + String(analogRead(PEDAL_POT)));
            delay(1000);

        }

        Serial.println("Done");

    #endif

}

void loop() {
    Serial.println("Test Bank Complete");
    delay(10000);

}

#ifdef TEST_PEDAL_SW

    void pedal_pressed() {
        pressed = true;
        Serial.println("Pedal Pressed");
        attachInterrupt(digitalPinToInterrupt(PEDAL_SW), pedal_released, RISING);

    }

    void pedal_released() {
        pressed = false;
        Serial.println("Pedal Released");
        attachInterrupt(digitalPinToInterrupt(PEDAL_SW), pedal_pressed, FALLING);

    }

#endif
