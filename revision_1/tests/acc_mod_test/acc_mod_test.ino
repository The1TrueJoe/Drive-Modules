#define TEST_RELAYS
#define TEST_PEDAL

#include <Arduino.h>

#ifdef TEST_RELAYS
    #define LEFT_TAIL_RELAY 8
    #define RIGHT_TAIL_RELAY 7
    #define TAIL_LIGHT_RELAY 6
    #define HEAD_LIGHT_RELAY 5
    #define REAR_BUZZ_RELAY 4
    #define HORN_RELAY 9
#endif

#ifdef TEST_PEDAL
    #define BRAKE_PEDAL 3
    volatile bool pressed;
#endif

void setup() {
    Serial.begin(115200);
    Serial.println("Testing Hardware.... DO NOT CONNECT TO LIVE CART!!");
    delay(2000);

    #ifdef TEST_RELAYS
        Serial.println("Testing Relays");

        Serial.println("Right Tail");
        pinMode(RIGHT_TAIL_RELAY, OUTPUT);
        for (int i = 0; i < 2; i++) {
            digitalWrite(RIGHT_TAIL_RELAY, HIGH);
            delay(1000);
            digitalWrite(RIGHT_TAIL_RELAY, LOW);
            delay(1000);

        }

        Serial.println("Left Tail");
        pinMode(LEFT_TAIL_RELAY, OUTPUT);
        for (int i = 0; i < 2; i++) {
            digitalWrite(LEFT_TAIL_RELAY, HIGH);
            delay(1000);
            digitalWrite(LEFT_TAIL_RELAY, LOW);
            delay(1000);

        }

        Serial.println("Tail");
        pinMode(TAIL_LIGHT_RELAY, OUTPUT);
        for (int i = 0; i < 2; i++) {
            digitalWrite(TAIL_LIGHT_RELAY, HIGH);
            delay(1000);
            digitalWrite(TAIL_LIGHT_RELAY, LOW);
            delay(1000);

        }

        Serial.println("Head");
        pinMode(HEAD_LIGHT_RELAY, OUTPUT);
        for (int i = 0; i < 2; i++) {
            digitalWrite(HEAD_LIGHT_RELAY, HIGH);
            delay(1000);
            digitalWrite(HEAD_LIGHT_RELAY, LOW);
            delay(1000);

        }

        Serial.println("Horn");
        pinMode(HORN_RELAY, OUTPUT);
        for (int i = 0; i < 2; i++) {
            digitalWrite(HORN_RELAY, HIGH);
            delay(1000);
            digitalWrite(HORN_RELAY, LOW);
            delay(1000);

        }

        Serial.println("Buzz");
        pinMode(REAR_BUZZ_RELAY, OUTPUT);
        for (int i = 0; i < 2; i++) {
            digitalWrite(REAR_BUZZ_RELAY, HIGH);
            delay(1000);
            digitalWrite(REAR_BUZZ_RELAY, LOW);
            delay(1000);

        }

        Serial.println("Done");
    #endif

    #ifdef TEST_PEDAL
        Serial.println("Tesing Pedal Switch");
        pinMode(BRAKE_PEDAL, INPUT);
        pressed = false;
        attachInterrupt(digitalPinToInterrupt(BRAKE_PEDAL), pedal_pressed, RISING);

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

}

void loop() {
    Serial.println("Test Bank Complete");
    delay(10000);

}

#ifdef TEST_PEDAL

    void pedal_pressed() {
        pressed = true;
        Serial.println("Pedal Pressed");
        attachInterrupt(digitalPinToInterrupt(BRAKE_PEDAL), pedal_released, RISING);

    }

    void pedal_released() {
        pressed = false;
        Serial.println("Pedal Released");
        attachInterrupt(digitalPinToInterrupt(BRAKE_PEDAL), pedal_pressed, FALLING);

    }

#endif