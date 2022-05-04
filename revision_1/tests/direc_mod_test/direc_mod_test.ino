#define TEST_STEERING
#define TEST_BRAKES
#define TEST_WHEEL

#ifdef TEST_STEERING
    // Steering Motor Ctrl
    #define STR_L_PWM 9
    #define STR_R_PWM 6
    #define STR_ENABLE 4

    // Steering Linear Actuator Potentiometer
    #define STR_POT A5

#endif

#ifdef TEST_WHEEL
    #include "Encoder.h"

    // Steering Wheel Input Encoder
    #define STR_WHL_ENC 3
    #define STR_WHL_ENC2 A3

    Encoder wheel_enc(STR_WHL_ENC, STR_WHL_ENC2);

#endif

#ifdef TEST_BRAKES
    // Brake Motor Ctrl
    #define BRK_L_PWM 5
    #define BRK_R_PWM 10
    #define BRK_ENABLE 7

    // Brake Actuator Potentiometer
    #define BRK_POT A4

#endif

void setup() {
    Serial.begin(115200);
    Serial.println("Testing Hardware.... DO NOT CONNECT TO LIVE CART!!");
    delay(2000);

    #ifdef TEST_STEERING
        pinMode(STR_ENABLE, OUTPUT);
        pinMode(STR_L_PWM, OUTPUT);
        pinMode(STR_R_PWM, OUTPUT);
        pinMode(STR_POT, INPUT);

        digitalWrite(STR_ENABLE, LOW);
        digitalWrite(STR_L_PWM, LOW);
        digitalWrite(STR_R_PWM, LOW);

        Serial.println("Running L PWM");

        digitalWrite(STR_L_PWM, 255);
        digitalWrite(STR_R_PWM, LOW);
        
        for (int i = 0; i < 50; i++) {
            Serial.println("Pot Reading: " + String(analogRead(STR_POT)));
            delay(100);
        }

        Serial.println("Running R PWM");

        digitalWrite(STR_L_PWM, LOW);
        digitalWrite(STR_R_PWM, 255);
        
        for (int i = 0; i < 50; i++) {
            Serial.println("Pot Reading: " + String(analogRead(STR_POT)));
            delay(100);
        }

        Serial.println("Done");

    #endif

    #ifdef TEST_BRAKES
        Serial.println("Testing Brakes")

        pinMode(BRK_ENABLE, OUTPUT);
        pinMode(BRK_L_PWM, OUTPUT);
        pinMode(BRK_R_PWM, OUTPUT);
        pinMode(BRK_POT, INPUT);

        digitalWrite(BRK_ENABLE, LOW);
        digitalWrite(BRK_L_PWM, LOW);
        digitalWrite(BRK_R_PWM, LOW);

        Serial.println("Running L PWM");

        digitalWrite(BRK_L_PWM, 255);
        digitalWrite(BRK_R_PWM, LOW);
        
        for (int i = 0; i < 50; i++) {
            Serial.println("Pot Reading: " + String(analogRead(BRK_POT)));
            delay(100);
        }

        Serial.println("Running R PWM");

        digitalWrite(BRK_L_PWM, LOW);
        digitalWrite(BRK_R_PWM, 255);
        
        for (int i = 0; i < 50; i++) {
            Serial.println("Pot Reading: " + String(analogRead(BRK_POT)));
            delay(100);
        }

        Serial.println("Done");

    #endif

    #ifdef TEST_WHEEL
        Serial.println("Testing Steering Wheel");
        Serial.println("Please Turn Wheel");

        for (int i = 0; i < 1000; i++) {
            Serial.println("Encoder Pos: " + String(wheel_enc.readAndReset()));
            delay(10);

        }

        Serial.println("Done");

    #endif

}

void loop() {
    Serial.println("Test Bank Complete");
    delay(10000);

}