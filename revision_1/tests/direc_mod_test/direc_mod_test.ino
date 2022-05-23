#define TEST_STEERING
#define TEST_BRAKES
#define TEST_WHEEL

#include <BTS7960.h>

#ifdef TEST_STEERING
    // Steering Motor Ctrl
    #define STR_L_PWM 6
    #define STR_R_PWM 9
    #define STR_ENABLE 4

    // Steering Linear Actuator Potentiometer
    #define STR_POT A5

    // Motor Controller
    BTS7960 steering_motor(STR_ENABLE, STR_L_PWM, STR_R_PWM);

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

    // Motor Controller
    BTS7960 brake_motor(BRK_ENABLE, BRK_L_PWM, BRK_R_PWM);

#endif

void setup() {
    Serial.begin(115200);
    Serial.println("Testing Hardware.... DO NOT CONNECT TO LIVE CART!!");
    delay(2000);

    #ifdef TEST_STEERING
        Serial.println("Testing Steering");

        steering_motor.Disable();
        steering_motor.Stop();

        Serial.println("Running L PWM");

        steering_motor.TurnLeft(255);
        steering_motor.Enable();
        
        for (int i = 0; i < 50; i++) {
            Serial.println("Pot Reading: " + String(analogRead(STR_POT)));
            delay(100);
        }

        steering_motor.Disable();
        steering_motor.Stop();

        Serial.println("Running R PWM");

        steering_motor.TurnRight(255);
        steering_motor.Enable();
        
        for (int i = 0; i < 50; i++) {
            Serial.println("Pot Reading: " + String(analogRead(STR_POT)));
            delay(100);
        }

        steering_motor.Disable();
        steering_motor.Stop();

        Serial.println("Done");

    #endif

    #ifdef TEST_BRAKES
        Serial.println("Testing Brakes");

        brake_motor.Disable();
        brake_motor.Stop();

        Serial.println("Running L PWM");

        brake_motor.TurnLeft(255);
        brake_motor.Enable();
        
        for (int i = 0; i < 50; i++) {
            Serial.println("Pot Reading: " + String(analogRead(BRK_POT)));
            delay(100);
        }

        brake_motor.Disable();
        brake_motor.Stop();

        Serial.println("Running R PWM");

        brake_motor.TurnRight(255);
        brake_motor.Enable();
        
        for (int i = 0; i < 50; i++) {
            Serial.println("Pot Reading: " + String(analogRead(BRK_POT)));
            delay(100);
        }

        brake_motor.Disable();
        brake_motor.Stop();
        
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
