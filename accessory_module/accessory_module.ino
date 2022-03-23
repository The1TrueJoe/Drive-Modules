/**
 * @file accessory_module.ino
 * 
 * @author Joseph Telaak
 * 
 * @brief Accessory control module for the golf cart
 * 
 * @version 0.1
 * @date 2022-03-22
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "module.h"

// --------- Definitions

// Relays
#define LEFT_TAIL_RELAY 8
#define RIGHT_TAIL_RELAY 7
#define TAIL_LIGHT_RELAY 6
#define HEAD_LIGHT_RELAY 5
#define REAR_BUZZ_RELAY 4
#define HORN_RELAY 9

// Relay IDs
const uint8_t right_tail_id = 0x00;
const uint8_t left_tail_id = 0x01;
const uint8_t head_light_id = 0x02;
const uint8_t tail_light_id = 0x03;
const uint8_t horn_id = 0x04;
const uint8_t rear_buzz_id = 0x05;

// Brake Pedal
#define BRAKE_PEDAL 3

// Time
volatile bool continue_loop = true;
volatile int def_blinking_interval = 2000;
volatile int def_horn_interval = 50;

// --------- Arduino

/**
 * @brief Main setup
 * 
 */

void setup() {
    // CAN ID
    m_can_id = accessory_module_default_address;

    // Standard module setup
    standardModuleSetup();

    // Announce Ready
    ready();
    holdTillEnabled();

    // Setup Interupts
    attachInterupt(digitalPinToInterrupt(CAN_INT), canLoop, FALLING);
    attachInterupt(digitalPinToInterrupt(BRAKE_PEDAL), pedalPressed, RISING);

    // Relay setup
    setupRelays();

}

/**
 * @brief Main loop
 * 
 */

void loop() {
    // Periodic updates
    postRelays();
    delay(10000);

}

/**
 * @brief CAN Message Processing
 * 
 */

void canLoop() {
    // Get message
    if (!getCANMessage()) { return: }

    standardModuleLoopHead();

    switch (can_msg_in.data[0]) {
        case 0x0A:
            switch (can_msg_in.data[2]) {
                case 0x01:
                    openRelay(can_msg_in.data[1]);
                    break;

                case 0x02:
                    closeRelay(can_msg_in.data[1]);
                    continue_loop = false;
                    break;

                default:
                    break;

            }

            break;
            

        case 0x0B:
            switch (can_msg_in.data[1]) {
                case 0x0B:
                    int interval = (can_msg_in.data[3] != 0x00) ? (convertToInt(can_msg_in.data[3]) * convertToInt(can_msg_in.data[4])) : def_blink_interval;
                    int id = can_msg_in.data[2];

                    while (continue_loop) {
                        closeRelay(id);
                        delay(interval);
                        openRelay(id);

                    }

                    break;


                case 0x01:
                    int interval = (can_msg_in.data[3] != 0x00) ? (convertToInt(can_msg_in.data[3]) * convertToInt(can_msg_in.data[4])) : def_horn_interval;

                    closeRelay(horn_id);
                    delay(interval);
                    openRelay(horn_id);

                    break;
                    
                default:
                    break;

            }

            break;


        case 0x0C:
            switch (can_msg_in.data[1]) {
                case 0x0A:
                    postRelayStatus(can_msg_in.data[2]);
                    break;

                case 0x0E:
                    switch (can_msg_in.data[2]) {
                        case 0x01:
                            def_blink_interval = (can_msg_in.data[3] != 0x00) ? (convertToInt(can_msg_in.data[3]) * convertToInt(can_msg_in.data[4])) : def_blink_interval;
                            break;

                        case 0x02:
                            def_horn_interval = (can_msg_in.data[3] != 0x00) ? (convertToInt(can_msg_in.data[3]) * convertToInt(can_msg_in.data[4])) : def_horn_interval;
                            break;

                        default:
                            break;
                            
                    }

                    break;

                default:
                    break;

            }

            break;
            
        default:
            break;

    }

    standardModuleLoopTail();
    
}

// --------- Pedals 

/** @brief When pedal is pressed */
void pedalPressed() {
    noInterrupts();

    closeRelay(tail_light_id);
    attachInterupt(BRAKE_PEDAL, pedalReleased, FALLING);

    interupts();

}

/** @brief When pedal is released */
void pedalReleased() {
    openRelay(tail_light_id);
    attachInterupt(BRAKE_PEDAL, pedalPressed, RISING);

}

// --------- Relay control

/** @brief Setup the relays  */
void setupRelays() {
    // Set pinmode
    Serial.println("Relays: Setting up Relay Control Pins");
    pinMode(RIGHT_TAIL_RELAY, OUTPUT);
    pinMode(LEFT_TAIL_RELAY, OUTPUT);
    pinMode(TAIL_LIGHT_RELAY, OUTPUT);
    pinMode(HEAD_LIGHT_RELAY, OUTPUT);
    pinMode(REAR_BUZZ_RELAY, OUTPUT);
    pinMode(HORN_RELAY, OUTPUT);

    // Reset relays
    resetRelays();

    // Setup complete
    Serial.println("Relays: Setup Complete");

}

/** @brief Reset all relays */
void resetRelays() {
    closeRelay(right_tail_id);
    closeRelay(left_tail_id);
    closeRelay(tail_light_id);
    closeRelay(head_light_id);
    closeRelay(horn_id);
    closeRelay(rear_buzz_id);

}

/**
 * @brief Close a relay
 * 
 * @param id relay id
 */

void closeRelay(uint8_t id) {
    switch (id) {
        case right_tail_id:
            Serial.println("Right Tail Light Relay: Close");
            digitalWrite(RIGHT_TAIL_RELAY, LOW); 
            break;

        case left_tail_id:
            Serial.println("Left Tail Light Relay: Close");
            digitalWrite(LEFT_TAIL_RELAY, LOW); 
            break;

        case tail_light_id:
            Serial.println("Head Light Relay: Close");
            digitalWrite(TAIL_LIGHT_RELAY, LOW);
            break;

        case head_light_id:
            Serial.println("Head Light Relay: Close");
            digitalWrite(HEAD_LIGHT_RELAY, LOW); 
            break;

        case horn_relay_id:
            Serial.println("Horn Relay: Close");
            digitalWrite(HORN_RELAY, LOW); 
            break;

        case rear_buzz_id:
            Serial.println("Rear Buzzer Relay: Close");
            digitalWrite(REAR_BUZZ_RELAY, LOW);
            break;

        default:
            break;

    }

    postRelayStatus(id);

}

/**
 * @brief Open a relay
 * 
 * @param id relay id
 */

void openRelay(uint8_t id) {
    switch (id) {
        case right_tail_id:
            Serial.println("Right Tail Light Relay: Open");
            digitalWrite(RIGHT_TAIL_RELAY, HIGH); 
            break;

        case left_tail_id:
            Serial.println("Left Tail Light Relay: Open");
            digitalWrite(LEFT_TAIL_RELAY, HIGH); 
            break;

        case tail_light_id:
            Serial.println("Head Light Relay: Open");
            digitalWrite(TAIL_LIGHT_RELAY, HIGH);
            break;

        case head_light_id:
            Serial.println("Head Light Relay: Open");
            digitalWrite(HEAD_LIGHT_RELAY, HIGH); 
            break;

        case horn_relay_id:
            Serial.println("Horn Relay: Open");
            digitalWrite(HORN_RELAY, HIGH); 
            break;

        case rear_buzz_id:
            Serial.println("Rear Buzzer Relay: Open");
            digitalWrite(REAR_BUZZ_RELAY, HIGH);
            break;

        default:
            break;

    }

    postRelayStatus(id);

}

/**
 * @brief Check the set position of a relay
 * 
 * @param id Relay id
 * 
 * @return true if the relay is set
 * @return false if the relay is not set
 */

bool checkRelay(uint8_t id) {
    switch (id) {
        case right_tail_id:
            return digitalRead(RIGHT_TAIL_RELAY) == HIGH;

        case left_tail_id:
            return digitalRead(LEFT_TAIL_RELAY) == HIGH;

        case tail_light_id:
            return digitalRead(TAIL_LIGHT_RELAY) == HIGH;

        case head_light_id:
            return digitalRead(HEAD_LIGHT_RELAY) == HIGH;

        case horn_relay_id:
            return digitalRead(HORN_RELAY) == HIGH;
        
        case rear_buzz_id:
            return digitalRead(REAR_BUZZ_RELAY) == HIGH;

        default:
            return false;

    }
}

/**
 * @brief Post relay status to the bus
 * 
 * @param id relay id
 */

void postRelayStatus(uint8_t id) {
    // Build Message
    uint8_t status = checkRelay(id) ? 0x01 : 0x02;
    uint8_t message[can_msg_in.dlc] = { 0x0C, 0x0C, 0x0A, id, status, 0x00, 0x00, 0x00 };

    // Send Message
    sendCANMessage(m_can_id, message);

}

/** @brief Post all relay statuses */
void postRelays() {
    postRelayStatus(right_tail_id);
    postRelayStatus(left_tail_id);
    postRelayStatus(tail_light_id);
    postRelayStatus(head_light_id);
    postRelayStatus(horn_id);
    postRelayStatus(rear_buzz_id);

}
