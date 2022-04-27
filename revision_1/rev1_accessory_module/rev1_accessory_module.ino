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
#include "port_config.h"

// Time
volatile bool continue_loop = true;
volatile int def_blink_interval = 2000;
volatile int def_horn_interval = 50;

// --------- Arduino

/**
 * @brief Main setup
 * 
 */

void setup() {
    // Standard module setup
    standardModuleSetup(10, 0xFF2);

    // Announce Ready
    ready();
    holdTillEnabled();

    // Setup Interupts
    attachInterrupt(digitalPinToInterrupt(Default_CAN_INT), canLoop, FALLING);
    attachInterrupt(digitalPinToInterrupt(BRAKE_PEDAL), pedalPressed, RISING);

    // Relay setup
    setupRelays();

}

/**
 * @brief Main loop
 * 
 */

void loop() {
    switch (can_adapter -> can_msg_in.data[0]) {
        case 0x0B:
            switch (can_adapter -> can_msg_in.data[1]) {
                case 0x0B:
                    int m_interval = (can_adapter -> can_msg_in.data[3] != 0x00) ? (convertToInt(can_adapter -> can_msg_in.data[3]) * convertToInt(can_adapter -> can_msg_in.data[4])) : def_blink_interval;
                    int id = can_adapter -> can_msg_in.data[2];

                    while (continue_loop) {
                        closeRelay(id);
                        delay(m_interval);
                        openRelay(id);

                    }

                    break;


                case 0x01:
                    int n_interval = (can_adapter -> can_msg_in.data[3] != 0x00) ? (convertToInt(can_adapter -> can_msg_in.data[3]) * convertToInt(can_adapter -> can_msg_in.data[4])) : def_horn_interval;

                    closeRelay(horn_id);
                    delay(n_interval);
                    openRelay(horn_id);

                    break;
                    
                default:
                    break;

            }

            break;

        default:
            break;

    }
}

/**
 * @brief CAN Message Processing
 * 
 */

void canLoop() {
    // Get message
    if (!can_adapter -> getCANMessage()) { return; }

    standardModuleLoopHead();

    switch (can_adapter -> can_msg_in.data[0]) {
        case 0x0A:
            switch (can_adapter -> can_msg_in.data[2]) {
                case 0x01:
                    openRelay(can_adapter -> can_msg_in.data[1]);
                    break;

                case 0x02:
                    closeRelay(can_adapter -> can_msg_in.data[1]);
                    continue_loop = false;
                    break;

                default:
                    break;

            }

            break;

        case 0x0C:
            switch (can_adapter -> can_msg_in.data[1]) {
                case 0x0A:
                    postRelayStatus(can_adapter -> can_msg_in.data[2]);
                    break;

                case 0x0E:
                    switch (can_adapter -> can_msg_in.data[2]) {
                        case 0x01:
                            def_blink_interval = (can_adapter -> can_msg_in.data[3] != 0x00) ? (convertToInt(can_adapter -> can_msg_in.data[3]) * convertToInt(can_adapter -> can_msg_in.data[4])) : def_blink_interval;
                            break;

                        case 0x02:
                            def_horn_interval = (can_adapter -> can_msg_in.data[3] != 0x00) ? (convertToInt(can_adapter -> can_msg_in.data[3]) * convertToInt(can_adapter -> can_msg_in.data[4])) : def_horn_interval;
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
    closeRelay(tail_light_id);
    attachInterrupt(BRAKE_PEDAL, pedalReleased, FALLING);

}

/** @brief When pedal is released */
void pedalReleased() {
    openRelay(tail_light_id);
    attachInterrupt(BRAKE_PEDAL, pedalPressed, RISING);

}

// --------- Relay control

/** @brief Setup the relays  */
void setupRelays() {
    #ifdef DEBUG
        // Set pinmode
        Serial.println("Relays: Setting up Relay Control Pins");
    #endif

    pinMode(RIGHT_TAIL_RELAY, OUTPUT);
    pinMode(LEFT_TAIL_RELAY, OUTPUT);
    pinMode(TAIL_LIGHT_RELAY, OUTPUT);
    pinMode(HEAD_LIGHT_RELAY, OUTPUT);
    pinMode(REAR_BUZZ_RELAY, OUTPUT);
    pinMode(HORN_RELAY, OUTPUT);

    // Reset relays
    resetRelays();

    #ifdef DEBUG
        // Setup complete
        Serial.println("Relays: Setup Complete");
    #endif

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
            digitalWrite(RIGHT_TAIL_RELAY, LOW); 
            break;

        case left_tail_id:
            digitalWrite(LEFT_TAIL_RELAY, LOW); 
            break;

        case tail_light_id:
            digitalWrite(TAIL_LIGHT_RELAY, LOW);
            break;

        case head_light_id:
            digitalWrite(HEAD_LIGHT_RELAY, LOW); 
            break;

        case horn_id:
            digitalWrite(HORN_RELAY, LOW); 
            break;

        case rear_buzz_id:
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
            digitalWrite(RIGHT_TAIL_RELAY, HIGH); 
            break;

        case left_tail_id:
            digitalWrite(LEFT_TAIL_RELAY, HIGH); 
            break;

        case tail_light_id:
            digitalWrite(TAIL_LIGHT_RELAY, HIGH);
            break;

        case head_light_id:
            digitalWrite(HEAD_LIGHT_RELAY, HIGH); 
            break;

        case horn_id:
            digitalWrite(HORN_RELAY, HIGH); 
            break;

        case rear_buzz_id:
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

        case horn_id:
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
    #ifdef DEBUG
        // Build Message
        if (checkRelay(id)) {
            Serial.println("Relay " + String(id) + " is Open");
            uint8_t status = 0x01;

        } else {
            Serial.println("Relay " + String(id) + " is Closed");
            uint8_t status = 0x02;

        }
        
    #else
        // Build Message
        uint8_t status = checkRelay(id) ? 0x01 : 0x02;

    #endif

    // Build Message
    uint8_t message[can_adapter -> m_can_dlc] = { 0x0C, 0x0C, 0x0A, id, status, 0x00, 0x00, 0x00 };

    // Send Message
    can_adapter -> sendCANMessage(can_adapter -> m_can_id, message);

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
