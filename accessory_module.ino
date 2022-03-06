#include "module.h"

// --------- Pin Definitions

#define RIGHT_TAIL_RELAY
#define LEFT_TAIL_RELAY
#define HEAD_LIGHT_RELAY
#define HORN_RELAY

// Relay IDs
const uint8_t right_tail_id = 0x00;
const uint8_t left_tail_id = 0x01;
const uint8_t head_light_id = 0x02;
const uint8_t horn_relay_id = 0x03;

// --------- Setup

/**
 * @brief Main setup
 * 
 */

void setup() {
    // Standard module setup
    standardModuleSetup();

    // Relay setup
    setupRelays();

}

/**
 * @brief Setup the relays
 * 
 */

void setupRelays() {
    // Set pinmode
    Serial.println("Relays: Setting up Relay Control Pins");
    pinMode(RIGHT_TAIL_RELAY, OUTPUT);
    pinMode(LEFT_TAIL_RELAY, OUTPUT);
    pinMode(HEAD_LIGHT_RELAY, OUTPUT);
    pinMode(HORN_RELAY, OUTPUT);

    // Reset relays
    resetRelays();

    // Setup complete
    Serial.println("Relays: Setup Complete");

}

// --------- Main Loop

/**
 * @brief Main loop
 * 
 */

void loop() {
    if (getCANMessage()) {
        standardModuleLoopHead();

        switch (can_msg_in.data[0]) {
            // 
            case 0x0C:
                relayControlSequence(); break;
                
        
            default:
                break;

        }

        standardModuleLoopTail();

    }

}

/**
 * @brief Relay control sequence
 * 
 */

void relayControlSequence() {
    uint8_t relay_id = can_msg_in.data[1];

    switch (can_msg_in.data[2]) {
        case 0x0A:
            switch (can_msg_in.data[3]) {
                case 0x00:
                    closeRelay(relay_id); break;
                case 0x01:
                    openRelay(relay_id); break;
                case 0x02:
                    postRelayStatus(relay_id); break;
                default:
                    invalidCommand(); break;

            }
            
            break;

        case 0x0C:
            switch (can_msg_in.data[3]) {
                case 0x00:
                    openRelay(relay_id); break;
                case 0x01:
                    closeRelay(relay_id); break;
                default:
                    invalidCommand(); break;

            }

            break;

        default:
            invalidCommand(); break;

    }

}

// --------- Relay control

/** @brief Reset all relays */
void resetRelays() {
    for (uint8_t i = 0x00; i < 0x04; i++) {
        closeRelay(i);

    }
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

        case head_light_id:
            Serial.println("Head Light Relay: Close");
            digitalWrite(HEAD_LIGHT_RELAY, LOW); 
            break;

        case horn_relay_id:
            Serial.println("Horn Relay: Close");
            digitalWrite(HORN_RELAY, LOW); 
            break;

        default:
            invalidCommand(); break;

    }
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

        case head_light_id:
            Serial.println("Head Light Relay: Open");
            digitalWrite(HEAD_LIGHT_RELAY, HIGH); 
            break;

        case horn_relay_id:
            Serial.println("Horn Relay: Open");
            digitalWrite(HORN_RELAY, HIGH); 
            break;

        default:
            invalidCommand(); break;

    }
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
            bool pos = (digitalRead(RIGHT_TAIL_RELAY) == 1);
            Serial.println("Right Tail Light Relay: Currently " + (pos ? "Open" : "Closed"));
            return pos;

        case left_tail_id:
            bool pos = (digitalRead(LEFT_TAIL_RELAY) == 1);
            Serial.println("Left Tail Light Relay: Currently " + (pos ? "Open" : "Closed"));
            return pos;

        case head_light_id:
            bool pos = (digitalRead(HEAD_LIGHT_RELAY) == 1);
            Serial.println("Head Light Relay: Currently " + (pos ? "Open" : "Closed"));
            return pos;

        case horn_relay_id:
            bool pos = (digitalRead(HORN_RELAY) == 1);
            Serial.println("Horn Relay: Currently " + (pos ? "Open" : "Closed"));
            return pos;
        
        default:
            invalidCommand();
            return false;

    }
}

/**
 * @brief Post relay status to the bus
 * 
 * @param id relay id
 */

void postRelayStatus(uint8_t id) {
    uint8_t status = checkRelay(id) ? 0x01 : 0x00;
    uint8_t message = { 0x0C, id, 0x0A, 0x02, status, 0x00, 0x00, 0x00 };

    sendCANMessage(master_can_id, message);

}