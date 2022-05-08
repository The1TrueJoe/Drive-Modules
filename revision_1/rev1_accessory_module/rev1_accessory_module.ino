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

#include "mcp2515.h"

// Relays
#define LEFT_TAIL_RELAY 8
#define RIGHT_TAIL_RELAY 7
#define TAIL_LIGHT_RELAY 6
#define HEAD_LIGHT_RELAY 5
#define REAR_BUZZ_RELAY 4
#define HORN_RELAY 9

// Releay ACT
#define RELAY_ACT LOW
#define RELAY_DEACT HIGH

// Relay IDs
#define right_tail_id 0x01
#define left_tail_id 0x02
#define head_light_id 0x03
#define tail_light_id 0x04
#define horn_id 0x05
#define rear_buzz_id 0x06

// Brake Pedal
#define BRAKE_PEDAL 3

// LEDS
#define COM_LED A0
#define ACT_LED A1

// CAN Pins
#define CAN_CS 10
#define CAN_INT 2

// CAN Message
#define CAN_ID 0x002
#define CAN_DLC 8

// CAN
MCP2515 can(CAN_CS);

// Blink statuses
volatile bool blink_right = false;
volatile bool blink_left = false;
volatile bool blink_head = false;
volatile bool blink_tail = false;
volatile bool honk_act = false;

/**
 * @brief Main setup
 * 
 */

void setup() {
    // Setup LEDS
    pinMode(ACT_LED, OUTPUT);
    pinMode(COM_LED, OUTPUT);

    // Init Hold and Display
    digitalWrite(ACT_LED, HIGH);
    digitalWrite(COM_LED, HIGH);
    delay(1000);
    digitalWrite(ACT_LED, HIGH);
    digitalWrite(COM_LED, LOW);
    delay(200);

    // Setup
    digitalWrite(ACT_LED, HIGH);

    // Setup CAN
    can.reset();
    can.setBitrate(CAN_125KBPS);
    can.setNormalMode();

    // Relay setup
    pinMode(RIGHT_TAIL_RELAY, OUTPUT);
    pinMode(LEFT_TAIL_RELAY, OUTPUT);
    pinMode(TAIL_LIGHT_RELAY, OUTPUT);
    pinMode(HEAD_LIGHT_RELAY, OUTPUT);
    pinMode(REAR_BUZZ_RELAY, OUTPUT);
    pinMode(HORN_RELAY, OUTPUT);

    // Reset relays
    resetRelays();

    // Flash head and tail lights
    closeRelay(tail_light_id);
    closeRelay(head_light_id);
    delay(500);
    openRelay(tail_light_id);
    openRelay(head_light_id);

    // LED Off
    digitalWrite(ACT_LED, LOW);

    // Interrupts
    attachInterrupt(digitalPinToInterrupt(CAN_INT), can_irq, FALLING);
    attachInterrupt(digitalPinToInterrupt(BRAKE_PEDAL), pedal_act, RISING);

}

/**
 * @brief Perodic updates and blinking/honking
 * 
 */

void loop() {
    digitalWrite(ACT_LED, HIGH);

    if (honk_act) {
       delay(200);
       openRelay(horn_id);

    }

    if (blink_right) { closeRelay(right_tail_id); }
    if (blink_left) { closeRelay(left_tail_id); }
    if (blink_head) { closeRelay(head_light_id); }
    if (blink_tail) { closeRelay(tail_light_id); }

    digitalWrite(ACT_LED, LOW);

    delay(2000);

    digitalWrite(ACT_LED, HIGH);

    if (blink_right) { openRelay(right_tail_id); }
    if (blink_left) { openRelay(left_tail_id); }
    if (blink_head) { openRelay(head_light_id); }
    if (blink_tail) { openRelay(tail_light_id); }

    digitalWrite(ACT_LED, LOW);

    delay(2000);

    digitalWrite(ACT_LED, HIGH);
    postRelays();
    digitalWrite(ACT_LED, LOW);
    
}

/**
 * @brief CAN Message Processing
 * 
 */

void can_irq() {
    // Message buffer
    struct can_frame can_msg_in;

    // Check message
    if (can.readMessage(&can_msg_in) == MCP2515::ERROR_OK) {
        digitalWrite(COM_LED, HIGH);

        // Check ID
        if (can_msg_in.can_id == CAN_ID) {

            if (can_msg_in.data[0] == 0x0A) {
                if (can_msg_in.data[2] == 0x01) {
                    closeRelay(can_msg_in.data[1]);

                } else if (can_msg_in.data[2] == 0x01) {
                    openRelay(can_msg_in.data[1]);

                    if (blink_right && can_msg_in.data[1] == right_tail_id)
                        blink_right = false;
                    else if (blink_left && can_msg_in.data[1] == left_tail_id)
                        blink_left = false;
                    else if (blink_head && can_msg_in.data[1] == head_light_id)
                        blink_head = false;
                    else if (blink_tail && can_msg_in.data[1] == tail_light_id)
                        blink_tail = false;

                }

                postRelayStatus(can_msg_in.data[1]);
                
            } else if (can_msg_in.data[0] == 0x0B) {
                if (can_msg_in.data[1] == 0x01) {
                    openRelay(horn_id);
                    honk_act = true;
                }
                

            } else if (can_msg_in.data[0] == 0x0C) {
                if (can_msg_in.data[1] == 0x0A) {
                    postRelayStatus(can_msg_in.data[2]);

                }
            }
        }

        // Clear the message buffer
        can_msg_in.data[0] = 0;
        can_msg_in.data[1] = 0;
        can_msg_in.data[2] = 0;
        can_msg_in.data[3] = 0;
        can_msg_in.data[4] = 0;
        can_msg_in.data[5] = 0;
        can_msg_in.data[6] = 0;
        can_msg_in.data[7] = 0;

        digitalWrite(COM_LED, LOW);

    }
}

// --------- Pedals 

/**
 * @brief Pedal is pressed
 * 
 */

void pedal_act() {
    noInterrupts();

    digitalWrite(COM_LED, HIGH);

    struct can_frame can_msg_out;

    can_msg_out.can_id = CAN_ID;
    can_msg_out.can_dlc = CAN_DLC;
    can_msg_out.data[0] = 0x0C;
    can_msg_out.data[1] = 0x0C;
    can_msg_out.data[2] = 0x0E;
    can_msg_out.data[3] = 0;
    can_msg_out.data[4] = 0;
    can_msg_out.data[5] = 0;
    can_msg_out.data[6] = 0;
    can_msg_out.data[7] = 0x02;

    can.sendMessage(&can_msg_out);
    interrupts();
    attachInterrupt(digitalPinToInterrupt(BRAKE_PEDAL), pedal_deact, FALLING);

    digitalWrite(COM_LED, LOW);

}

/**
 * @brief Pedal is released
 * 
 */

void pedal_deact() {
    noInterrupts();

    digitalWrite(COM_LED, HIGH);

    struct can_frame can_msg_out;

    can_msg_out.can_id = CAN_ID;
    can_msg_out.can_dlc = CAN_DLC;
    can_msg_out.data[0] = 0x0C;
    can_msg_out.data[1] = 0x0C;
    can_msg_out.data[2] = 0x0E;
    can_msg_out.data[3] = 0;
    can_msg_out.data[4] = 0;
    can_msg_out.data[5] = 0;
    can_msg_out.data[6] = 0;
    can_msg_out.data[7] = 0x01;

    can.sendMessage(&can_msg_out);

    interrupts();
    attachInterrupt(digitalPinToInterrupt(BRAKE_PEDAL), pedal_act, RISING);

    digitalWrite(COM_LED, LOW);

}

// --------- Relay control

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
            digitalWrite(RIGHT_TAIL_RELAY, RELAY_DEACT); 
            break;

        case left_tail_id:
            digitalWrite(LEFT_TAIL_RELAY, RELAY_DEACT); 
            break;

        case tail_light_id:
            digitalWrite(TAIL_LIGHT_RELAY, RELAY_DEACT);
            break;

        case head_light_id:
            digitalWrite(HEAD_LIGHT_RELAY, RELAY_DEACT); 
            break;

        case horn_id:
            digitalWrite(HORN_RELAY, RELAY_DEACT); 
            break;

        case rear_buzz_id:
            digitalWrite(REAR_BUZZ_RELAY, RELAY_DEACT);
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
            digitalWrite(RIGHT_TAIL_RELAY, RELAY_ACT); 
            break;

        case left_tail_id:
            digitalWrite(LEFT_TAIL_RELAY, RELAY_ACT); 
            break;

        case tail_light_id:
            digitalWrite(TAIL_LIGHT_RELAY, RELAY_ACT);
            break;

        case head_light_id:
            digitalWrite(HEAD_LIGHT_RELAY, RELAY_ACT); 
            break;

        case horn_id:
            digitalWrite(HORN_RELAY, RELAY_ACT); 
            break;

        case rear_buzz_id:
            digitalWrite(REAR_BUZZ_RELAY, RELAY_ACT);
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
            return digitalRead(RIGHT_TAIL_RELAY) == RELAY_ACT;

        case left_tail_id:
            return digitalRead(LEFT_TAIL_RELAY) == RELAY_ACT;

        case tail_light_id:
            return digitalRead(TAIL_LIGHT_RELAY) == RELAY_ACT;

        case head_light_id:
            return digitalRead(HEAD_LIGHT_RELAY) == RELAY_ACT;

        case horn_id:
            return digitalRead(HORN_RELAY) == RELAY_ACT;
        
        case rear_buzz_id:
            return digitalRead(REAR_BUZZ_RELAY) == RELAY_ACT;

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
    digitalWrite(COM_LED, HIGH);

    struct can_frame can_msg_out;

    can_msg_out.can_id = CAN_ID;
    can_msg_out.can_dlc = CAN_DLC;
    can_msg_out.data[0] = 0x0C;
    can_msg_out.data[1] = 0x0C;
    can_msg_out.data[2] = 0x0A;
    can_msg_out.data[3] = id;
    can_msg_out.data[4] = checkRelay(id) ? 0x01 : 0x02;
    can_msg_out.data[5] = 0;
    can_msg_out.data[6] = 0;
    can_msg_out.data[7] = 0;

    can.sendMessage(&can_msg_out);
    digitalWrite(COM_LED, LOW);

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
