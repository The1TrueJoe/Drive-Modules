/**
 * @file rev1_drive_module.ino
 * 
 * @author Joseph Telaak
 * 
 * @brief This module controls the golf cart's motor control unit.
 *          It is responsible for switching between forward and reverse.
 *          It reports input from the accelerator pedal and ouputs a potentiometer
 *          signal to the motor control unit to act as the "pedal"
 * 
 * @version 0.1
 * 
 * @date 2022-04-28
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "mcp2515.h"
#include "mcp4xxx.h"

// Accelerator CS
#define ACCEL_CS 9

// Solenoid control
#define ACT_SW 4
#define FWD_REV_SEL 5

// Pedal
#define PEDAL_POT A3
#define PEDAL_SW 3
#define PEDAL_EN_HEADER 8

// Releay ACT
#define RELAY_ACT HIGH
#define RELAY_DEACT LOW

// LEDs
#define COM_LED A0
#define ACT_LED A1
#define PEDAL_LED A2

// CAN Pins
#define CAN_CS 10
#define CAN_INT 2

// CAN Messages
#define CAN_ID 0x003
#define CAN_DLC 8

// Digital Potentiometer
using namespace icecave::arduino;
volatile MCP4XXX* accel;
volatile int wiper_pos = 0;

// CAN
MCP2515 can(CAN_CS);

// Pedal status
volatile bool pedal_pressed = false;
volatile bool pedal_detect_enable = false;

/**
 * @brief Main setup
 * 
 */

void setup() {
    // Setup LEDS
    pinMode(ACT_LED, OUTPUT);
    pinMode(COM_LED, OUTPUT);
    pinMode(PEDAL_LED, OUTPUT);

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

    // Switches
    pinMode(ACT_SW, OUTPUT);
    pinMode(FWD_REV_SEL, OUTPUT);

    // Reset
    digitalWrite(ACT_SW, RELAY_DEACT);
    digitalWrite(FWD_REV_SEL, RELAY_DEACT);

    // Pedal
    pinMode(PEDAL_EN_HEADER, INPUT);
    pedal_detect_enable = digitalRead(PEDAL_EN_HEADER) == HIGH;

    if (pedal_detect_enable) {
        pinMode(PEDAL_POT, INPUT);
        pinMode(PEDAL_SW, INPUT);

        for (int i = 0; i < 4; i++) {
            digitalWrite(PEDAL_LED, HIGH);
            delay(200);
            digitalWrite(PEDAL_LED, LOW);
            delay(200);

        }
    }

    // Digital Accel
    accel = new MCP4XXX(ACCEL_CS);
    pot_zero();

    // Announce ready to CAN bus
    announce();

    // LED Low
    digitalWrite(ACT_LED, LOW);

    // Interrupts
    attachInterrupt(digitalPinToInterrupt(CAN_INT), can_irq, FALLING);

}

// Timer counter
int counter = 0;

/**
 * @brief Peroidic update loop
 * 
 */

void loop() {
    if (pedal_detect_enable) {
        if (digitalRead(PEDAL_SW) == HIGH) {
            digitalWrite(ACT_LED, HIGH);
            pedal_act();

            while (digitalRead(PEDAL_SW) == HIGH) { 
                get_pedal_pos(); 
                
                if (counter % 5 == 0) { get_wiper_pos(); }

                delay(400);
                counter++;
            
            } 

            pedal_deact();
            digitalWrite(ACT_LED, LOW);

        }
    }
    
    if (counter % 100 == 0) { 
        get_wiper_pos();   

        if (counter >= 200 == 0) {
            digitalWrite(ACT_LED, HIGH);

            get_direc(); 
            get_en_status(); 

            counter = 0;

            digitalWrite(ACT_LED, LOW);

        }
    }

    delay(100);
    counter++;

}

/**
 * @brief CAN Message Processing
 * 
 */

void can_irq() {
    // Message
    struct can_frame can_msg_in;

    // Check message
    if (can.readMessage(&can_msg_in) == MCP2515::ERROR_OK) {
        digitalWrite(ACT_LED, HIGH);

        // Check ID
        if (can_msg_in.can_id == CAN_ID) {

            if (can_msg_in.data[0] == 0x0A) {
                if (can_msg_in.data[1] == 0x0A) {
                    if (can_msg_in.data[2] == 0x0A) {
                        pot_write(can_msg_in.data[3]);
                        get_wiper_pos();

                    } else if (can_msg_in.data[2] == 0x0E) {
                        if (can_msg_in.data[3] == 0x01)
                            digitalWrite(ACT_SW, LOW);
                        else if (can_msg_in.data[3] == 0x02) 
                            digitalWrite(ACT_SW, HIGH);
                        else
                            get_en_status();

                    }

                } else if (can_msg_in.data[1] == 0x0D) {
                    if (can_msg_in.data[2] == 0x01) 
                        digitalWrite(FWD_REV_SEL, LOW);
                    else if (can_msg_in.data[2] == 0x02) 
                        digitalWrite(FWD_REV_SEL, HIGH);
                    else
                        get_direc();

                }

            }
        } else if (can_msg_in.data[0] == 0x0B) {
            if (can_msg_in.data[1] == 0x0A) {
                if (can_msg_in.data[2] == 0x01) 
                    accel -> increment();
                else if (can_msg_in.data[2] == 0x02) 
                    accel -> decrement();
                else
                    get_wiper_pos();

            }

        } else if (can_msg_in.data[0] == 0x0C) {
            if (can_msg_in.data[1] == 0x0A) {
                if (can_msg_in.data[2] == 0x0A) 
                    get_wiper_pos();
                else if (can_msg_in.data[2] == 0x0D) 
                    get_pedal_pos();
                else if (can_msg_in.data[2] == 0x0E)
                    get_en_status();
                

            } else if (can_msg_in.data[1] == 0x0D) {
                get_direc();

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

        digitalWrite(ACT_LED, LOW);
    }
}

/**
 * @brief Get the wiper pos
 * 
 */

void announce() {
    digitalWrite(COM_LED, HIGH);

    struct can_frame can_msg_out;

    can_msg_out.can_id = CAN_ID;
    can_msg_out.can_dlc = CAN_DLC;
    can_msg_out.data[0] = 1;
    can_msg_out.data[1] = 2;
    can_msg_out.data[2] = 3;
    can_msg_out.data[3] = 4;
    can_msg_out.data[4] = 5;
    can_msg_out.data[5] = 6;
    can_msg_out.data[6] = 7;
    can_msg_out.data[7] = 8;

    for (int i = 0; i < 5; i++) {
        can.sendMessage(&can_msg_out);
        delay(100);

    }
    
    digitalWrite(COM_LED, LOW);
    
}

/**
 * @brief Write to the potentiometer
 * 
 * @param pos Positon (0-255)
 */

void pot_write(int pos) {
    if (pos < 0) 
        pos = 0;
    else if (pos > 255)
        pos = 255;

    while (pos != wiper_pos) {
        if (pos > wiper_pos)
            accel -> increment();
            wiper_pos++;
        else if (pos < wiper_pos)
            accel -> decrement();
            wiper_pos--;
    }

    get_wiper_pos();

}

/**
 * @brief Zero the potentiometer
 * 
 */

void pot_zero() {
    for (int i = 0; i < 260; i++) {
        accel -> decrement();

    }
    
    wiper_pos = 0;
    
}

/**
 * @brief Get the wiper pos
 * 
 */

void get_wiper_pos() {
    digitalWrite(COM_LED, HIGH);

    struct can_frame can_msg_out;

    can_msg_out.can_id = CAN_ID;
    can_msg_out.can_dlc = CAN_DLC;
    can_msg_out.data[0] = 0x0C;
    can_msg_out.data[1] = 0x0C;
    can_msg_out.data[2] = 0x0A;
    can_msg_out.data[3] = 0x0A;
    can_msg_out.data[4] = 0;
    can_msg_out.data[5] = 0;
    can_msg_out.data[6] = wiper_pos >> 8;
    can_msg_out.data[7] = wiper_pos & 0xFF;

    can.sendMessage(&can_msg_out);
    digitalWrite(COM_LED, LOW);
    
}

/**
 * @brief Get the pedal pos
 * 
 */

void get_pedal_pos() {
    digitalWrite(COM_LED, HIGH);

    int pedal_pos = analogRead(PEDAL_POT);

    struct can_frame can_msg_out;

    can_msg_out.can_id = CAN_ID;
    can_msg_out.can_dlc = CAN_DLC;
    can_msg_out.data[0] = 0x0C;
    can_msg_out.data[1] = 0x0C;
    can_msg_out.data[2] = 0x0A;
    can_msg_out.data[3] = 0x0D;
    can_msg_out.data[4] = 0;
    can_msg_out.data[5] = 0;
    can_msg_out.data[6] = pedal_pos >> 8;
    can_msg_out.data[7] = pedal_pos & 0xFF;

    can.sendMessage(&can_msg_out);
    digitalWrite(COM_LED, LOW);
    
}

/**
 * @brief Get the enable status
 * 
 */

void get_en_status() {
    digitalWrite(COM_LED, HIGH);

    struct can_frame can_msg_out;

    can_msg_out.can_id = CAN_ID;
    can_msg_out.can_dlc = CAN_DLC;
    can_msg_out.data[0] = 0x0C;
    can_msg_out.data[1] = 0x0C;
    can_msg_out.data[2] = 0x0A;
    can_msg_out.data[3] = 0x0E;
    can_msg_out.data[4] = 0;
    can_msg_out.data[5] = 0;
    can_msg_out.data[6] = 0;
    can_msg_out.data[7] = digitalRead(ACT_SW) == LOW ? 0x01 : 0x02;

    can.sendMessage(&can_msg_out);
    digitalWrite(COM_LED, LOW);

}

/**
 * @brief Get the direction
 * 
 */

void get_direc() {
    digitalWrite(COM_LED, HIGH);

    struct can_frame can_msg_out;

    can_msg_out.can_id = CAN_ID;
    can_msg_out.can_dlc = CAN_DLC;
    can_msg_out.data[0] = 0x0C;
    can_msg_out.data[1] = 0x0C;
    can_msg_out.data[2] = 0x0D;
    can_msg_out.data[3] = 0;
    can_msg_out.data[4] = 0;
    can_msg_out.data[5] = 0;
    can_msg_out.data[6] = 0;
    can_msg_out.data[7] = digitalRead(ACT_SW) == LOW ? 0x01 : 0x02;

    can.sendMessage(&can_msg_out);
    digitalWrite(COM_LED, LOW);

}

/**
 * @brief Pedal pressed
 * 
 */

void pedal_act() {
    digitalWrite(PEDAL_LED, HIGH);
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

    digitalWrite(COM_LED, LOW);

}

/**
 * @brief Pedal released
 * 
 */

void pedal_deact() {
    digitalWrite(PEDAL_LED, LOW);
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

    digitalWrite(COM_LED, LOW);

}
