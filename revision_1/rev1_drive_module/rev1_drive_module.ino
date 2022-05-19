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
MCP4XXX* accel;
volatile int wiper_pos = 0;

// CAN
MCP2515 can(CAN_CS);

// Pedal status
volatile bool pedal_detect_enable = false;

// Direct pedal feed
volatile bool direct_pedal_feed = false;

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
        pinMode(PEDAL_SW, INPUT_PULLUP);

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
    wiper_pos = 0;

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
    while (identify) {
        digitalWrite(ACT_LED, LOW);
        digitalWrite(COM_LED, LOW);

        delay(1000);

        digitalWrite(ACT_LED, HIGH);
        digitalWrite(COM_LED, HIGH);

        delay(1000);

    }

    if (pedal_detect_enable) {
        if (digitalRead(PEDAL_SW) == LOW) {
            digitalWrite(PEDAL_LED, HIGH);
            pedal_act();

            if (direct_pedal_feed) {
                while (digitalRead(PEDAL_SW) == LOW) {
                    pot_write(map(analogRead(PEDAL_POT), 0, 1023, 0, 255));

                    if (counter > 100) { compound_update(); counter = 0; }

                    delay(10);
                    counter++;

                }

                counter /= 10;

            } else {
                while (digitalRead(PEDAL_SW) == LOW) { 
                    get_pedal_pos(); 
                    
                    if (counter % 10 == 0) { compound_update(); }

                    delay(100);
                    counter++;
                
                } 
            }

            pedal_deact();
            digitalWrite(PEDAL_LED, LOW);

        }
    }
    
    if (counter >= 10) {  compound_update(); counter == 0; }

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
                            digitalWrite(ACT_SW, RELAY_DEACT);
                        else if (can_msg_in.data[3] == 0x02) 
                            digitalWrite(ACT_SW, RELAY_ACT);
                        else
                            get_en_status();

                    }

                } else if (can_msg_in.data[1] == 0x0D) {
                    if (can_msg_in.data[2] == 0x01) 
                        digitalWrite(FWD_REV_SEL, RELAY_DEACT);
                    else if (can_msg_in.data[2] == 0x02) 
                        digitalWrite(FWD_REV_SEL, RELAY_ACT);
                    else
                        get_direc();

                }

            } else if (can_msg_in.data[0] == 0x0B) {
                if (can_msg_in.data[1] == 0x0A) {
                    if (can_msg_in.data[2] == 0x01) {
                        if (can_msg_in.data[3] == 0x00) {
                            accel -> increment();
                            wiper_pos++;

                        } else {
                            for (int i = 0; i < can_msg_in.data[3]; i++) {
                                accel -> increment();
                                wiper_pos++;

                            }
                        }
                        
                    } else if (can_msg_in.data[2] == 0x02) {
                        if (can_msg_in.data[3] == 0x00) {
                            accel -> decrement();
                            wiper_pos--;

                        } else {
                            for (int i = 0; i < can_msg_in.data[3]; i++) {
                                accel -> decrement();
                                wiper_pos--;

                            }
                        }

                    } else {
                        get_wiper_pos();

                    }
                    
                } else if (can_msg_in.data[1] = 0x0E) {
                    if (can_msg_in.data[2] = 0x01) {
                        identify = true;

                        digitalWrite(ACT_LED, HIGH);
                        digitalWrite(COM_LED, HIGH);

                    } else if (can_msg_in.data[2] = 0x02) {
                        identify = false;

                        digitalWrite(ACT_LED, LOW);
                        digitalWrite(COM_LED, LOW);

                    }

                } else if (can_msg_in.data[1] = 0x0F) {
                    if (can_msg_in.data[2] = 0x01)
                        direct_pedal_feed = true;
                    else (can_msg_in.data[2] = 0x02)
                        direct_pedal_feed = false;

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
        }

        // Clear the message buffer
        fill_data(&can_msg_in, 0, 7);

        digitalWrite(ACT_LED, LOW);
    }
}

/**
 * @brief Fill the rest of the data frame with zeroes
 * 
 * @param frame Pointer to can frame
 * @param start Start index (inclusive)
 * @param end End index (inclusive)
 */

void fill_data(can_frame* frame, uint8_t start, uint8_t end) {
    for (int i = start; i < end+1; i++) {
        frame->data[i] = 0;

    }
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
        if (pos > wiper_pos) {
            accel -> increment();
            wiper_pos++;
            
        } else if (pos < wiper_pos) {
            accel -> decrement();
            wiper_pos--;
            
        }
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
 * @brief Updates general information
 * 
 */

void compound_update() {
    digitalWrite(COM_LED, HIGH);

    struct can_frame can_msg_out;

    can_msg_out.can_id = CAN_ID;
    can_msg_out.can_dlc = CAN_DLC;
    can_msg_out.data[0] = 0x0C;
    can_msg_out.data[1] = 0x0C;
    can_msg_out.data[2] = 0x0C;
    can_msg_out.data[3] = digitalRead(ACT_SW) == RELAY_ACT ? 0x02 : 0x01;
    can_msg_out.data[4] = digitalRead(FWD_REV_SEL) == RELAY_ACT ? 0x02 : 0x01;
    can_msg_out.data[5] = wiper_pos;
    can_msg_out.data[6] = pedal_detect_enable ? digitalRead(PEDAL_SW) + 0x01 : 0x00;
    can_msg_out.data[7] = pedal_detect_enable ? map(analogRead(PEDAL_POT), 0, 1023, 0, 255) : 0;

    can.sendMessage(&can_msg_out);
    digitalWrite(COM_LED, LOW);

}

/**
 * @brief Get the wiper pos
 * 
 */

int get_wiper_pos() {
    digitalWrite(COM_LED, HIGH);

    struct can_frame can_msg_out;

    can_msg_out.can_id = CAN_ID;
    can_msg_out.can_dlc = CAN_DLC;
    can_msg_out.data[0] = 0x0C;
    can_msg_out.data[1] = 0x0C;
    can_msg_out.data[2] = 0x0A;
    can_msg_out.data[3] = 0x0A;
    fill_data(&can_msg_out, 4, 6);
    can_msg_out.data[7] = wiper_pos;

    can.sendMessage(&can_msg_out);
    digitalWrite(COM_LED, LOW);

    return wiper_pos;
    
}

/**
 * @brief Get the pedal pos
 * 
 */

int get_pedal_pos() {
    digitalWrite(COM_LED, HIGH);

    int pedal_pos = map(analogRead(PEDAL_POT), 0, 1023, 0, 255);

    struct can_frame can_msg_out;

    can_msg_out.can_id = CAN_ID;
    can_msg_out.can_dlc = CAN_DLC;
    can_msg_out.data[0] = 0x0C;
    can_msg_out.data[1] = 0x0C;
    can_msg_out.data[2] = 0x0A;
    can_msg_out.data[3] = 0x0D;
    fill_data(&can_msg_out, 4, 6);
    can_msg_out.data[7] = pedal_pos;

    can.sendMessage(&can_msg_out);
    digitalWrite(COM_LED, LOW);

    return pedal_pos;
    
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
    fill_data(&can_msg_out, 4, 6);
    can_msg_out.data[7] = digitalRead(ACT_SW) == RELAY_ACT ? 0x02 : 0x01;

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
    fill_data(&can_msg_out, 3, 6);
    can_msg_out.data[7] = digitalRead(FWD_REV_SEL) == RELAY_ACT ? 0x02 : 0x01;

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
    fill_data(&can_msg_out, 3, 6);
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
    fill_data(&can_msg_out, 3, 6);
    can_msg_out.data[7] = 0x01;

    can.sendMessage(&can_msg_out);

    digitalWrite(COM_LED, LOW);

}
