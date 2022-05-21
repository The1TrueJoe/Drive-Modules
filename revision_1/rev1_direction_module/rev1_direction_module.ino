/**
 * @file rev1_direction_module.ino
 * 
 * @author Joseph Telaak
 * 
 * @brief Code for the simple motor controller module
 * 
 * @version 0.1
 * @date 2022-02-12
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include <mcp2515.h>
#include <Encoder.h>
#include <BTS7960.h>

// Steering Motor Ctrl
#define STR_L_PWM 6
#define STR_R_PWM 9
#define STR_ENABLE 4

// Steering Linear Actuator Potentiometer
#define STR_POT A5
#define STEER_TOL 10 // Tolerance in Increments of ADC's pot value (0-1023)

// Steering Wheel Input Encoder
#define STR_WHL_ENC 3
#define STR_WHL_ENC2 A3

// Brake Motor Ctrl
#define BRK_L_PWM 5
#define BRK_R_PWM 10
#define BRK_ENABLE 7

// Brake Actuator Potentiometer
#define BRK_POT A4

// LEDS
#define COM_LED A0
#define ACT_LED A1

// CAN
#define CAN_CS 8
#define CAN_INT 2
#define CAN_DLC 8
#define CAN_ID 0x001

// CAN Adapter
MCP2515 can(CAN_CS);

// Encoder
Encoder wheel_enc(STR_WHL_ENC, STR_WHL_ENC2);
volatile long old_pos = -999;

// Motor Controllers
BTS7960 steering_motor(STR_ENABLE, STR_L_PWM, STR_R_PWM);
BTS7960 brake_motor(BRK_ENABLE, BRK_L_PWM, BRK_R_PWM);

// Identfiy
volatile bool identify = false;

// Direct pedal feed
volatile bool direct_wheel_feed = false;

/**
 * @brief Setup function
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

    // CAN Setup
    can.reset();
    can.setBitrate(CAN_125KBPS);
    can.setNormalMode();
  
    // Setup Motor
    steering_motor.Disable();
    steering_motor.Stop();

    brake_motor.Disable();
    brake_motor.Stop();

    // LED Low
    digitalWrite(ACT_LED, LOW);

    // CAN Interrupt
    attachInterrupt(digitalPinToInterrupt(CAN_INT), can_irq, FALLING);

}

// Timer counter
int counter = 0;

/**
 * @brief Periodic updates
 * 
 */

void loop() {
    while (identify) {
        digitalWrite(ACT_LED, LOW);
        digitalWrite(COM_LED, LOW);

        delay(1000);
        counter++;

        digitalWrite(ACT_LED, HIGH);
        digitalWrite(COM_LED, HIGH);

        delay(1000);
        counter++;

    }

    if (direct_wheel_feed) {
        long pos = wheel_enc.readAndReset();

        if (pos > old_pos) {
            steering_motor.TurnLeft(255);
            
        } else if (pos < old_pos) {
            steering_motor.TurnRight(255);

        } else {
            steering_motor.Stop();

        }

        delay(10);
         
    } else {
        delay(100);
        counter++;

    }
    
    if (counter >= 10) {
         // Updates
        digitalWrite(ACT_LED, HIGH);
        compound_update();
        digitalWrite(ACT_LED, LOW);

        counter = 0;

    }
}

/**
 * @brief CAN Interrupt
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
                if (can_msg_in.data[1] == 0x01) {
                    if (can_msg_in.data[2] == 0x0A) {
                        if (can_msg_in.data[3] == 0x01) {
                            steering_motor.Disable(); 
                            steering_motor.Stop();

                        } else if (can_msg_in.data[3] == 0x02) {
                            steering_motor.Enable();
                        } else {
                            read_str_state();

                        }

                    } else if (can_msg_in.data[2] == 0x0C) {
                        if (can_msg_in.data[3] == 0x01)
                            steering_motor.TurnLeft(can_msg_in.data[4]);
                        else if (can_msg_in.data[3] == 0x02)
                            steering_motor.TurnRight(can_msg_in.data[4]);
                        else
                            read_str_pot();

                    }

                } else if (can_msg_in.data[1] == 0x02) {
                    if (can_msg_in.data[2] == 0x0A) {
                        if (can_msg_in.data[3] == 0x01) {
                            brake_motor.Disable();
                            brake_motor.Stop();

                        } else if (can_msg_in.data[4] == 0x02) {
                            brake_motor.Enable();

                        } else {
                            read_brk_state();

                        }

                    } else if (can_msg_in.data[2] == 0x0C) {
                        if (can_msg_in.data[3] == 0x01) 
                            brake_motor.TurnLeft(can_msg_in.data[4]);
                        else if (can_msg_in.data[3] == 0x02)
                            brake_motor.TurnRight(can_msg_in.data[4]);
                        else
                            read_brk_pot();

                    }
                }

            } else if (can_msg_in.data[0] == 0x0B) {
                if (can_msg_in.data[1] == 0x01) {
                    steer_to_pos(can_msg_in.data[2] | (can_msg_in.data[3] << 8), can_msg_in.data[3] != 0 ? 255 : can_msg_in.data[4]);

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

                } else if (can_msg_in.data[1] == 0x0F) {
                    if (can_msg_in.data[2] == 0x01)
                        direct_wheel_feed = true;
                    else if (can_msg_in.data[2] == 0x02)
                        direct_wheel_feed = false;
                    
                }
              
            } else if (can_msg_in.data[0] == 0x0C) {
                if (can_msg_in.data[1] == 0x01) {
                    if (can_msg_in.data[2] == 0x0A)
                        read_str_state();
                    else if (can_msg_in.data[3] == 0x0E) 
                        read_str_whl();
                    else if (can_msg_in.data[3] == 0x0F)
                        read_str_pot();

            
                } else if (can_msg_in.data[2] == 0x02) {
                    if (can_msg_in.data[2] == 0x0A) 
                        read_brk_state();
                    else if (can_msg_in.data[2] == 0x0F) 
                        read_brk_pot();

                }
            }
        }

        // Clear the message buffer
        can_msg_in.can_id = CAN_ID;
        can_msg_in.can_dlc = CAN_DLC;
        can_msg_in.data[0] = 0x00;
        can_msg_in.data[1] = 0x00;
        can_msg_in.data[2] = 0x00;
        can_msg_in.data[3] = 0x00;
        can_msg_in.data[4] = 0x00;
        can_msg_in.data[5] = 0x00;
        can_msg_in.data[6] = 0x00;
        can_msg_in.data[7] = 0x00;

        digitalWrite(ACT_LED, LOW);

    }
}

/**
 * @brief Steer to a set postion
 * 
 * @param pos Position (0-1023)
 * @param power Power Level (0-255)
 */

void steer_to_pos(int pos, int power) {
    if (digitalRead(STR_ENABLE) != LOW) { return; }
    int current_pos = read_str_pot();

    while (abs(current_pos - pos) > STEER_TOL) {
        if (current_pos < pos) {
            steering_motor.TurnLeft(power);
            steering_motor.Enable();

        } else if (current_pos > pos) {
            steering_motor.TurnRight(power);
            steering_motor.Enable();

        }

        current_pos = read_str_pot();
    }

    read_str_pot();
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
    can_msg_out.data[3] = 0x01;
    can_msg_out.data[4] = digitalRead(STR_ENABLE) + 0x01;
    can_msg_out.data[5] = can_msg_out.data[4] == 0x02 ? ( digitalRead(STR_ENABLE) == HIGH ? 0x01 : 0x02 ) : 0x00;
    can_msg_out.data[6] = max(analogRead(STR_L_PWM), analogRead(STR_R_PWM));
    can_msg_out.data[7] = map(analogRead(STR_POT), 0, 1023, 0, 255);

    can.sendMessage(&can_msg_out);

    can_msg_out.can_id = CAN_ID;
    can_msg_out.can_dlc = CAN_DLC;
    can_msg_out.data[0] = 0x0C;
    can_msg_out.data[1] = 0x0C;
    can_msg_out.data[2] = 0x0C;
    can_msg_out.data[3] = 0x02;
    can_msg_out.data[4] = digitalRead(BRK_ENABLE) + 0x01;
    can_msg_out.data[5] = can_msg_out.data[4] == 0x02 ? ( digitalRead(BRK_ENABLE) == HIGH ? 0x01 : 0x02 ) : 0x00;
    can_msg_out.data[6] = max(analogRead(BRK_L_PWM), analogRead(BRK_R_PWM));
    can_msg_out.data[7] = map(analogRead(BRK_POT), 0, 1023, 0, 255);

    can.sendMessage(&can_msg_out);

    digitalWrite(COM_LED, LOW);

}

/**
 * @brief Read the brake potentiometer
 * 
 */

int read_brk_pot() {
    digitalWrite(COM_LED, HIGH);

    int pot_value = map(analogRead(BRK_POT), 0, 1023, 0, 255);

    struct can_frame can_msg_out;

    can_msg_out.can_id = CAN_ID;
    can_msg_out.can_dlc = CAN_DLC;
    can_msg_out.data[0] = 0x0C;
    can_msg_out.data[1] = 0x0C;
    can_msg_out.data[2] = 0x02;
    can_msg_out.data[3] = 0x0F;
    can_msg_out.data[4] = 0x00;
    can_msg_out.data[5] = 0x00;
    can_msg_out.data[6] = 0x00;
    can_msg_out.data[7] = pot_value;

    can.sendMessage(&can_msg_out);
    digitalWrite(COM_LED, LOW);

    return pot_value;

}

/**
 * @brief Read the brake enable state
 * 
 */

void read_brk_state() {
    digitalWrite(COM_LED, HIGH);
    
    struct can_frame can_msg_out;

    can_msg_out.can_id = CAN_ID;
    can_msg_out.can_dlc = CAN_DLC;
    can_msg_out.data[0] = 0x0C;
    can_msg_out.data[1] = 0x0C;
    can_msg_out.data[2] = 0x02;
    can_msg_out.data[3] = 0x0A;
    can_msg_out.data[4] = 0x00;
    can_msg_out.data[5] = 0x00;
    can_msg_out.data[6] = 0x00;
    can_msg_out.data[7] = digitalRead(BRK_ENABLE) + 0x01;
    
    can.sendMessage(&can_msg_out);
    digitalWrite(COM_LED, LOW);

}

/**
 * @brief Read the steering position
 * 
 */

int read_str_pot() {
    digitalWrite(COM_LED, HIGH);

    int pot_value = map(analogRead(STR_POT), 0, 1023, 0, 255);

    struct can_frame can_msg_out;

    can_msg_out.can_id = CAN_ID;
    can_msg_out.can_dlc = CAN_DLC;
    can_msg_out.data[0] = 0x0C;
    can_msg_out.data[1] = 0x0C;
    can_msg_out.data[2] = 0x01;
    can_msg_out.data[3] = 0x0F;
    can_msg_out.data[4] = 0x00;
    can_msg_out.data[5] = 0x00;
    can_msg_out.data[6] = 0x00;
    can_msg_out.data[7] = pot_value;
    
    can.sendMessage(&can_msg_out);
    digitalWrite(COM_LED, LOW);

    return pot_value;

}

/**
 * @brief Read the steering enable state
 * 
 */

void read_str_state() {
    digitalWrite(COM_LED, HIGH);

    struct can_frame can_msg_out;

    can_msg_out.can_id = CAN_ID;
    can_msg_out.can_dlc = CAN_DLC;
    can_msg_out.data[0] = 0x0C;
    can_msg_out.data[1] = 0x0C;
    can_msg_out.data[2] = 0x01;
    can_msg_out.data[3] = 0x0A;
    can_msg_out.data[4] = 0x00;
    can_msg_out.data[5] = 0x00;
    can_msg_out.data[6] = 0x00;
    can_msg_out.data[7] = digitalRead(STR_ENABLE) + 0x01;

    can.sendMessage(&can_msg_out);
    digitalWrite(COM_LED, LOW);

}

/**
 * @brief Read the steering wheel change
 * 
 */

long read_str_whl() {
    digitalWrite(COM_LED, HIGH);

    long pos = wheel_enc.readAndReset();
    uint8_t change_id = 0x00;

    if (pos > old_pos)
        change_id = 0x01;
    else if (pos < old_pos)
        change_id = 0x02;

    struct can_frame can_msg_out;

    can_msg_out.can_id = CAN_ID;
    can_msg_out.can_dlc = CAN_DLC;
    can_msg_out.data[0] = 0x0C;
    can_msg_out.data[1] = 0x0C;
    can_msg_out.data[2] = 0x01;
    can_msg_out.data[3] = 0x0E;
    can_msg_out.data[4] = 0x00;
    can_msg_out.data[5] = 0x00;
    can_msg_out.data[6] = 0x00;
    can_msg_out.data[7] = change_id;

    can.sendMessage(&can_msg_out);
    digitalWrite(COM_LED, LOW);

    return pos;

}
