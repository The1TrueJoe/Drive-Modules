/**
 * @file key_switch.cpp
 * 
 * @author Joseph Telaak
 * 
 * @brief Controls the forward and reverse settings that would normally be controlled by the key switch
 * 
 * @version 0.1
 * 
 * @date 2022-04-28
 * 
 * @copyright Copyright (c) 2022
 * 
 */

// --------------- Imports
#include "key_switch.h"

/**
 * @brief Set up the hardware key switch
 * 
 */

void keyswitch_setup() {
    // Relay pin mode
    pinMode(FWD_REV_SEL, OUTPUT);

    // Default forwards
    forwards();

}

/**
 * @brief CAN processing specific to the key switch
 * 
 */

void keyswitch_can_processing() {
    switch (can -> can_msg_in.data[can_submodule_component_indentifier]) {
        case switch_component_identifier:
            switch (can -> can_msg_in.data[can_op_identifier]) {
                case can_function_op:
                    switch (can -> can_msg_in.data[can_function_macro_identifier]) {
                        case set_forwards_macro:
                            forwards(); break;
                        case set_reverse_macro:
                            reverse(); break;
                        default:
                            break;
                    }

                    break;

                case can_req_op:
                    switch (can -> can_msg_in.data[can_function_macro_identifier]) {
                        case fwdrev_position_req:
                            update_keyswitch_pos(); break;
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
}

/**
 * @brief Post the position of the key switch
 * 
 */

void update_keyswitch_pos() {
    // Make message
    uint8_t keyswitch_position_post[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    keyswitch_position_post[can_bool_setting_identifier] = digitalRead(ACT_SW) == HIGH ? can_msg_true : can_msg_false;

    // Send message
    can -> sendCANMessage(CAN_ID, keyswitch_position_post);

}

/** @brief set the key switch to forwards */
void forwards() { digitalWrite(FWD_REV_SEL, HIGH); update_keyswitch_pos(); }
/** @brief set the key switch to reverse */
void reverse() { digitalWrite(FWD_REV_SEL, LOW); update_keyswitch_pos(); }