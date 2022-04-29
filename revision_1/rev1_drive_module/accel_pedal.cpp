/**
 * @file accel_pedal.cpp
 * 
 * @author Joseph Telaak
 * 
 * @brief Reads inputs from the physical accelerator pedal
 * 
 * @version 0.1
 * 
 * @date 2022-04-28
 * 
 * @copyright Copyright (c) 2022
 * 
 */

// --------------- Imports
#include "accel_pedal.h"

/**
 * @brief Sets up the accelerator pedal
 * 
 */

void accel_pedal_setup() {
    // Attach Pedal Interrupt
    attachInterrupt(digitalPinToInterrupt(PEDAL_SW), __on_pedal_press, RISING);
    
    // Potentiometer Pinmode
    pinMode(PEDAL_POT, INPUT);

}

/**
 * @brief CAN processing specific to the pedal
 * 
 */

void accel_pedal_can_processing() {
    switch (can -> can_msg_in.data[can_submodule_component_indentifier]) {
        case accel_pedal_pot_component_identifier:
            switch (can -> can_msg_in.data[can_op_identifier]) {
                case can_req_op:
                    switch (can -> can_msg_in.data[can_function_macro_identifier]) {
                        case accel_pedal_pot_pos_req:
                            update_pedal_pot_pos(); break;
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
 * @brief Post the accelerator pedal position
 * 
 */

void update_pedal_pot_pos() {
    // Get reading
    int pos = analogRead(PEDAL_POT);

    // Build message
    uint8_t accel_pedal_pot_pos_post[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    accel_pedal_pot_pos_post[can_int_setting_msb_identifier] = pos >> 8;
    accel_pedal_pot_pos_post[can_int_setting_lsb_identifier] = pos & 0xFF;

    // Send
    can -> sendCANMessage(CAN_ID, accel_pedal_pot_pos_post);

}

/**
 * @brief Interrupt when the pedal is pressed
 * 
 */

void __on_pedal_press() {
    // Attach Interupt
    attachInterrupt(digitalPinToInterrupt(PEDAL_SW), __on_pedal_release, FALLING);

    // Build Message
    uint8_t accel_pedal_sw_post[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    accel_pedal_sw_post[can_bool_setting_identifier] = can_msg_true;
    can -> sendCANMessage(can -> m_can_id, accel_pedal_sw_post);

}

/**
 * @brief Interrupt when the pedal is released
 * 
 */

void __on_pedal_release() {
    // Attach Interupt
    attachInterrupt(digitalPinToInterrupt(PEDAL_SW), __on_pedal_press, RISING);

    // Build Message
    uint8_t accel_pedal_sw_post[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    accel_pedal_sw_post[can_bool_setting_identifier] = can_msg_false;
    can -> sendCANMessage(can -> m_can_id, accel_pedal_sw_post);

}