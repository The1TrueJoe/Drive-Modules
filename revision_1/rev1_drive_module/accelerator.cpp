/**
 * @file accelerator.cpp
 * 
 * @author Joseph Telaak
 * 
 * @brief Emulates the physical pedal by using relays and a digital potentiometer
 * 
 * @version 0.1
 * 
 * @date 2022-04-28
 * 
 * @copyright Copyright (c) 2022
 * 
 */

// --------------- Imports
#include "accelerator.h"

// --------------- Accelerator Control
MCP4XXX* accel_pot = new MCP4XXX(SPEED_CTRL_CS);

#ifdef ONE_WAY_POT
    int accel_wiper_pos = 0;
#endif

/**
 * @brief Setup function
 * 
 */

void accelerator_setup() {
    // Pins
    pinMode(ACT_SW, OUTPUT);

    // Zero and disable the Accelerator
    zero_accelerator();
    accelerator_disable();

}

/**
 * @brief Component-specific CAN processing
 * 
 */

void accelerator_can_processing() {
    switch (can -> can_msg_in.data[can_submodule_component_indentifier]) {
        case accel_pot_component_identifier:
            __accelerator_pot_can_processing(); break;
        case accel_sw_component_identifier:
            __accelerator_switch_can_processing(); break;
        default:  
            break;

    }
}

/**
 * @brief CAN processing for the pot emulator
 * 
 */

void __accelerator_pot_can_processing() {
    switch (can -> can_msg_in.data[can_op_identifier]) {
        case can_assignment_op:
            switch (can -> can_msg_in.data[can_function_macro_identifier]) {
                case set_accelerator_setting:
                    set_accelerator(can -> can_msg_in.data[can_int_setting_msb_identifier]); break;
                default:
                    break;
            }

            break;

        case can_function_op:
            switch (can -> can_msg_in.data[can_function_macro_identifier]) {
                case inc_accelerator_macro:
                    inc_accelerator(); break;
                case dec_accelerator_macro:
                    dec_accelerator(); break;
                case zero_accelerator_macro:
                    zero_accelerator(); break;
                default:
                    break;
            }

            break;

        case can_req_op:
            switch (can -> can_msg_in.data[can_function_macro_identifier]) {
                case accel_pos_req:
                    update_accelerator_pos(); break;
                default:
                    break;
            }

            break;

        default:
            break;
    }
}

/**
 * @brief CAN processing for the enable switch
 * 
 */

void __accelerator_switch_can_processing() {
    switch (can -> can_msg_in.data[can_op_identifier]) {
        case can_function_op:
            switch (can -> can_msg_in.data[can_function_macro_identifier]) {
                case enable_accelerator_macro:
                    accelerator_enable(); break;
                case disable_accelerator_macro:
                    accelerator_disable(); break;
                default:
                    break;
            }

            break;

        case can_req_op:
            switch (can -> can_msg_in.data[can_function_macro_identifier]) {
                case accel_status_req:
                    update_accelerator_enable(); break;
                default:
                    break;
            }

            break;

        default:
            break;
    }
}

/**
 * @brief Update the accelerator setting
 * 
 */

void update_accelerator_pos() { 
    // Read pos
    #ifdef ONE_WAY_POT
        uint8_t pos = accel_wiper_pos;
    #else 
        uint8_t pos = accel_pot -> read();
    #endif

    // Build message
    uint8_t accel_pos_post[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    accel_pos_post[can_int_setting_msb_identifier] = pos >> 8;
    accel_pos_post[can_int_setting_lsb_identifier] = pos & 0xFF;

    // Send message
    can -> sendCANMessage(CAN_ID, accel_pos_post);

}

/**
 * @brief Update the status of the accelerator enable switch
 * 
 */

void update_accelerator_enable() {
    // Build message
    uint8_t accel_status_post[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    accel_status_post[can_bool_setting_identifier] = digitalRead(ACT_SW) == HIGH ? can_msg_true : can_msg_false;

    // Send message
    can -> sendCANMessage(CAN_ID, accel_status_post);

}

/**
 * @brief Set the accelerator object
 * 
 * @param pos 
 */

void set_accelerator(int pos) {
    #ifdef ONE_WAY_POT
        while (accel_wiper_pos != pos) {
            if (accel_wiper_pos > pos) {
                accel_pot -> decrement();

            } else if (accel_wiper_pos < pos) {
                accel_pot -> increment();

            }
        }

    #else 
        accel_pot -> set(pos);

    #endif
}

/** @brief Zero the accelerator */
void zero_accelerator() { set_accelerator(0); }

/** @brief Increment the accelerator */
void inc_accelerator() { accel_pot -> increment(); update_accelerator_pos(); }
/** @brief Decrement the accelerator */
void dec_accelerator() { accel_pot -> decrement(); update_accelerator_pos(); }

/** @brief Enable the accelerator */
void accelerator_enable() { digitalWrite(ACT_SW, HIGH); update_accelerator_enable(); }
/** @brief Disable the accelerator */
void accelerator_disable() { digitalWrite(ACT_SW, LOW); update_accelerator_enable(); }