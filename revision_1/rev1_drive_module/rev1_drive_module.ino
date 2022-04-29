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

// Imports
#include "accelerator.h"
#include "accel_pedal.h"
#include "key_switch.h"

/**
 * @brief Main setup function
 * 
 */

void setup() {
    // CAN
    can -> setupCAN(CAN_CS, CAN_ID);
    attachInterrupt(digitalPinToInterrupt(CAN_INT), can_processing, FALLING);

    // Setup functions
    accel_pedal_setup();
    accelerator_setup();
    keyswitch_setup();

}

int timing_counter = 0;

/**
 * @brief Main loop to provide periodic updates
 * 
 */

void loop() {
    if (timing_counter % 10 == 0) {
        update_accelerator_pos();
        update_accelerator_enable();
        update_keyswitch_pos();

        timing_counter = 0;

    } 
    
    update_pedal_pot_pos();

    delay(10);
    timing_counter++;
    
}

/**
 * @brief CAN Processing
 * 
 */

void can_processing() {
    can -> getCANMessage();

    switch (can -> can_msg_in.data[can_submodule_indentifier]) {
        case accel_submodule_identifier:
            accelerator_can_processing(); break;
        case keyswitch_submodule_identifier:
            keyswitch_can_processing(); break;
        case accel_pedal_submodule_identifier:
            accel_pedal_can_processing(); break;
        default:
            break;
    }
}