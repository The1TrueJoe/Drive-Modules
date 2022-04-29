/**
 * @file accel_pedal.h
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

#ifndef ACCELERATOR_PEDAL_H
#define ACCELERATOR_PEDAL_H

// --------------- Imports
#include "config.h"

// --------------- Methods
void accel_pedal_can_processing();
void accel_pedal_setup();
void update_pedal_pot_pos();

void __on_pedal_press();
void __on_pedal_release();

// --------------- CAN

// ------- Indentifiers
#define accel_pedal_pot_component_identifier 0x1

// ------- Set Messages

// ------- Function Macros

// ------- Request Messages
#define accel_pedal_pot_pos_req 0x1

#endif