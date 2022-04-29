/**
 * @file accelerator.h
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

#ifndef ACCELERATOR_H
#define ACCELERATOR_H

// --------------- Imports
#include "config.h"
#include "mcp4xxx.h"

// --------------- Accelerator Control
using namespace icecave::arduino;

// --------------- Function
void accelerator_can_processing();
void accelerator_setup();
void update_accelerator_pos();
void update_accelerator_enable();

void __accelerator_switch_can_processing();
void __accelerator_pot_can_processing();

// -------- Accelerator Functions
void set_accelerator(int pos);
void inc_accelerator();
void dec_accelerator();
void zero_accelerator();

// -------- Drive Functions
void accelerator_enable();
void accelerator_disable();

// --------------- CAN

// ------- Indentifiers
#define accel_pot_component_identifier 0x1
#define accel_sw_component_identifier 0x2

// ------- Set Messages
#define set_accelerator_setting 0x1

// ------- Function Macros
#define inc_accelerator_macro 0x1
#define dec_accelerator_macro 0x2
#define zero_accelerator_macro 0x3

#define enable_accelerator_macro 0x1
#define disable_accelerator_macro 0x2

// ------- Request Messages
#define accel_status_req 0x1
#define accel_pos_req 0x2

#endif