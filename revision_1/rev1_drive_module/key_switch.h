/**
 * @file key_switch.h
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

#ifndef KEY_SWITCH_H
#define KEY_SWITCH_H

// --------------- Imports
#include "config.h"

// --------------- Function
void keyswitch_can_processing();
void keyswitch_setup();
void update_keyswitch_pos();

// -------- Key Switch Functions
void forwards();
void reverse();

// --------------- CAN

// ------- Indentifiers
#define switch_component_identifier 0x1

// ------- Function Macros
#define set_forwards_macro 0x1
#define set_reverse_macro 0x2

// ------- Request Messages
#define fwdrev_position_req 0x1

#endif