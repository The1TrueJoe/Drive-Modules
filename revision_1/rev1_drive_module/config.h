/**
 * @file onfig.h
 * 
 * @author Joseph Telaak
 * 
 * @brief Configuration for the module
 * 
 * @version 0.1
 * 
 * @date 2022-04-26
 * 
 * @copyright Copyright (c) 2022ß
 * 
 */

#ifndef CONFIG_H
#define CONFIG_H

// --------------- Imports
#include "can_adapter.h"
#include "sys_can.h"
#include "Arduino.h"

// --------------- Config

//#define DEBUG
#define ONE_WAY_POT

// --------------- Accelerator

// CAN Identifer
#define accel_submodule_identifier 0x1

// Speed Controller
#define SPEED_CTRL_CS 9

// Activity Control
#define ACT_SW 4

// --------------- Acclerator Pedal

// CAN Identifer
#define accel_pedal_submodule_identifier 0x2

// Accelerator Pedal
#define PEDAL_POT A3

// Pedal Switch
#define PEDAL_SW 3

// --------------- Key Switch

// CAN Identifer
#define keyswitch_submodule_identifier 0x3

// Direction Control
#define FWD_REV_SEL 5

// --------------- CAN
static CAN_Adapter *can;

// CAN
#define CAN_CS 10
#define CAN_INT 2

#define CAN_ID 0x7F3

#endif