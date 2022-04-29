/**
 * @file net.h
 * 
 * @author Joseph Telaak
 * 
 * @brief Contains the address for the other devices on the CAN network
 * 
 * @version 0.1
 * 
 * @date 2022-04-26
 * 
 * @copyright Copyright (c) 2022
 * 
 */

// --------------- Default Module CAN Addresses
#define master_can_id 0x001
#define direction_module_address 0xFF1
#define accessory_module_address 0xFF2
#define drive_module_address 0xFF3

// --------------- CAN Message Structures
#define can_assignment_op 0x0A
#define can_function_op 0x0B
#define can_req_op 0x0C

#define can_msg_true 0x01
#define can_msg_false 0x02

// Universally Applied
#define can_op_identifier 0
#define can_submodule_indentifier 1
#define can_submodule_component_indentifier 2

#define can_function_macro_identifier 3

#define can_int_setting_msb_identifier 7
#define can_int_setting_lsb_identifier 8
#define can_bool_setting_identifier 8
