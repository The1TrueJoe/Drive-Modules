// --------- Standard Module Components
#include "can_adapter.h"
#include "id_light.h"

// CAN Default Message Headers
#define COM_SET 0x0A
#define COM_OP 0x0B
#define COM_GET 0x0C

// Default Module CAN Addresses
const uint32_t master_can_id = 0x001;
const uint32_t direction_control_default_address = 0xFF1;
const uint32_t accessory_module_default_address = 0xFF2;
const uint32_t speed_module_default_address = 0xFF3;

public:
    void standardModuleSetup(int CAN_CS_PIN = Default_CAN_CS);
    void standardModuleLoopHead();
    void standardModuleLoopTail();
    void ready();
    void holdTillEnabled();