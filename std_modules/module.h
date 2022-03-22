// --------- Standard Module Components
#include "can_adapter.h"
#include "id_light.h"

#define COM_SET 0x0A
#define COM_OP 0x0B
#define COM_GET 0x0C

uint32_t master_can_id = 0x001;

public:
    void standardModuleSetup(int CAN_CS_PIN = Default_CAN_CS);
    void standardModuleLoopHead();
    void standardModuleLoopTail();
    void ready();
    void holdTillEnabled();