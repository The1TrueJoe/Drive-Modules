
#include <module.h>
#include <MCP4131.h>

// Accelerator Input
#define ACCEL_INPUT_SEL 7

// Activity Control
#define ACT_SW 4
#define ACT_SEL 3

// Direction Control
#define FWD_REV_SEL 5
bool buzzer_enabled = false;
uint32_t accessory_control_address = accessory_module_default_address;

// Digital Potentiometer
#define SPEED_CTRL_CS 9
MCP4131 mcp4131(SPEED_CTRL_CS);

// CAN
#define CAN_CS 10

// --------- Arduino

void setup() {
    // CAN ID
    m_can_id = speed_module_default_address;

    // Standard module setup
    standardModuleSetup(CAN_CS);

    // Announce Ready
    ready();
    holdTillEnabled();

    // Setup Interupts
    attachInterupt(CAN_INT, canLoop, FALLING);

    // Relay setup
    setupRelays();

}

void loop() {

}

void canLoop() {

}

// --------- Forward / Reverse

void setupDirectionSelector() { pinMode(FWD_REV_SEL, OUTPUT); }

void forward() { 
    digitalWrite(FWD_REV_SEL, LOW); 
    postDirection();

    if (buzzer_enabled) {
        uint8_t message[8] = { 0x0A, 0x06, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00 };
        sendCANMessage(accessory_control_address, message);

    }
    
}

void reverse() {
    digitalWrite(FWD_REV_SEL, HIGH);
    postDirection();

    if (buzzer_enabled) {
        uint8_t message[8] = { 0x0A, 0x06, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00 };
        sendCANMessage(accessory_control_address, message);

    }

}

void isForwards() { }

void postDirection() {

}

void disableBuzzer() { buzzer_enabled = false; }

void enableBuzzer() { buzzer_enabled = true; }

void postBuzzerEnable() {
    

}



// --------- Speed Control

// --------- Motor Activity