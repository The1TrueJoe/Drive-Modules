#include "can_adapter.h"
#include "Adafruit_LiquidCrystal.h"

#define Register_Select 7
#define Enable 6
#define D4 5
#define D5 4
#define D6 3
#define D7 2

Adafruit_LiquidCrystal lcd(Register_Select, Enable, D4, D5, D6, D7);

/**
 * @brief Main Arduino Setup
 * 
 */

void setup() {
    // Init serial port
    Serial.begin(115200);

    // Setup the can bus transceiver
    setupCAN();

    // Setup the Display
    lcd.begin(16, 2);

    // Startup
    lcd.print("GSSM AutoCart");
    lcd.setCursor(0, 1);
    lcd.print("Adapter Start");


}

/**
 * @brief Main Arduino Loop
 * 
 */

void loop() {
    getCANMessage();

    if (Serial.available() > 0) {
        String drive_com_msg = Serial.readString();

        if (drive_com_msg.indexOf("CMD-Send: ") != -1) {
            adapterSendMessage(drive_com_msg);

        } else if (drive_com_msg.indexOf("Display: ") != -1) {
            display(drive_com_msg);

        }
    }m
}

/**
 * @brief Print a message to the drive computer LCD
 * 
 * @param drive_com_msg message to display ("L{line num}: {message}")
 */

void display(String drive_com_msg) {
    drive_com_msg = drive_com_msg.replace("Display: ", "");

    if (drive_com_msg.indexOf("L1: ") != -1) {
        drive_com_msg = drive_com_msg.replace("L1: ", "");
        lcd.setCursor(0, 0);
        lcd.print(drive_com_msg);

    } else if (drive_com_msg.indexOf("L2: ") != -1) {
        drive_com_msg = drive_com_msg.replace("L2: ", "");
        lcd.setCursor(0, 1);
        lcd.print(drive_com_msg);

    } else if (drive_com_msg.indexOf("Clear") != -1) {
        lcd.setCursor(0, 0);
        lcd.print("");
        lcd.setCursor(0, 1);
        lcd.print("");

    } else if (drive_com_msg.indexOf("Off") != -1) {
        lcd.noDisplay();

    } else if (drive_com_msg.indexOf("On") != -1) {
        lcd.display();

    }
}

/**
 * @brief Sends a message through the CAN Adapter
 * 
 * @param drive_com_msg Message to send (8x bytes separated by spaces)
 */

void adapterSendMessage(String drive_com_msg) {
    // Remove message header
    drive_com_msg = drive_com_msg.replace("CMD-Send: ", "");

    // Get the ID indexes
    int id_begin_index = drive_com_msg.indexOf("(");
    int id_end_index = drive_com_msg.indexOf(")");

    // Check ID
    if (id_begin_index == -1 || id_end_index == -1) {
        Serial.println("Err: CAN message is missing ID");
        continue;

    }

    // Get the ID
    String str_id = drive_com_msg.substring(id_begin_index, id_end_index - 1);
    byte id_buf[4];
    str_id.StringToCharArray(id_buf, sizeof(id_buf));
    uint32_t set_id = id_buf[0] | (id_buf[1] << 8) | (id_buf[2] << 16) | (id_buf[3] << 24);

    // Clear ID
    drive_com_msg.replace(drive_com_msg.substring(0, id_end_index + 1), "");

    // Clear Spaces
    drive_com_msg.replace(" ", "");

    // Get data
    uint8_t data_buf[8];
    drive_com_msg.StringToCharArray(data_buf, sizeof(data_buf));

    // Send message
    sendCANMessage(set_id, data_buf);

}