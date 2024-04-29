#include <SEXY_ESP32.h>

// RFID PINOUT

byte data[12] = {"Embebed-Sys"};

SEXY_ESP32 bot;

void setup() {
    bot.begin();
    bot.setupRFID();           // Init MFRC522 card
}

void loop() {
        // Look for new cards if not found rerun the loop function
bot.Tag_Detected();
delay(50);
}

