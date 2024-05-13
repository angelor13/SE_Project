#include <SEXY_ESP32.h>

// RFID PINOUT

byte data[12] = {"Embebed-Sys"};

SEXY_ESP32 bot;

void setup() {
    bot.begin();
}

void loop() {
        // Look for new cards if not found rerun the loop function
if(bot.Tag_Detected()){
    Serial.println("Tag Detected");
}
delay(50);
}

