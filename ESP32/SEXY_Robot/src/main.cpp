#include <SEXY_ESP32.h>

SEXY_ESP32 bot;



void setup(){
bot.begin();
bot.printI2C();               
}


void loop() {
  if(bot.getTagDetected()){
    Serial.println("Detected!");
  }
  
}

