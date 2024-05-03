#include <SEXY_ESP32.h>

SEXY_ESP32 bot;



void setup(){
bot.begin();  
}


void loop() {
  if(bot.getTagDetected()){
    Serial.println("Detected!");
  }
  bot.moveMotors(255,200);
  delay(100);

}

