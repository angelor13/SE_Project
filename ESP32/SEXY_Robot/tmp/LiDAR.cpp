#include <SEXY_ESP32.h>

SEXY_ESP32 bot;


void setup(){
    
bot.begin();
bot.printI2C();

}

void loop(){
  uint32_t distance=bot.getFrontDistance();
  Serial.println(distance);
  delay(50);


}


