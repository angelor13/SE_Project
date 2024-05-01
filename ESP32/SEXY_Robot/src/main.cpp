// #include <Arduino.h>
// #include <SPI.h>
// #include <VL53L0X.h>
// #include <MFRC522.h> 
// #include <Wire.h>
#include <SEXY_ESP32.h>

SEXY_ESP32 bot;



void setup(){
  
bot.begin();
bot.printI2C();
//SPI.begin();                  // Init SPI bus
  //Dist.begin(0);
}


void loop() {
 
    if(bot.getTagDetected()){
      Serial.println("Detected!");
    }
}

