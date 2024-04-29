// #include <Arduino.h>
// #include <SPI.h>
// #include <VL53L0X.h>
// #include <MFRC522.h> 
// #include <Wire.h>
#include <SEXY_ESP32.h>
//#include <DistanceGP2Y0A21YK.h>

//DistanceGP2Y0A21YK Dist;
SEXY_ESP32 bot;



void setup(){
  
bot.begin();
// SPI.begin();                  // Init SPI bus
  //Dist.begin(0);
}


void loop() {
    // Look for new cards if not found rerun the loop function
    uint32_t distance=bot.getFrontDistance();
    Serial.println("Distância medida:"+(String)distance);
    delay(50);

}

