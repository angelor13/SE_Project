#include <Arduino.h>
#include <SPI.h>


static constexpr uint16_t PIN_VP_LEFT=36;


int distance;

void setup()
{
  Serial.begin(115200);

}

void loop()
{
  uint16_t value=analogRead(PIN_VP_LEFT);
  uint16_t distance=constrain(map(value,0,4095,100,800),100,800);
  Serial.print("\nDistance in centimers: ");
  Serial.print(distance);  
  delay(500); //make it readable
}






