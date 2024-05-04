#include <SEXY_ESP32.h>

SEXY_ESP32 bot;
          
float float_map(float vx, float vin_min, float vin_max, float  vout_min, float  vout_max) {
  return (vx - vin_min) * (vout_max - vout_min) / (vin_max - vin_min) + vout_min;
}


void setup(){
bot.begin();  
}


void loop() {
  //bot.getTagDetected();
  bot.moveMotors(500,500);
  delay(100);

// uint32_t left_distance=bot.getLeftDistance();
// uint32_t front_distance=bot.getFrontDistance();
// uint32_t right_distance=bot.getRightDistance();

// long start_millis=millis();
// float leftPWM=float_map(bot.getDotphiL(),-bot.MAX_DOTPHI,bot.MAX_DOTPHI,-511,511);
// float rightPWM=float_map(bot.getDotphiR(),-bot.MAX_DOTPHI,bot.MAX_DOTPHI,-511,511);
// bot.moveMotors(leftPWM,rightPWM);
// while (millis()-start_millis<=2000)
// {

// }

}

