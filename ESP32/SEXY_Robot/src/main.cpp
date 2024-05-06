#include <SEXY_ESP32.h>

SEXY_ESP32 bot;
          
float float_map(float vx, float vin_min, float vin_max, float  vout_min, float  vout_max) {
  return (vx - vin_min) * (vout_max - vout_min) / (vin_max - vin_min) + vout_min;
}

float leftPWM,rightPWM;

void curve90Circule(float vx,float R){
  float w=vx/R;
  leftPWM=float_map(bot.calculatedDotphiL(vx,w,bot.L,bot.getDotphiL()),-bot.MAX_DOTPHI,bot.MAX_DOTPHI,-511,511);
  rightPWM=float_map(bot.calculatedDotphiR(vx,w,bot.L,bot.getDotphiR()),-bot.MAX_DOTPHI,bot.MAX_DOTPHI,-511,511);

  if(vx<0){
    // Bot.moveMotors(leftPWM,rightPWM);
    // long start_millis=millis();
    // while (millis()-start_millis<=300){

    // }
    // rotate_90_Stacionary();
    // rotate_90_Stacionary();
    float aux=-leftPWM;
    leftPWM=-rightPWM;
    rightPWM=aux;
    vx=-vx;
  }
  
bot.moveMotors(leftPWM,rightPWM);
}

void setup(){
bot.begin();  
}
int32_t rx[2];

void loop() {
  //bot.getTagDetected();

    bot.moveMotors(500,500);
    delay(10); 
  
  // for(int i=0;i<2;i++){
  //   Serial.println((String)bot.dotphiL + (String)bot.dotphiR );

  // }
  // digitalWrite(5, LOW);
  // SPI.transferBytes(rx,NULL, sizeof(rx));
  // digitalWrite(5, HIGH);
  //   for(int i=0;i<2;i++){
  //   Serial.println((String)rx[0] + (String)rx[1] );
  // }

// uint32_t left_distance=bot.getLeftDistance();
// uint32_t front_distance=bot.getFrontDistance();
// uint32_t right_distance=bot.getRightDistance();


// bot.moveMotors(500,500);    // andar os pimeiros 50 cm
// long start_millis=millis();
// while(millis()-start_millis<=1000){

// }


// curve90Circule(bot.getVx(),bot.getR(0.2));
// long start_millis=millis();
// while (millis()-start_millis<=1300)
// {

// }

}

