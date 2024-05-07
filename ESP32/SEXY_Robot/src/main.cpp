#include <SEXY_ESP32.h>

SEXY_ESP32 bot;
          
float float_map(float vx, float vin_min, float vin_max, float  vout_min, float  vout_max) {
  return (vx - vin_min) * (vout_max - vout_min) / (vin_max - vin_min) + vout_min;
}

float leftW,rightW;

void curve90Circule(float vx,float R){
  float w=vx/R;
  leftW=float_map(bot.calculatedDotphiL(vx,w,bot.L,bot.getDotphiL()),-bot.MAX_DOTPHI,bot.MAX_DOTPHI,-511,511);
  rightW=float_map(bot.calculatedDotphiR(vx,w,bot.L,bot.getDotphiR()),-bot.MAX_DOTPHI,bot.MAX_DOTPHI,-511,511);

  if(vx<0){
    // Bot.moveMotors(leftPWM,rightPWM);
    // long start_millis=millis();
    // while (millis()-start_millis<=300){

    // }
    // rotate_90_Stacionary();
    // rotate_90_Stacionary();
    float aux=-leftW;
    leftPWM=-rightPW;
    rightPWM=aux;
    vx=-vx;
  }
  
bot.moveMotors(leftPWM,rightPWM);
}

void setup(){
bot.begin();  
}

byte rx[4];
byte tx=10;
void loop() {
  delay(1);
  // if(bot.getTagDetected()){
  //   Serial.println("Tag detected");
  // }

    // bot.moveMotors(123,123);
    // delay(10); 
  
  // for(int i=0;i<2;i++){
  //   Serial.println((String)bot.dotphiL + (String)bot.dotphiR );

  // }
  // digitalWrite(5, LOW);
  // SPI.transferBytes(&tx,NULL, sizeof(tx));
  // digitalWrite(5, HIGH);
  //   for(int i=0;i<2;i++){
  //   Serial.println((String)rx[0] + (String)rx[1] );
  // }
  //   digitalWrite(5, LOW);
  // SPI.transferBytes(NULL,rx,1);
  // digitalWrite(5, HIGH);

 
  // Done varios SPI

  // for(int i=0;i<4;i++){

  // digitalWrite(5, LOW);
  // SPI.transferBytes(NULL,rx,1);
  // digitalWrite(5, HIGH);
  // Serial.printf("%f\n",(float)rx[0]);
  
    
  //  }
  
  // Done

//  for(int i=0;i<4;i++){

//   Serial.printf("%f\n",(float)rx[i]);
//  }

// uint32_t left_distance=bot.getLeftDistance();
// uint32_t front_distance=bot.getFrontDistance();
// uint32_t right_distance=bot.getRightDistance();

// Real Code


// bot.moveMotors(500,500);    // andar os pimeiros 50 cm
// long start_Ldistance=bot.getDistanceL();
// long start_Rdistance=bot.getDistanceR();
// while(bot.getDistanceL()-start_Ldistance<=50 && bot.getDistanceR()-start_Rdistance<=50){

// }


// curve90Circule(bot.getVx(),bot.getR(0.5));
// start_Ldistance=bot.getDistanceL();
// start_Rdistance=bot.getDistanceR();
// while (bot.getDistanceL()-start_Ldistance<=2*PI*(50-(bot.L/2)) && bot.getDistanceR()-start_Rdistance<=2*PI*(50+(bot.L/2)) )
// {

// }

}

