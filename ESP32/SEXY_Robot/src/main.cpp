#include <SEXY_ESP32.h>
#include "vec2.hpp"


SEXY_ESP32 bot;
          
float float_map(float vx, float vin_min, float vin_max, float  vout_min, float  vout_max) {
  return (vx - vin_min) * (vout_max - vout_min) / (vin_max - vin_min) + vout_min;
}


float leftW,rightW;

void curve90Circule(float vx,float R){
  float w=vx/R;
  leftW=float_map(bot.calculatedDotphiL(vx,w,bot.L,bot.getDotphiL()),-bot.MAX_DOTPHI,bot.MAX_DOTPHI,-100,100);
  rightW=float_map(bot.calculatedDotphiR(vx,w,bot.L,bot.getDotphiR()),-bot.MAX_DOTPHI,bot.MAX_DOTPHI,-100,100);

  if(vx<0){
    float aux=-leftW;
    leftW=-rightW;
    rightW=aux;
    vx=-vx;
  }
  
bot.moveMotors(leftW,rightW);
}
// void navegacao(){
//   if(left_distance>=400 && front_distance<=1000 && right_distance>=100){  //Deteta aberturas com lidars

//   Bot.moveMotors(0,0);
//   delay(200);
//   curve90Circule(vx,0.25);  // faz a curva no sentido anti-horário
//   long start=millis();

//   while(left_distance>=align+limiar || right_distance<=align-limiar|| millis()-start<=900){ // Faz a curva enquanto as seguintes condições
//   // Volta a ler distâncias
//   left_distance=Bot.getLidarLeftDistance();
//   front_distance=Bot.getLidarFrontDistance();
//   right_distance=Bot.getLidarRightDistance();

//     if(left_distance<=40){
//     break;
//   }
//   }
//   Bot.moveMotors(0,0);
//   delay(100);
//  }
// //Serial.println("  Esquerda: "+(String)left_distance+" Frente:  "+(String)front_distance+"  Direita:  "+(String)right_distance);

// // Virar á direita, sendo que esta função poderá ser substituida pelo rotate_90_Stationary() pu apena pela navegação natural


// if(right_distance>=400 && front_distance<=1000 && right_distance>=100){
//   int32_t left_velocity=float_map(constrain(left_distance,0,1300),0,1300,511,60);
//   int32_t right_velocity=float_map(constrain(left_distance,0, 1300),0,1300,5,300);
//   Bot.moveMotors(0,0);
//   delay(200);
//   curve90Circule(-vx,0.2);  // faz a curva no sentido anti-horário
//   long start=millis();
//   while(left_distance>=align+limiar || right_distance>=align-limiar||  millis()-start<=900){
//     // Volta a ler distâncias
//   left_distance=Bot.getLidarLeftDistance();
//   front_distance=Bot.getLidarFrontDistance();
//   right_distance=Bot.getLidarRightDistance();
//   if(left_distance<=40){
//     break;
//   }
//   }
//     Bot.moveMotors(0,0);
//   delay(100);
//  }

//  // Navegação natural seguindo a parede esquerda

// if((left_distance>=align+limiar || left_distance<=align-limiar ) && front_distance>=65){
  
//   left_velocity=float_map(constrain(log(left_distance+limiar)+50,50,log(2*align)+50),50,log(1300)+50,511,-20);
//   right_velocity=float_map(constrain(log(left_distance+limiar)+50,50,log(2*align)+50),50, log(1300)+50,-70,500);
//   Bot.moveMotors(2.3*left_velocity,0.8*right_velocity);
// }
//}
void setup(){
bot.begin();  
}

byte rx[4];
byte tx=10;





/// @brief Get motor velocities measured from encoders in [#PULSES] / [#MILLISECONDS]. Negative values mean inverted direction.
/// @return vec2(LEFT_VELOCITY, RIGHT_VELOCITY)
vec2 getMotorDeltas() {
    int32_t rxdata[2];

    digitalWrite(5, 0);
    SPI.transfer(0xAB);
    SPI.transfer(0xCD);
    SPI.transfer(rxdata, sizeof(rxdata));
    digitalWrite(5, 1);

    float left_velocity = (rxdata[0] / 1.0f);
    float right_velocity = (rxdata[1] / 1.0f);
    
    return vec2(left_velocity, right_velocity);
}

/// @brief Set motor velocities in [#PULSES] / [#MILLISECONDS]. Negative values mean inverted direction. [TO BE IMPLEMENTED ON STM32]
void setMotorDeltas(float left_velocity, float right_velocity) {
    int32_t txdata[2] = { (int32_t) left_velocity, (int32_t) right_velocity };

    digitalWrite(5, 0);
    SPI.transfer(0xDE);
    SPI.transfer(0xAD);
    SPI.transfer(txdata, sizeof(txdata));
    digitalWrite(5, 1);
}



void loop() {
  delay(1);
  // if(bot.getTagDetected()){
  //   Serial.println("Tag detected");
  // }


  
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
   vec2 deltas = getMotorDeltas();

    Serial.printf("Left: %.5f, Right: %.5f\n", deltas.x, deltas.y);

    delay(100);

}

