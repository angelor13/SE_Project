#include <SEXY_ESP32.h>
#include "vec2.hpp"

#define limiar 2

SEXY_ESP32 bot;
          
float float_map(float vx, float vin_min, float vin_max, float  vout_min, float  vout_max) {
  return (vx - vin_min) * (vout_max - vout_min) / (vin_max - vin_min) + vout_min;
}


float leftW,rightW;

void curve90Circule(float vx,float R, float phi){
  bot.R=R;
  bot.changeCurvingState();
  switch (bot.currentDirection)
  {
  case bot.FRONT:
      bot.currentDirection=bot.LEFT;
      break;
  case bot.LEFT:
      bot.currentDirection=bot.BACK;
      break;
  case bot.BACK:
      bot.currentDirection=bot.RIGHT;
      break;
  case bot.RIGHT:
      bot.currentDirection=bot.FRONT;
      break;
  default:
      break;
  }
  float w=vx/R;
  leftW=float_map(bot.calculatedDotphiL(vx,w,bot.L,bot.getDotphiL()),-bot.MAX_Vx,bot.MAX_Vx,-100,100);
  rightW=float_map(bot.calculatedDotphiR(vx,w,bot.L,bot.getDotphiR()),-bot.MAX_Vx,bot.MAX_Vx,-100,100);

  if(vx<0){
switch (bot.currentDirection)
{
case bot.FRONT:
    bot.currentDirection=bot.RIGHT;
    break;
case bot.LEFT:
    bot.currentDirection=bot.FRONT;
    break;
case bot.BACK:
    bot.currentDirection=bot.LEFT;
    break;
case bot.RIGHT:
    bot.currentDirection=bot.BACK;
    break;
default:
    break;
}
    float aux=-leftW;
    leftW=-rightW;
    rightW=aux;
    vx=-vx;
  }
  
bot.moveMotors(leftW,rightW);
long start_Ldistance=bot.getDistanceL();
long start_Rdistance=bot.getDistanceR();
bot.align=(start_Ldistance*sin(PI/4)+start_Rdistance*sin(PI/4))/2;
 while (start_Ldistance>=bot.align+limiar || start_Rdistance<=bot.dotphiL-limiar|| (bot.getDistanceL()-start_Ldistance<=phi*((R*100)-(bot.L/2))/(2*PI) && bot.getDistanceR()-start_Rdistance<=phi*((R*100)+(bot.L/2)/(2*PI))))
 {

  start_Ldistance=bot.getDistanceL();
  start_Rdistance=bot.getDistanceR();
  bot.align=(start_Ldistance*sin(PI/4)+start_Rdistance*sin(PI/4))/2;

 }
 bot.changeCurvingState();
}
// void navegacao(){
//   if(left_distance>=400 && front_distance<=950){  //Deteta aberturas com lidars

//   Bot.moveMotors(0,0);
//   delay(200);
//   curve90Circule(vx,0.25);  // faz a curva no sentido anti-horário
//   long start=millis();

//   while(left_distance>=align+limiar || right_distance<=align-limiar|| millis()-start<=800){ // Faz a curva enquanto as seguintes condições
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


// if(right_distance>=400 && front_distance<=950 {
//   int32_t left_velocity=float_map(constrain(left_distance,0,1300),0,1300,100,12);
//   int32_t right_velocity=float_map(constrain(left_distance,0, 1300),0,1300,1,59);
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
  
//   left_velocity=float_map(constrain(log(left_distance+limiar)+50,50,log(2*align)+50),50,log(1300)+50,100,-4);
//   right_velocity=float_map(constrain(log(left_distance+limiar)+50,50,log(2*align)+50),50, log(1300)+50,-14,98);
//   Bot.moveMotors(2.3*left_velocity,0.8*right_velocity);
// }
//}
void setup(){
  
  Serial.begin(115200);
  bot.begin();  
}

byte rx[4];
byte tx=10;











void loop() {
  delay(1);
  // if(bot.getTagDetected()){
  //   Serial.println("Tag detected");
  // }
// bot.setMotorVelocity(5,5);
bot.moveMotors(100,100);
delay(40);
  
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

//    vec2 deltas = bot.getMotorDeltas();

//     Serial.printf("Left: %.5f, Right: %.5f\n", deltas.x, deltas.y);

//     delay(100);


}
