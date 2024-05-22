#include <SEXY_ESP32.h>
#include "vec2.hpp"

#define limiar 2

SEXY_ESP32 bot;
          
float float_map(float vx, float vin_min, float vin_max, float  vout_min, float  vout_max) {
  return (vx - vin_min) * (vout_max - vout_min) / (vin_max - vin_min) + vout_min;
}

void rotate_90_Stationary(){
  bot.moveMotors(0,0);
  delay(100);
  bot.moveMotors(100,-100);
  delay(225);
  bot.moveMotors(0,0);
  delay(100);
}


float leftW,rightW;

void curve90Circule(float vx,float R, float phi){
  bot.R=R;
  float w=vx/R;
  leftW=float_map(bot.calculatedDotphiL(vx,w,bot.L,bot.getDotphiL()),-bot.MAX_Vx,bot.MAX_Vx,-100,100);
  rightW=float_map(bot.calculatedDotphiR(vx,w,bot.L,bot.getDotphiR()),-bot.MAX_Vx,bot.MAX_Vx,-100,100);

  if(vx<0){
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
}

void setup(){
  
  Serial.begin(115200);
  bot.begin();  
}

byte rx[4];
byte tx=10;



void loop() {


// uint32_t left_distance=bot.getLeftDistance();
// uint32_t front_distance=bot.getFrontDistance();
// uint32_t right_distance=bot.getRightDistance();
// bot.align=(left_distance*sin(PI/4)+right_distance*sin(PI/4))/2;

// Serial.println("Esquerda: "+(String)left_distance+"  Center: "+(String)front_distance+"  Right: "+(String)right_distance);

// if(left_distance<=40 && front_distance<=60){
//   bot.moveMotors(0,0);
//   delay(200);
//   bot.moveMotors(-50,-50);
//   delay(50);
//   bot.moveMotors(0,0);
//   delay(200);
//   rotate_90_Stationary();
//   delay(200);
//   bot.moveMotors(0,0);
//   delay(200);
// }
// if(right_distance<=40 && front_distance<=60){
//   bot.moveMotors(0,0);
//   delay(200);
//   bot.moveMotors(-50,-50);
//   delay(50);
//   bot.moveMotors(0,0);
//   delay(200);
//   for(int i=0;i<3;i++){
//   rotate_90_Stationary();
//   delay(200);
//   bot.moveMotors(0,0);
//   delay(200);
//   }
// }

//   if(left_distance>=400 && front_distance<=950){  //Deteta aberturas com lidars

//   bot.moveMotors(0,0);
//   delay(200);
//   curve90Circule(bot.vx,0.25,PI/4);  // faz a curva no sentido anti-horário
//   long start=millis();

//   while(left_distance>=bot.align+limiar || right_distance<=bot.align-limiar|| millis()-start<=800){ // Faz a curva enquanto as seguintes condições
//   // Volta a ler distâncias
//   left_distance=bot.getLeftDistance();
//   front_distance=bot.getFrontDistance();
//   right_distance=bot.getRightDistance();

//     if(left_distance<=40){
//     break;
//   }
//   }
//   bot.moveMotors(0,0);
//   delay(100);
//  }
// //Serial.println("  Esquerda: "+(String)left_distance+" Frente:  "+(String)front_distance+"  Direita:  "+(String)right_distance);

// // Virar á direita, sendo que esta função poderá ser substituida pelo rotate_90_Stationary() pu apena pela navegação natural


// if(right_distance>=400 && front_distance<=950) {
//   int32_t left_velocity=float_map(constrain(left_distance,0,1300),0,1300,100,12);
//   int32_t right_velocity=float_map(constrain(left_distance,0, 1300),0,1300,1,59);
//   bot.moveMotors(0,0);
//   delay(200);
//   curve90Circule(-bot.vx,0.2,PI/4);  // faz a curva no sentido anti-horário
//   long start=millis();
//   while(left_distance>=bot.align+limiar || right_distance>=bot.align-limiar||  millis()-start<=900){
//     // Volta a ler distâncias
//   left_distance=bot.getLeftDistance();
//   front_distance=bot.getFrontDistance();
//   right_distance=bot.getRightDistance();
//   if(right_distance<=40){
//     break;
//   }
//   }
//     bot.moveMotors(0,0);
//     delay(100);
//  }

//  // Navegação natural seguindo a parede esquerda

//   if((left_distance>=bot.align+limiar || left_distance<=bot.align-limiar ) && front_distance>=65){
    
//     float left_velocity=float_map(constrain(log(left_distance+limiar)+50,50,log(2*bot.align)+50),50,log(1300)+50,100,-4);
//     float right_velocity=float_map(constrain(log(left_distance+limiar)+50,50,log(2*bot.align)+50),50, log(1300)+50,-14,98);
//     bot.moveMotors(2.3*left_velocity,0.8*right_velocity);
//   }


  bot.moveMotors(10,10);
  delay(10);



}
