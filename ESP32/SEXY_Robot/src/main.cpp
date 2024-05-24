#include <SEXY_ESP32.h>
#include "vec2.hpp"
#include <WebServer.h>

#define limiar 2

SEXY_ESP32 bot;
          
float float_map(float vx, float vin_min, float vin_max, float  vout_min, float  vout_max) {
  return (vx - vin_min) * (vout_max - vout_min) / (vin_max - vin_min) + vout_min;
}

#define Automatic 0
#define Manual 1

uint8_t current_mode=Automatic;

// void handleSwitchMode() {
//     String mode = server.hasArg("m") ? server.arg("m") : "0";

//     if (mode == "0") {
//         // AUTO
//         Serial.println("AUTO");
//     } else { 
//         // MANUAL
//         Serial.println("MANUAL");
//     }
//     server.send(200, "text/plain", "OK");
// }


void rotate_90_Stationary(){
  bot.moveMotors(0,0);
  delay(100);
  bot.moveMotors(100,-100);
  delay(225);
  bot.moveMotors(0,0);
  delay(100);
}


float leftW,rightW;
float last_align=0;

void curve90Circule(float vx,float R, float phi){
  bot.R=R;
  float w=vx/R;
  float dotl=bot.calculatedDotphiL(vx,w,bot.L,bot.r);
  float dotr=bot.calculatedDotphiR(vx,w,bot.L,bot.r);
  //   Serial.println("dot1:"+(String)dotl);
  // Serial.println("dot2:"+(String)dotr);
  leftW=float_map(bot.calculatedDotphiL(vx,w,bot.L,bot.r)/10,-bot.MAX_Vx,bot.MAX_Vx,-100.0,100.0);
  rightW=float_map(bot.calculatedDotphiR(vx,w,bot.L,bot.r)/10,-bot.MAX_Vx,bot.MAX_Vx,-100.0,100.0);

  if(vx<0){
    float aux=-leftW;
    leftW=-rightW;
    rightW=aux;
    vx=-vx;
  }
  // Serial.println("LeftW:"+(String)leftW);
  // Serial.println("RightW:"+(String)rightW);
bot.moveMotors(leftW,rightW);
long start_Ldistance=bot.getDistanceL();
long start_Rdistance=bot.getDistanceR();
bot.align=(start_Ldistance*sin(PI/4)+start_Rdistance*sin(PI/4))/2;
//  while (start_Ldistance>=bot.align+limiar || start_Rdistance<=bot.dotphiL-limiar|| (bot.getDistanceL()-start_Ldistance<=phi*((R*100)-(bot.L/2))/(2*PI) && bot.getDistanceR()-start_Rdistance<=phi*((R*100)+(bot.L/2)/(2*PI))))
//  {

//   start_Ldistance=bot.getDistanceL();
//   start_Rdistance=bot.getDistanceR();
//   bot.align=(start_Ldistance*sin(PI/4)+start_Rdistance*sin(PI/4))/2;

//  }
 }

void setup(){
  
  Serial.begin(115200);
  bot.begin();  
}

void loop() {

// if(current_mode==Automatic){
uint32_t left_distance=bot.getLeftDistance();
uint32_t front_distance=bot.getFrontDistance();
uint32_t right_distance=bot.getRightDistance();
bot.align=(left_distance*sin(PI/4)+right_distance*sin(PI/4))/2;

Serial.println("Esquerda: "+(String)left_distance+"  Center: "+(String)front_distance+"  Right: "+(String)right_distance);

if((left_distance<=40 && front_distance<=65)){
    while(left_distance<=40 && front_distance<=65){
    left_distance=bot.getLeftDistance();
    front_distance=bot.getFrontDistance();
    right_distance=bot.getRightDistance();

    bot.moveMotors(-10,-10);
    //delay(50);
  }
    bot.moveMotors(0,0);
    delay(100);

  rotate_90_Stationary();
  delay(200);
  bot.moveMotors(0,0);
  delay(200);
 }
else if((right_distance<=40 && front_distance<=65) || right_distance<=30){
    while(right_distance<=40 && front_distance<=65){
    left_distance=bot.getLeftDistance();
    front_distance=bot.getFrontDistance();
    right_distance=bot.getRightDistance();

    bot.moveMotors(-10,-10);
    // delay(50);
  }
    bot.moveMotors(0,0);
    delay(100);

  for(int i=0;i<3;i++){
  rotate_90_Stationary();
  delay(200);
  bot.moveMotors(0,0);
  delay(200);
  }
}
else if(front_distance<=65){
  while(front_distance<=65){
    left_distance=bot.getLeftDistance();
    front_distance=bot.getFrontDistance();
    right_distance=bot.getRightDistance();

    bot.moveMotors(-10,-10);
    // delay(50);
  }
    bot.moveMotors(0,0);
    delay(100);
  //   Bot.moveMotors(0,0);
  //   delay(500);
  rotate_90_Stationary();
  delay(200);
  bot.moveMotors(0,0);
  delay(200);

}


if(left_distance<=40 && front_distance<=60){
  bot.moveMotors(0,0);
  delay(200);
  bot.moveMotors(-50,-50);
  delay(50);
  bot.moveMotors(0,0);
  delay(200);
  rotate_90_Stationary();
  delay(200);
  bot.moveMotors(0,0);
  delay(200);
}
if(right_distance<=40 && front_distance<=60){
  bot.moveMotors(0,0);
  delay(200);
  bot.moveMotors(-50,-50);
  delay(50);
  bot.moveMotors(0,0);
  delay(200);
  for(int i=0;i<3;i++){
  rotate_90_Stationary();
  delay(200);
  bot.moveMotors(0,0);
  delay(200);
  }
}



if(left_distance>=650 && front_distance<=450){  //Deteta aberturas com lidars
  bot.moveMotors(0,0);
  delay(500);
  bot.moveMotors(10,10);
  delay(500);
   bot.moveMotors(0,0);
  delay(500);
  curve90Circule(bot.vx,0.2,PI/4);  // faz a curva no sentido anti-horário
  delay(100);
  long start=millis();
//|| front_distance<=250||right_distance<=300
  while(left_distance>=300 || millis()-start<=700){ // Faz a curva enquanto as seguintes condições
  // Volta a ler distâncias
  left_distance=bot.getLeftDistance();
  front_distance=bot.getFrontDistance();
  right_distance=bot.getRightDistance();
    if(left_distance<=270){
      break;
    }
  }
  bot.moveMotors(0,0);
  delay(200);
  }
// //Serial.println("  Esquerda: "+(String)left_distance+" Frente:  "+(String)front_distance+"  Direita:  "+(String)right_distance);

// // Virar á direita, sendo que esta função poderá ser substituida pelo rotate_90_Stationary() pu apena pela navegação natural


if(right_distance>=600 && front_distance<=450) {

  bot.moveMotors(0,0);
  delay(200);
  bot.moveMotors(10,10);
  delay(600);
   bot.moveMotors(0,0);
  delay(500);
  curve90Circule(-bot.vx,0.2,PI/4);  // faz a curva no sentido anti-horário
  delay(100);
  long start=millis();
  //|| left_distance>=400|| 
  while(right_distance>=300 || millis()-start<=800){
    // Volta a ler distâncias
  left_distance=bot.getLeftDistance();
  front_distance=bot.getFrontDistance();
  right_distance=bot.getRightDistance();
  if(right_distance<=270){
    break;
  }
  }
    bot.moveMotors(0,0);
    delay(100);
  }

//  // Navegação natural seguindo a parede esquerda
  if((left_distance>=bot.align+limiar || left_distance<=bot.align-limiar) && front_distance>=65){
     last_align=(float)bot.align/1000000.0;
     Serial.println("Last:"+(String)last_align);
    float left_velocity=float_map(constrain(log(left_distance+limiar)+50,50,log(2*bot.align)+50),50,log(800)+50,100,-5);
    float right_velocity=float_map(constrain(log(left_distance+limiar)+50,50,log(2*bot.align)+50),50,log(800)+50,-40,60);
    bot.moveMotors(0.5*3.1*left_velocity,0.5*0.3*right_velocity);
  }

// Serial.println("x:"+(String)bot.robot_pos.x+"y:"+(String)bot.robot_pos.y+"phi:"+(String)bot.robot_pos.phi);

delay(50);

  // bot.moveMotors(-100,-100);
  // for(int i=-100;i<=100;i++){
  //   bot.moveMotors(i,i);
  //   delay(100);
  // }
  // delay(100);
  
// }

// else{

// }



}
