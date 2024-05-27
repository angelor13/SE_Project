#include <SEXY_ESP32.h>
#include "vec2.hpp"
#include <WebServer.h>

#define limiar 2

SEXY_ESP32 bot;
          
float float_map(float vx, float vin_min, float vin_max, float  vout_min, float  vout_max) {
  return (vx - vin_min) * (vout_max - vout_min) / (vin_max - vin_min) + vout_min;
}

String ON="on";
String OFF="off";



void sendCoordenates(){
  WiFiClient client =bot.server.available(); 
  if (client) {                             // if you get a client,
    Serial.println("New Client.");           // print a message out the serial port
    String currentLine = "";                // make a String to hold incoming data from the client
    while (client.connected()) {            // loop while the client's connected
      if (client.available()) {             // if there's bytes to read from the client,
        char c = client.read();             // read a byte, then
        Serial.write(c);                    // print it out the serial monitor
        if (c == '\n') {                    // if the byte is a newline character

          // if the current line is blank, you got two newline characters in a row.
          // that's the end of the client HTTP request, so send a response:
          if (currentLine.length() == 0) {
            // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
            // and a content-type so the client knows what's coming, then a blank line:
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/html");
            client.println();

            // the content of the HTTP response follows the header:
            client.print("Click <a href=\"/H\">here</a> to turn the LED on pin 5 on.<br>");
            client.print("Click <a href=\"/L\">here</a> to turn the LED on pin 5 off.<br>");

            
            client.printf("\n Send coordinates: ");
            bot.robot_pos.x,bot.robot_pos.y,bot.robot_pos.phi = client.read();
            delay(500);

            
            // The HTTP response ends with another blank line:
            client.println();
            // break out of the while loop:
            break;
          } else {    // if you got a newline, then clear currentLine:
            currentLine = "";
          }
        } else if (c != '\r') {  // if you got anything else but a carriage return character,
          currentLine += c;      // add it to the end of the currentLine
        }

        // Check to see if the client request was "GET /H" or "GET /L":
        if (currentLine.endsWith("GET /H")) {
          digitalWrite(15, HIGH);               // GET /H turns the LED on
        }
        if (currentLine.endsWith("GET /L")) {
          digitalWrite(15, LOW);                // GET /L turns the LED off
        }
      }
    }
    // close the connection:
    client.stop();
    Serial.println("Client Disconnected.");
    Serial.printf("Received coordinates: %d %d  %d", bot.robot_pos.x,bot.robot_pos.y,bot.robot_pos.phi);
}
}
void switch_mode(){
  if(bot.current_mode==ON){
    bot.current_mode=OFF;
  }
  else{
    bot.current_mode=ON;
  }
}

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
  bot.rotating=true;
  bot.moveMotors(0,0);
  delay(100);
  bot.moveMotors(100,-100);
  delay(225*2.5);
  bot.moveMotors(0,0);
  delay(100);
  bot.rotating=false;
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

// if(current_mode==ON){
uint32_t left_distance=bot.getLeftDistance();
uint32_t front_distance=bot.getFrontDistance();
uint32_t right_distance=bot.getRightDistance();
bot.align=(left_distance*sin(PI/4)+right_distance*sin(PI/4))/2;

Serial.println("Esquerda: "+(String)left_distance+"  Center: "+(String)front_distance+"  Right: "+(String)right_distance);

// if((left_distance<=40 && front_distance<=65)){
//     while(left_distance<=40 && front_distance<=65){
//     left_distance=bot.getLeftDistance();
//     front_distance=bot.getFrontDistance();
//     right_distance=bot.getRightDistance();

//     bot.moveMotors(-10,-10);
//     //delay(50);
//   }
//     bot.moveMotors(0,0);
//     delay(100);

//   rotate_90_Stationary();
//   delay(200);
//   bot.moveMotors(0,0);
//   delay(200);
//  }
// else if((right_distance<=40 && front_distance<=65) || right_distance<=30){
//     while(right_distance<=40 && front_distance<=65){
//     left_distance=bot.getLeftDistance();
//     front_distance=bot.getFrontDistance();
//     right_distance=bot.getRightDistance();

//     bot.moveMotors(-10,-10);
//     // delay(50);
//   }
//     bot.moveMotors(0,0);
//     delay(100);

//   for(int i=0;i<3;i++){
//   rotate_90_Stationary();
//   delay(200);
//   bot.moveMotors(0,0);
//   delay(200);
//   }
// }
// else if(front_distance<=65){
//   while(front_distance<=65){
//     left_distance=bot.getLeftDistance();
//     front_distance=bot.getFrontDistance();
//     right_distance=bot.getRightDistance();

//     bot.moveMotors(-10,-10);
//     // delay(50);
//   }
//     bot.moveMotors(0,0);
//     delay(100);
//   //   Bot.moveMotors(0,0);
//   //   delay(500);
//   rotate_90_Stationary();
//   delay(200);
//   bot.moveMotors(0,0);
//   delay(200);

// }


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



if(left_distance>=600 && front_distance<=450){  //Deteta aberturas com lidars
  bot.moveMotors(0,0);
  delay(500);
  bot.moveMotors(10,10);
  delay(500*2.3);
   bot.moveMotors(0,0);
  delay(500);
  curve90Circule(bot.vx,last_align/2,PI/4);  // faz a curva no sentido anti-horário
  delay(100);
  long start=millis();
//|| front_distance<=250||right_distance<=300
  while(left_distance>=300 || millis()-start<=700){ // Faz a curva enquanto as seguintes condições
  // Volta a ler distâncias
  left_distance=bot.getLeftDistance();
  front_distance=bot.getFrontDistance();
  right_distance=bot.getRightDistance();
    if(left_distance<=300){
      break;
    }
  }
  bot.moveMotors(0,0);
  delay(200);
  }
// // // //Serial.println("  Esquerda: "+(String)left_distance+" Frente:  "+(String)front_distance+"  Direita:  "+(String)right_distance);

// // // // Virar á direita, sendo que esta função poderá ser substituida pelo rotate_90_Stationary() pu apena pela navegação natural


if(right_distance>=600 && front_distance<=450) {
  //Serial.println("START");
  bot.moveMotors(0,0);
  delay(200);
  bot.moveMotors(100,100);
  delay(600);
   bot.moveMotors(0,0);
  delay(500);
  curve90Circule(-bot.vx,last_align/2,PI/4);  // faz a curva no sentido anti-horário
  delay(100*20);
  long start=millis();
  //|| left_distance>=400|| 
  // while( millis()-start<=1900){
  //   // Volta a ler distâncias
  // left_distance=bot.getLeftDistance();
  // front_distance=bot.getFrontDistance();
  // right_distance=bot.getRightDistance();
  // if(right_distance<=300 ){
  //   break;
  // }
  // }
    bot.moveMotors(0,0);
    delay(100);
  }


// //  // Navegação natural seguindo a parede esquerda
  if((left_distance>=bot.align+limiar || left_distance<=bot.align-limiar) && front_distance>=65){
     last_align=(float)bot.align/1000.0;
     Serial.println("Last:"+(String)last_align);
    float left_velocity=float_map(constrain(log(left_distance+limiar)+50,50,log(2*bot.align)+50),50,log(800)+50,100,-5);
    float right_velocity=float_map(constrain(log(left_distance+limiar)+50,50,log(2*bot.align)+50),50,log(800)+50,-40,60);
    bot.moveMotors(0.5*3.7*left_velocity,0.5 *0.3*right_velocity);
  }

// Serial.println("x:"+(String)bot.robot_pos.x+"y:"+(String)bot.robot_pos.y+"phi:"+(String)bot.robot_pos.phi);

delay(50);

 
  // for(int i=-100;i<=100;i++){
  //   bot.moveMotors(i,i);
  //   delay(100);
  // }
  // delay(2000);
  // bot.moveMotors(0,0);
  // delay(2000);
  // bot.moveMotors(500,500);
  // delay(2000);

  
//}

// else{
  // bot.stopMotors();
  // delay(50);
// }



}
