#include <Arduino.h>
#include <SPI.h>

float currentPos;

static float Kp=0;    // Proporcitional constant from PID Controller
static float Ki=0;    // Integrative constant from PID Controller
static float Kd=0;    // Derivative constant from PID Controller


float previousError=0;

float integral=0;
long previousTime=0;


float PID_Controller(float target){
    
    long currentTime=micros();
    
    float deltaTime=(float)(currentTime-previousTime)/10*(-6);

    float error= currentPos-target;

    float derivate=(error-previousError)/deltaTime;

    integral+= error*deltaTime;

    float output=(Kp*error)+(Kd*derivate)+(Ki*integral);


    previousTime=currentTime;
    previousError=error;

return output;
}










