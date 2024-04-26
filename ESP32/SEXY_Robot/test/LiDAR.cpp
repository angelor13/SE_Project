#include <Arduino.h>
#include <SPI.h>
#include <VL53L0X.h>
#include <Wire.h>

VL53L0X LidarFront;

static constexpr uint16_t PIN_XSHUT_FRONT = 33;
static constexpr uint16_t PIN_SCL_FRONT = 22;
static constexpr uint16_t PIN_SDA_FRONT = 21;
static constexpr uint16_t PIN_XSHUT_FRONT = 33;
static constexpr uint16_t ADDR_LIDAR_FRONT= 0x71;

static const uint32_t DIST_LIDAR_MIN = 0;
static const uint32_t DIST_LIDAR_MAX = 2600;


void Lidar_Setup(){
    pinMode(PIN_XSHUT_FRONT, OUTPUT);
    digitalWrite(PIN_XSHUT_FRONT, LOW);
    delay(200);
    digitalWrite(PIN_XSHUT_FRONT, HIGH);
    delay(200);
    LidarFront.setAddress(ADDR_LIDAR_FRONT);
    LidarFront.setTimeout(500);
    LidarFront.init(true);
    LidarFront.startContinuous(0);
}

uint16_t GetFrontDistance(){
    uint16_t result = LidarFront.readRangeContinuousMillimeters();
    return constrain(result, DIST_LIDAR_MIN, DIST_LIDAR_MAX);
}

void setup(){
    
Serial.begin(115200);
Wire.begin();
Lidar_Setup();

}

void loop(){
    uint32_t distance=GetFrontDistance();
    Serial.println("Distância medida:"+(String)distance);

}


