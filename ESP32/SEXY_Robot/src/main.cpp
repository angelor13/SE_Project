#include <Arduino.h>
#include <SPI.h>
#include <VL53L0X.h>
#include <MFRC522.h> 
#include <Wire.h>
//#include <DistanceGP2Y0A21YK.h>

constexpr uint8_t RST_PIN = 9;     // Configurable, see typical pin layout above
constexpr uint8_t SS_PIN = 10;     // Configurable, see typical pin layout above


static constexpr uint16_t PIN_XSHUT_FRONT = 33;
static constexpr uint16_t ADDR_LIDAR_FRONT= 0x70;

static constexpr uint32_t DIST_LIDAR_MIN = 0;
static constexpr uint32_t DIST_LIDAR_MAX = 2600;

#define VSPI_MISO 33
#define VSPI_MOSi 25
#define VSPI_SCLK 26
#define VSPI_SS 27
#define BUFFER_SIZE 10

VL53L0X LidarFront;
MFRC522 mfrc522(SS_PIN, RST_PIN);   // Create MFRC522 instance.




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

SPI.begin();                  // Init SPI bus
mfrc522.PCD_Init();           // Init MFRC522 card    
Serial.begin(115200);
Wire.begin();
Lidar_Setup();

}

void loop(){
    uint32_t distance=GetFrontDistance();
    Serial.println("Distância medida:"+(String)distance);

}
