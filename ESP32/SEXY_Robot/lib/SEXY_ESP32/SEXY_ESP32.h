
#ifndef SEXYESP32_H
#define SEXYESP32_H


#include <Arduino.h>
#include <SPI.h>
#include <VL53L0X.h>
#include <MFRC522.h>
#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <WiFi.h>
#include <vector>
#include <vec3.hpp>

//#include <WiFiUdp.h>

class SEXY_ESP32 {

private:                        // Index Map:
    static byte RxBuffer[4];    //  0 -> dotphiL        1-> dotphiR         2->             3->
    static byte TxBuffer[4];    //  0 -> MotorL_PWM     1-> MotorR_PWM      2->             3->

    static std::vector<vec3> mapPointCloud;

    static float distanceMotorR,distanceMotorL;
    static long previous_millis;

    // Pin constants


    // Motor Pins
    static constexpr uint16_t PIN_MOTOR_L_1 = 17;
    static constexpr uint16_t PIN_MOTOR_L_2 = 16;
    static constexpr uint16_t PIN_MOTOR_R_1 = 13;
    static constexpr uint16_t PIN_MOTOR_R_2 = 27;

    static constexpr uint8_t PWM_RESOLUTION_BITS = 9;
    static constexpr int16_t DUTY_MOTOR_MAX = 512 - 1;

    // RFID pins
    static constexpr uint8_t RST_PIN = INT8_MAX;   
    static constexpr uint8_t PIN_RFID_SDA=5;       // pin 4 in robot
    static constexpr uint8_t PIN_RFID_SCK=18;
    static constexpr uint8_t PIN_RFID_MISO=19;
    static constexpr uint8_t PIN_RFID_MOSI=23;

    // LIDAR constants

    // Left Sensor 1
    static constexpr uint16_t PIN_VP_LEFT=36;

    // Front Sensor 2
    static constexpr uint16_t PIN_SCL_FRONT = 33;
    static constexpr uint16_t PIN_SDA_FRONT = 32;
    static constexpr uint16_t PIN_XSHUT_FRONT=17;

    // TO test in our robot

    // static constexpr uint16_t PIN_SCL_FRONT = 22;
    // static constexpr uint16_t PIN_SDA_FRONT = 21;
    // static constexpr uint16_t PIN_XSHUT_FRONT=33;

    // Right Sensor 3
    static constexpr uint16_t PIN_VP_RIGHT=34;



    static constexpr uint16_t ADDR_LIDAR_FRONT= 0x71;

    static const uint32_t DIST_LIDAR_MIN = 0;
    static const uint32_t DIST_LIDAR_MAX = 2600;


    // ADC  Pins
    static constexpr uint16_t ADDR_ADC=0x48;    //0x49
    static constexpr uint16_t PIN_ADC_SCL=22;
    static constexpr uint16_t PIN_ADC_SDA=21;   

    // SPI Pins

    static constexpr uint16_t VSPI_MISO=19;
    static constexpr uint16_t VSPI_MOSi=23;
    static constexpr uint16_t VSPI_SCLK=18;
    static constexpr uint16_t VSPI_SS=5;

    // To use on breadboard

    // static constexpr uint16_t VSPI_MISO=33;
    // static constexpr uint16_t VSPI_MOSi=25;
    // static constexpr uint16_t VSPI_SCLK=26;
    // static constexpr uint16_t VSPI_SS=5;

    
    static constexpr uint16_t BUFFER_SIZE=4;


    // WIFI constants

    static constexpr char ssid[]="";
    static constexpr char password[]="";


    static bool isTagDetected;

    // Robot Velocity

    

    
    // Tansmit SPI COM
    static void transmitSPIcom();


    // Tasks Handles_t

    static TaskHandle_t taskReadRFIDHandle;
    static TaskHandle_t taskReceiveSPiComHandle;
    static TaskHandle_t taskGetPointCloudHandle;
    static TaskHandle_t taskUpdatePositionHandle;


    // TaskFunctions_t
    static void taskReadRFID(void*);
    static void taskReceiveSPICom(void*);
    static void taskGetPointCloud(void*);
    static void taskUpdatePosition(void*);

    


    // Internal variables


    // Devices
    static MFRC522 RFID_device;
    static VL53L0X LidarFront;
    static Adafruit_ADS1115 gasADC;
    //static WiFiUDP wifi;

    // Declarations
    static void setupWifi();
    static void setupADC();
    static void setupSharps();
    static void setupLidar();
    static void setupRFID();
    static void setupMotors();
    static void setupSPI();


public:
    // Robot Atributes
    static float L,r,dotphiL,dotphiR,vx,w,R;
    
    static constexpr float MAX_DOTPHI=40;

    struct SEXY_POS
    {
        float x=0;
        float y=0;
        float phi=0;
        float vetor[2]={1,0};
    };

    static SEXY_POS robot_pos;
    

    // RFID constants
    static const byte TARGET_BLOCK= 60;
    static byte buffer[18];
    static constexpr uint8_t N_SECTORS=15;   //Sectors number

    static constexpr MFRC522::MIFARE_Key key={.keyByte = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}};

    static void begin();
 
    static void moveMotorLeft(int16_t);
    static void moveMotorRight(int16_t);
    static void moveMotors(int16_t, int16_t);
    static void stopMotors();

    static uint16_t getLeftDistance();
    static uint16_t getFrontDistance();
    static uint16_t getRightDistance();

    static uint16_t getADCvalue();

    static bool readCard(byte target_block, byte read_buffer[], byte length);
    static int writeBlock(int blockNumber, byte arrayAddress[]);
    static bool Tag_Detected();

    static void printI2C();

    static bool getTagDetected();

    // Calculation Functions
    static float calculatedDotphiL(const float vx,const float w, const float L,const float r);
    static float calculatedDotphiR(const float vx,const float w, const float L,const float r);
    static float calculatedVx(const float dotphiR,const float dotphiLL,const float r);
    static float calculatedW(const float dotphiR,const float dotphiL,const float L,const float r);

    static float getDotphiL();          // Maybe unused
    static float getDotphiR();          // Maybe unused
    static float getVx();
    static float getW();
    static float getR(float Raio);

    // Other functions
    static float getDistanceR();
    static float getDistanceL();
    
};

#endif