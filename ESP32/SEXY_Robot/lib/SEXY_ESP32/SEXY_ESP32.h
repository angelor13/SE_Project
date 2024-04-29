
#ifndef SEXYESP32_H
#define SEXYESP32_H


#include <Arduino.h>
#include <SPI.h>
#include <VL53L0X.h>
#include <MFRC522.h>
#include <Wire.h>


class SEXY_ESP32 {
private:
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
    static constexpr uint8_t PIN_RFID_SDA=5;
    static constexpr uint8_t PIN_RFID_SCK=18;
    static constexpr uint8_t PIN_RFID_MISO=19;
    static constexpr uint8_t PIN_RFID_MOSI=23;

    // LIDAR constants

    // Right Sensor 1
    static constexpr uint16_t PIN_VP_LEFT=36;

    // Front Sensor 2
    static constexpr uint16_t PIN_SCL_FRONT = 33;
    static constexpr uint16_t PIN_SDA_FRONT = 32;
    static constexpr uint16_t PIN_XSHUT_FRONT=17;
    // static constexpr uint16_t PIN_SCL_FRONT = 22;
    // static constexpr uint16_t PIN_SDA_FRONT = 21;
    // static constexpr uint16_t PIN_XSHUT_FRONT=33;

    // Left Sensor 3
    static constexpr uint16_t PIN_VP_RIGHT=34;



    static constexpr uint16_t ADDR_LIDAR_FRONT= 0x71;

    static const uint32_t DIST_LIDAR_MIN = 0;
    static const uint32_t DIST_LIDAR_MAX = 2600;


    // ADC  Pins
    
    static constexpr uint16_t PIN_ADC_SCL=22;
    static constexpr uint16_t PIN_ADC_SDA=21;

    // SPI Pins

    static constexpr uint16_t VSPI_MISO=19;
    static constexpr uint16_t VSPI_MOSi=23;
    static constexpr uint16_t VSPI_SCLK=18;
    static constexpr uint16_t VSPI_SS=5;
    static constexpr uint16_t BUFFER_SIZE=4;



    static bool isTagDetected;


    // Tasks Handles_t

    static TaskHandle_t taskReadRFIDHandle;
    // static void taskMonitorBatteryValue(void*);



    // TaskFunctions_t

    static void taskReadRFID(void*);


    // Internal variables


    // Devices
    static MFRC522 RFID_device;
    static VL53L0X LidarFront;


    // Declarations



    static void setupLidar();
    static void setupRFID();
    static void setupMotors();


public:
    // RFID constants
    static const byte TARGET_BLOCK= 60;
    static byte buffer[18];
    static constexpr uint8_t N_SECTORS =15;   //Sectors number

    static constexpr MFRC522::MIFARE_Key key={.keyByte = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}};

    static void begin();
    static void SPIsetup();
    static void moveMotorLeft(int16_t);
    static void moveMotorRight(int16_t);
    static void moveMotors(int16_t, int16_t);

    //static uint16_t getLidarLeftDistance();
    static uint16_t getFrontDistance();
    //static uint16_t getLidarRightDistance();
    static bool readCard(byte target_block, byte read_buffer[], byte length);

    static int writeBlock(int blockNumber, byte arrayAddress[]);

    static bool Tag_Detected();

    static void print(const char*);

    static void printI2C();

    static bool getTagDetected();


};

#endif