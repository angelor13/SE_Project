
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
    static constexpr uint8_t RST_PIN = INT8_MAX;   
    static constexpr uint8_t PIN_RFID_SDA=5;
    static constexpr uint8_t PIN_RFID_SCK=18;
    static constexpr uint8_t PIN_RFID_MISO=19;
    static constexpr uint8_t PIN_RFID_MOSI=23;

    // LIDAR constants
    static constexpr uint16_t PIN_SCL_FRONT = 22;
    static constexpr uint16_t PIN_SDA_FRONT = 21;
    static constexpr uint16_t PIN_XSHUT_FRONT=33;


    static constexpr uint16_t ADDR_LIDAR_FRONT= 0x71;

    static const uint32_t DIST_LIDAR_MIN = 0;
    static const uint32_t DIST_LIDAR_MAX = 2600;

    // SPI Pins

    // static constexpr uint16_t VSPI_MISO =33;
    // static constexpr uint16_t VSPI_MOSi= 25;
    // static constexpr uint16_t VSPI_SCLK =26;
    // static constexpr uint16_t VSPI_SS =27;
    // static constexpr uint16_t BUFFER_SIZE =10;




    // RFID constants
    static const byte TARGET_BLOCK= 60;
    static byte buffer[18];
    MFRC522::MIFARE_Key key={.keyByte = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}};
    // Internal variables

    // static TaskHandle_t mainTaskHandle;
    // static TaskHandle_t batteryTaskHandle;
    // static TaskHandle_t rfidTaskHandle;
    // static TaskHandle_t udpTaskHandle;


    // static AsyncServer *server;
    // static AsyncClient *tcpClients[MAX_TCP_CLIENTS];



    static MFRC522 RFID_device;

    static VL53L0X LidarFront;


    // Declarations



    //static void setupMotors();
    static void setupLidar();
    static void setupRFID();




    // static void taskMonitorBatteryValue(void*);
    // static void taskReadActiveRFIDValue(void*);
    // static void taskMonitorTargetPosition(void*);

public:
    static void begin();

    // static void moveMotorLeft(int16_t);
    // static void moveMotorRight(int16_t);
    // static void moveMotors(int16_t, int16_t);

    //static uint16_t getLidarLeftDistance();
    static uint16_t getFrontDistance();
    //static uint16_t getLidarRightDistance();
    static bool readCard(byte target_block, byte read_buffer[], byte length);

    static int writeBlock(int blockNumber, byte arrayAddress[]);

    static bool Tag_Detected();

    static void print(const char*);

    static void printI2C();
   // static void printLidarValue();


};

#endif