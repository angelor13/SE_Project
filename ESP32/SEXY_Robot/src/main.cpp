#include <Arduino.h>
#include <SPI.h>
#include <VL53L0X.h>
#include <MFRC522.h> 
#include <Wire.h>
#include <DistanceGP2Y0A21YK.h>




// RFID PINOUT
constexpr uint8_t RST_PIN = INT8_MAX;   
constexpr uint8_t PIN_RFID_SDA=5;
constexpr uint8_t PIN_RFID_SCK=18;
constexpr uint8_t PIN_RFID_MISO=19;
constexpr uint8_t PIN_RFID_MOSI=23;

MFRC522::MIFARE_Key key = {.keyByte = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}};

// Lidar PINOUT

static constexpr uint16_t PIN_XSHUT_FRONT = 33;
static constexpr uint16_t PIN_SCL_FRONT = 22;
static constexpr uint16_t PIN_SDA_FRONT = 21;
static constexpr uint16_t PIN_XSHUT_FRONT = 33;
static constexpr uint16_t ADDR_LIDAR_FRONT= 0x70;

static constexpr uint32_t DIST_LIDAR_MIN = 0;
static constexpr uint32_t DIST_LIDAR_MAX = 2600;



byte data[12] = {"Embebed-Sys"};


#define VSPI_MISO 33
#define VSPI_MOSi 25
#define VSPI_SCLK 26
#define VSPI_SS 27
#define BUFFER_SIZE 10

VL53L0X LidarFront;
MFRC522 mfrc522(PIN_RFID_SDA, RST_PIN);   // Create MFRC522 instance.


DistanceGP2Y0A21YK Dist;


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
  Dist.begin(0);
}




byte TARGET_BLOCK = 60;
byte buffer[18];



//---------------------------------- Write data to TAG ------------------------------------


int writeBlock(int blockNumber, byte arrayAddress[]){
  //check if the block number corresponds to data block or triler block, rtuen with error if it's trailer block.
  int largestModulo4Number = blockNumber / 4 * 4;
  int trailerBlock = largestModulo4Number + 3; //determine trailer block for the sector
  if (blockNumber > 2 && (blockNumber + 1) % 4 == 0) {
    Serial.print(blockNumber);
    Serial.println(" is a trailer block: Error");
    return 2;
  }
  //authentication
  MFRC522::StatusCode status;

  status =(MFRC522::StatusCode) mfrc522.PCD_Authenticate(MFRC522::PICC_CMD_MF_AUTH_KEY_A, trailerBlock, &key, &(mfrc522.uid));
  if (status != MFRC522::STATUS_OK) {
    Serial.print("Authentication failed: ");
    Serial.println(mfrc522.GetStatusCodeName(status));
    return 3;//return "3" as error message
  }
  //writing data to the block
  status = mfrc522.MIFARE_Write(blockNumber, arrayAddress, 16);
  //status = mfrc522.MIFARE_Write(9, value1Block, 16);
  if (status != MFRC522::STATUS_OK) {
    Serial.print("Data write failed: ");
    Serial.println(mfrc522.GetStatusCodeName(status));
    return 4;//return "4" as error message
  }
}



// ---------------------------- Read data from TAG ----------------------------------- 

bool readCard(byte target_block, byte read_buffer[], byte length){                     // Buffer size must be 18 bytes
    if ( ! mfrc522.PICC_IsNewCardPresent()) { // Card present?
        return false;
    }
    if ( ! mfrc522.PICC_ReadCardSerial()) {  // read UID
   
        return false;
    }

    MFRC522::StatusCode status;

    // Authenticate using key A
    Serial.println(F("Authenticating using key A..."));
    status = (MFRC522::StatusCode) mfrc522.PCD_Authenticate(MFRC522::PICC_CMD_MF_AUTH_KEY_A,target_block, &key, &(mfrc522.uid));
    if (status != MFRC522::STATUS_OK) {
        Serial.print(F("PCD_Authenticate() failed: "));
        Serial.println(mfrc522.GetStatusCodeName(status));
        return false;
    }

    //byte length = 18;
    status = (MFRC522::StatusCode)mfrc522.MIFARE_Read(target_block, read_buffer, &length);

    if (status != MFRC522::STATUS_OK) {
        Serial.print(F("Tag read Failed: "));
        Serial.println(mfrc522.GetStatusCodeName(status));
        return false;
    }
    // Halt PICC
    mfrc522.PICC_HaltA();
    // Stop encryption on PCD
    mfrc522.PCD_StopCrypto1();
    return true;
}

void loop() {
    // Look for new cards if not found rerun the loop function
  uint32_t distance = Dist.getDistanceCentimeter();
  Serial.print("\nDistance in centimers: ");
  Serial.print(distance);  
  delay(500); //make it readable
}

