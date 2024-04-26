#include <SPI.h>
#include <MFRC522.h> 
// RFID PINOUT
static constexpr uint8_t RST_PIN = INT8_MAX;   
static constexpr uint8_t PIN_RFID_SDA=5;
static constexpr uint8_t PIN_RFID_SCK=18;
static constexpr uint8_t PIN_RFID_MISO=19;
static constexpr uint8_t PIN_RFID_MOSI=23;
MFRC522 mfrc522(PIN_RFID_SDA, RST_PIN);
MFRC522::MIFARE_Key key = {.keyByte = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}};
byte data[12] = {"Embebed-Sys"};

#define N_SECTORS 15   //Sectors number

void setup() {
    Serial.begin(115200);         // Initialize serial com
    while (!Serial);              // Do nothing if no serial port is opened
    SPI.begin();                  // Init SPI bus
    mfrc522.PCD_Init();           // Init MFRC522 card
}

static byte TARGET_BLOCK = 60;
byte buffer[18];

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


bool Tag_Detected(){
    if ( ! mfrc522.PICC_IsNewCardPresent()) {
    return false;
  }
    if ( ! mfrc522.PICC_ReadCardSerial()){
    return false;
  }
    return true;
}

void loop() {
        // Look for new cards if not found rerun the loop function
//   if ( ! mfrc522.PICC_IsNewCardPresent()) {
//     return;
//   }
//   // read from the card if not found rerun the loop function
//   if ( ! mfrc522.PICC_ReadCardSerial())
//   {
//     return;
//   }
//   Serial.println("card detected. Writing data");
//   writeBlock(1,data); //write data1 to the block 1 of the tag
//   Serial.println("reading data from the tag");
//   readCard(1, buffer,sizeof(buffer));   //read block 1
//   //print data
//   Serial.print("read block 1: ");
//   for (int j = 0 ; j < 14 ; j++)
//   {
//     Serial.print(buffer[j]);
//   }


    if(readCard(TARGET_BLOCK, buffer, sizeof(buffer))){
        for(int i=0; i < sizeof(buffer); i++){
            Serial.print(buffer[i]);
            Serial.print(" ");
        }
        Serial.println("");
    }
}

