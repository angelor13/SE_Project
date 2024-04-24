#include <SPI.h>
#include <MFRC522.h> 
constexpr uint8_t RST_PIN = 9;     // Configurable, see typical pin layout above
constexpr uint8_t SS_PIN = 10;     // Configurable, see typical pin layout above
MFRC522 mfrc522(SS_PIN, RST_PIN);   // Create MFRC522 instance.
MFRC522::MIFARE_Key key = {.keyByte = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}};

#define N_SECTORS 15   //Sectors number

void setup() {
    Serial.begin(115200);         // Initialize serial com
    while (!Serial);              // Do nothing if no serial port is opened
    SPI.begin();                  // Init SPI bus
    mfrc522.PCD_Init();           // Init MFRC522 card
}

byte TARGET_BLOCK = 60;
byte buffer[18];

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
    byte comp=sizeof(buffer);
    if(readCard(TARGET_BLOCK, buffer, comp)){
        for(int i=0; i < sizeof(buffer); i++){
            Serial.print(buffer[i]);
            Serial.print(" ");
        }
        Serial.println("");
    }
}

