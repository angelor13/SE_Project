#include "SEXY_ESP32.h"

// Static defines

// TaskHandle_t FCTUC::batteryTaskHandle;
// TaskHandle_t FCTUC::rfidTaskHandle;
// TaskHandle_t FCTUC::udpTaskHandle;
// TaskHandle_t FCTUC::mainTaskHandle;


MFRC522 SEXY_ESP32 :: RFID_device (PIN_RFID_SDA,RST_PIN);

VL53L0X SEXY_ESP32::LidarFront;



// Implementation


// void SEXY_ESP32::setupMotors() {
//     pinMode(PIN_MOTOR_L_1, OUTPUT);
//     pinMode(PIN_MOTOR_L_2, OUTPUT);
//     pinMode(PIN_MOTOR_R_1, OUTPUT);
//     pinMode(PIN_MOTOR_R_2, OUTPUT);

//     analogWriteResolution(PWM_RESOLUTION_BITS);
// }
/**
 * @brief Initialize the Lidar front
 */
void SEXY_ESP32::setupLidar() {
    Wire.begin();

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




/**
 * @brief Initialize the RFID.
 */

void SEXY_ESP32::setupRFID() {
  SPI.begin(); 
  RFID_device.PCD_Init();
}

 

/**
 * @brief Initialize the hardware interface. Must be called to interact with robot.
 */
void SEXY_ESP32::begin() {
    //setupMotors();
    setupLidar();
    setupRFID();
    Serial.begin(115200);
    
}








/**
  @brief Control left motor speed.
  @param duty desired duty cycle for the motor, value between [-511, 511]
 */
// void FCTUC::moveMotorLeft(int16_t duty) {
//     duty = constrain(duty * motorCoefficientLeft, -DUTY_MOTOR_MAX, DUTY_MOTOR_MAX);

//     if (duty <= 0) {
//         digitalWrite(PIN_MOTOR_L_1, LOW);
//     } else {
//         duty = DUTY_MOTOR_MAX - duty;
//         digitalWrite(PIN_MOTOR_L_1, HIGH);
//     }

//     analogWrite(PIN_MOTOR_L_2, abs(duty));
// }

/**
  @brief Control right motor speed.
  @param duty desired duty cycle for the motor, value between [-511, 511]
 */
// void FCTUC::moveMotorRight(int16_t duty) {
//     duty = constrain(duty * motorCoefficientRight, -DUTY_MOTOR_MAX, DUTY_MOTOR_MAX);

//     if (duty <= 0) {
//         digitalWrite(PIN_MOTOR_R_1, LOW);
//     } else {
//         duty = DUTY_MOTOR_MAX - duty;
//         digitalWrite(PIN_MOTOR_R_1, HIGH);
//     }

//     analogWrite(PIN_MOTOR_R_2, abs(duty));
// }

/**
  @brief Control both motors simultaneously.
  @param dutyMotorLeft desired duty cycle for left motor, value between [-511, 511]
  @param dutyMotorRight desired duty cycle for right motor, value between [-511, 511]
 */
// void FCTUC::moveMotors(int16_t dutyMotorLeft, int16_t dutyMotorRight) {
//     moveMotorLeft(dutyMotorLeft);
//     moveMotorRight(dutyMotorRight);
// }

/**
 * @brief Get the right LiDAR distance value, in millimeters.
 * @return value between [0, 2600] (mm)
 */

/**
 * @brief Get the front LiDAR distance value, in millimeters.
 * @return value between [0, 2600] 
 */
uint16_t SEXY_ESP32::getFrontDistance() {
    uint16_t result = LidarFront.readRangeContinuousMillimeters();
    return constrain(result, DIST_LIDAR_MIN, DIST_LIDAR_MAX);
}
/**
  @brief Read RFID Tags
 */
bool SEXY_ESP32 ::readCard(byte target_block, byte read_buffer[], byte length){                     // Buffer size must be 18 bytes
    if ( ! RFID_device.PICC_IsNewCardPresent()) { // Card present?
        return false;
    }
    if ( ! RFID_device.PICC_ReadCardSerial()) {  // read UID
   
        return false;
    }
  static MFRC522::MIFARE_Key key_tag={.keyByte = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}};
    MFRC522::StatusCode status;

    // Authenticate using key A
    Serial.println(F("Authenticating using key A..."));
    status = (MFRC522::StatusCode) RFID_device.PCD_Authenticate(MFRC522::PICC_CMD_MF_AUTH_KEY_A,target_block, &key_tag, &(RFID_device.uid));
    if (status != MFRC522::STATUS_OK) {
        Serial.print(F("PCD_Authenticate() failed: "));
        Serial.println(RFID_device.GetStatusCodeName(status));
        return false;
    }

    //byte length = 18;
    status = (MFRC522::StatusCode)RFID_device.MIFARE_Read(target_block, read_buffer, &length);

    if (status != MFRC522::STATUS_OK) {
        Serial.print(F("Tag read Failed: "));
        Serial.println(RFID_device.GetStatusCodeName(status));
        return false;
    }
    // Halt PICC
    RFID_device.PICC_HaltA();
    // Stop encryption on PCD
    RFID_device.PCD_StopCrypto1();
    return true;
}
/**
  @brief Write in RFID Tags
 */
int SEXY_ESP32 ::writeBlock(int blockNumber, byte arrayAddress[]){
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
  static MFRC522::MIFARE_Key key_tag={.keyByte = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}};
  status =(MFRC522::StatusCode) RFID_device.PCD_Authenticate(MFRC522::PICC_CMD_MF_AUTH_KEY_A, trailerBlock, &key_tag, &(RFID_device.uid));
  if (status != MFRC522::STATUS_OK) {
    Serial.print("Authentication failed: ");
    Serial.println(RFID_device.GetStatusCodeName(status));
    return 3;//return "3" as error message
  }
  //writing data to the block
  status = RFID_device.MIFARE_Write(blockNumber, arrayAddress, 16);
  //status = mfrc522.MIFARE_Write(9, value1Block, 16);
  if (status != MFRC522::STATUS_OK) {
    Serial.print("Data write failed: ");
    Serial.println(RFID_device.GetStatusCodeName(status));
    return 4;//return "4" as error message
  }
}

/**
  @brief detect RFID Tags
 */
bool SEXY_ESP32:: Tag_Detected(){
    if ( ! RFID_device.PICC_IsNewCardPresent()) {
    return false;
  }
    if ( ! RFID_device.PICC_ReadCardSerial()){
    return false;
  }
    return true;
}
/**
  @brief Prints a string via serial and via WiFi
 */
void SEXY_ESP32::print(const char* str){
    Serial.print(str);

    // for(uint8_t i = 0; i < MAX_TCP_CLIENTS; i++){
    //     if(tcpClients[i] != nullptr){
    //         tcpClients[i]->add(str, strlen(str));
    //         tcpClients[i]->send();
    //     }
    // }
}

/**
  @brief Print all detected I2C devices.
 */
void SEXY_ESP32::printI2C() {
    Serial.println("I2C scanner. Scanning ...");
    byte count = 0;

    for (byte i = 1; i < 120; i++) {
        Wire.beginTransmission(i);

        if (Wire.endTransmission() == 0) {
            Serial.print("Found address: ");
            Serial.print(i, DEC);
            Serial.print(" (0x");
            Serial.print(i, HEX);
            Serial.println(")");
            count++;
            delay(1); // maybe unneeded?
        }
    }

    Serial.println("Done.");
    Serial.print("Found ");
    Serial.print(count, DEC);
    Serial.println(" device(s).");
}

/**
  @brief Print all LiDAR distance values.
//  */
// void FCTUC::printLidarValue() {
//     uint16_t left = getLidarLeftDistance();
//     uint16_t front = getLidarFrontDistance();
//     uint16_t right = getLidarRightDistance();

//     println(
//         "Left : Front : Right - " + String(left) + " : " +  String(front)  + " : " + String(right) + " (mm)"
//     );
// }

/**
  @brief Print (to serial ONLY) the detected RFID reader firmware versions. Useful for detecting connection issues.
 */
