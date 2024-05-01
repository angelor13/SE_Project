#include "SEXY_ESP32.h"

// Static defines
bool SEXY_ESP32::isTagDetected=false;

TaskHandle_t SEXY_ESP32::taskReadRFIDHandle;
TaskHandle_t SEXY_ESP32::taskTransmitSPiComHandle;


MFRC522 SEXY_ESP32 :: RFID_device (PIN_RFID_SDA,RST_PIN);

VL53L0X SEXY_ESP32::LidarFront;


byte SEXY_ESP32::RxBuffer[8];
byte SEXY_ESP32::TxBuffer[8];

float SEXY_ESP32::L,r,dotphiL=0,dotphiR=0,vx=0,w=0;



// Implementation
void SEXY_ESP32::setupMotors() {
    pinMode(PIN_MOTOR_L_1, OUTPUT);
    pinMode(PIN_MOTOR_L_2, OUTPUT);
    pinMode(PIN_MOTOR_R_1, OUTPUT);
    pinMode(PIN_MOTOR_R_2, OUTPUT);

    analogWriteResolution(PWM_RESOLUTION_BITS);
}
/**
 * @brief Initialize the Lidar front
 */
void SEXY_ESP32::setupLidar() {
    Wire.begin();

    // LiDAR
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


void SEXY_ESP32::setupSharps(){
    // Sharps 
    pinMode(PIN_VP_LEFT,INPUT);
    pinMode(PIN_VP_RIGHT,INPUT);
}

void SEXY_ESP32::setupSPI(){
  SPI.begin(VSPI_SCLK,VSPI_MISO,VSPI_MOSi);
  SPI.setBitOrder(MSBFIRST);
  SPI.setDataMode(SPI_MODE0);
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
    setupSharps();
    setupLidar();
    setupRFID();
    Serial.begin(115200);
    setupSPI();
    xTaskCreatePinnedToCore(taskReadRFID, "TASK_RFID", 2000, nullptr, 1, &taskReadRFIDHandle, 0);
    xTaskCreatePinnedToCore(taskTransmitSPICom, "TASK_SPI_COM", 2000, nullptr, 1, &taskTransmitSPiComHandle, 1);
}

/**
  @brief Control left motor speed.
  @param duty desired duty cycle for the motor, value between [-511, 511]
 */
void SEXY_ESP32::moveMotorLeft(int16_t duty) {
    duty = constrain(duty , -DUTY_MOTOR_MAX, DUTY_MOTOR_MAX);

    if (duty <= 0) {
        digitalWrite(PIN_MOTOR_L_1, LOW);
    } else {
        duty = DUTY_MOTOR_MAX - duty;
        digitalWrite(PIN_MOTOR_L_1, HIGH);
    }

    analogWrite(PIN_MOTOR_L_2, abs(duty));
}

/**
  @brief Control right motor speed.
  @param duty desired duty cycle for the motor, value between [-511, 511]
 */
void SEXY_ESP32 :: moveMotorRight(int16_t duty) {
    duty = constrain(duty, -DUTY_MOTOR_MAX, DUTY_MOTOR_MAX);

    if (duty <= 0) {
        digitalWrite(PIN_MOTOR_R_1, LOW);
    } else {
        duty = DUTY_MOTOR_MAX - duty;
        digitalWrite(PIN_MOTOR_R_1, HIGH);
    }

    analogWrite(PIN_MOTOR_R_2, abs(duty));
}

/**
  @brief Control both motors simultaneously.
  @param dutyMotorLeft desired duty cycle for left motor, value between [-511, 511]
  @param dutyMotorRight desired duty cycle for right motor, value between [-511, 511]
 */
void SEXY_ESP32::moveMotors(int16_t dutyMotorLeft, int16_t dutyMotorRight) {
    moveMotorLeft(dutyMotorLeft);
    moveMotorRight(dutyMotorRight);
}

/**
  @brief Stop Motors
 */
void SEXY_ESP32::stopMotors(){
  moveMotorLeft(0);
  moveMotorRight(0);
}
/**
 * @brief Get the left distance value, in millimeters.
 * @return value between [100, 800] (mm)
 */
uint16_t SEXY_ESP32::getLeftDistance(){
  uint16_t value=analogRead(PIN_VP_LEFT);
  return constrain(map(value,0,4095,100,800),100,800);
}
/**
 * @brief Get the front LiDAR distance value, in millimeters.
 * @return value between [0, 2600] 
 */
uint16_t SEXY_ESP32::getFrontDistance() {
    uint16_t distance = LidarFront.readRangeContinuousMillimeters();
    return constrain(distance, DIST_LIDAR_MIN, DIST_LIDAR_MAX);
}

/**
 * @brief Get the left distance value, in millimeters.
 * @return value between [100, 800] (mm)
 */
uint16_t SEXY_ESP32::getRightDistance(){
  uint16_t value=analogRead(PIN_VP_RIGHT);
  return constrain(map(value,0,4095,100,800),100,800);
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
    Serial.println("Tag detected");
    return true;
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

void SEXY_ESP32::taskTransmitSPICom(void*){
  while(1){
    digitalWrite(VSPI_SS, LOW);
    SPI.transferBytes(TxBuffer,RxBuffer,BUFFER_SIZE);
    digitalWrite(VSPI_SS, HIGH);
    delay(10);
  }
}

void SEXY_ESP32:: taskReadRFID(void*){
  while(1){
    isTagDetected=Tag_Detected();
    delay(25);
  }
}


bool SEXY_ESP32 ::getTagDetected(){
  return isTagDetected;
}

/**
  @brief Calculate dot phiL
 */
float SEXY_ESP32::calculatedDotphiL(const float vx,const float w, const float L,const float r){
  return (vx - (L/2)*w)/r;
}

/**
  @brief Calculate dot phiR
 */
float SEXY_ESP32::calculatedDotphiR(const float vx,const float w, const float L,const float r){
  return (vx + (L/2)*w)/r;
}

/**
  @brief Calculate V_x
 */
float SEXY_ESP32::calculatedVx(const float dotphiR,const float dotphiL,const float r){
  return (dotphiR+dotphiL)*r/2;
}


/**
  @brief Calculate W
 */
float SEXY_ESP32::calculatedW(const float dotphiR,const float dotphiL,const float L,const float r){
  return (dotphiR-dotphiL)*r/L;
}

/**
  @brief Get dot phiL
 */
float SEXY_ESP32::getDotphiL(){
  return dotphiL;
}
/**
  @brief Get dot phiR
 */
float SEXY_ESP32::getDotphiR(){
  return dotphiR;
}
/**
  @brief Get Vx
 */
float SEXY_ESP32::getVx(){
  return vx;
}
/**
  @brief Get W
 */
float SEXY_ESP32::getW(){
  return dotphiL;
}








