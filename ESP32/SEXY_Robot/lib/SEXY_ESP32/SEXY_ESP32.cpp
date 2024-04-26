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
uint16_t SEXY_ESP32::GetFrontDistance() {
    uint16_t result = LidarFront.readRangeContinuousMillimeters();
    return constrain(result, DIST_LIDAR_MIN, DIST_LIDAR_MAX);
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
