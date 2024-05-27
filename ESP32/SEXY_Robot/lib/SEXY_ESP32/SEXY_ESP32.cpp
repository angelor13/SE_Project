#include "SEXY_ESP32.h"



// Static defines
bool SEXY_ESP32::isTagDetected=false;

TaskHandle_t SEXY_ESP32::taskReadRFIDHandle;
TaskHandle_t SEXY_ESP32::taskReceiveSPiComHandle;
TaskHandle_t SEXY_ESP32::taskGetPointCloudHandle;
// TaskHandle_t SEXY_ESP32::taskServerHandle;
TaskHandle_t SEXY_ESP32::taskGetworkStateHandle;

std::vector<vec2> SEXY_ESP32::mapPointCloud;

char* SEXY_ESP32::ssid = "Coiso";
char* SEXY_ESP32::password = "iimartinsb85";


MFRC522 SEXY_ESP32 :: RFID_device (PIN_RFID_SDA,RST_PIN);
WiFiServer SEXY_ESP32:: server(80);


VL53L0X SEXY_ESP32::LidarFront;
Adafruit_ADS1115 SEXY_ESP32::gasADC;
//WiFiUDP SEXY_ESP32::wifi;

byte SEXY_ESP32::RxBuffer[4];
byte SEXY_ESP32::TxBuffer[4];

float SEXY_ESP32::R=0.2;
float SEXY_ESP32::L=0.1605;
float SEXY_ESP32::r=0.0455/2;
float SEXY_ESP32::dotphiL;
float SEXY_ESP32::dotphiR;
float SEXY_ESP32::vx=MAX_Vx*r;
float SEXY_ESP32::w=vx/R;
float SEXY_ESP32::PercentL=0,PercentR=0;
bool SEXY_ESP32::rotating=false;
uint8_t SEXY_ESP32::currentDirection=FRONT;
uint32_t SEXY_ESP32::align=0;

String SEXY_ESP32::current_mode="off";
SEXY_ESP32::SEXY_POS SEXY_ESP32::robot_pos;


float SEXY_ESP32::distanceMotorL=0;
float SEXY_ESP32::distanceMotorR=0;

long SEXY_ESP32::previous_millis=0;
long SEXY_ESP32::previous_distanceMotorL=0;
long SEXY_ESP32::previous_distanceMotorR=0;


bool SEXY_ESP32::enable_send=true;
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
	Wire.begin(PIN_SDA_FRONT,PIN_SCL_FRONT);

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
/**
 * @brief Initialize Wifi
 */


/**
 * @brief Initialize the Lidar front
 */
void SEXY_ESP32::setupSharps(){
	// Sharps
	pinMode(PIN_VP_LEFT,INPUT);
	pinMode(PIN_VP_RIGHT,INPUT);
}
/**
 * @brief Initialize the SPI
 */
void SEXY_ESP32::setupSPI(){
	pinMode(VSPI_SS,OUTPUT);
  SPI.begin(VSPI_SCLK,VSPI_MISO,VSPI_MOSi,VSPI_SS);
  SPI.setBitOrder(MSBFIRST);
  SPI.setDataMode(SPI_MODE0);
  SPI.setFrequency(10000);
}

void SEXY_ESP32::setupWifi(){
	pinMode(output26, OUTPUT);
  // Set outputs to LOW
  digitalWrite(output26, LOW);
	WiFi.begin(ssid, password);
	while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  // Print local IP address and start web server
  Serial.println("");
  Serial.println("WiFi connected.");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

/**
 * @brief Initialize the ADC
 */
// void SEXY_ESP32::setupADC(){
// gasADC.begin(ADDR_ADC);
// }
// /**
//  * @brief Initialize the RFID.
//  */
// void SEXY_ESP32::setupRFID() {
//   //SPI.begin();
//   RFID_device.PCD_Init();
// }

 /**
 * @brief Initialize the hardware interface. Must be called to interact with robot.
 */
void SEXY_ESP32::begin() {
	//setupADC();
	setupSharps();
	setupLidar();
	//setupWifi();
	//setupRFID();
	setupSPI();
	//xTaskCreatePinnedToCore(taskReadRFID, "TASK_RFID", 2000, nullptr, 2, &taskReadRFIDHandle,0);
	xTaskCreatePinnedToCore(taskReceiveSPICom, "TASK_SPI_COM", 2000, nullptr, 1, &taskReceiveSPiComHandle, 0);
	//xTaskCreatePinnedToCore(taskGetPointCloud, "TASK_SLAM_POINTS", 2000, nullptr, 1, &taskGetPointCloudHandle, 0);
	// xTaskCreatePinnedToCore(taskServer, "TASK_Server", 2000, nullptr, 1, &taskServerHandle, 0);
	//xTaskCreatePinnedToCore(taskGetworkState, "TASK_taskGetworkStateHandle", 2000, nullptr, 1, &taskGetworkStateHandle, 0);
}
/**
  @brief Control left motor speed.
  @param perL desired velocity for the motor, value between [-511, 511]
 */
void SEXY_ESP32::moveMotorLeft(int16_t perL) {
	perL = constrain(perL , -MAXPERCENT, MAXPERCENT);
	setMotorVelocity(perL*MAX_Vx,0);

}

/**
  @brief Control right motor speed.
  @param perR desired velocity for the motor, value between [-511, 511]
 */
void SEXY_ESP32 :: moveMotorRight(int16_t perR) {
	perR = constrain(perR , -MAXPERCENT, MAXPERCENT);
	setMotorVelocity(0,perR*MAX_Vx);
}

/**
  @brief Control both motors simultaneously.
  @param perL desired duty cycle for left motor, value between [-100 100]
  @param perR desired duty cycle for right motor, value between [-100,100]
 */
void SEXY_ESP32::moveMotors(int16_t perL, int16_t perR) {
	setMotorVelocity(perL*MAX_Vx,perR*MAX_Vx);
}

/**
  @brief Stop Motors
 */
void SEXY_ESP32::stopMotors(){
	moveMotors(0,0);
}
/**
 * @brief Get the left distance value, in millimeters.
 * @return value between [100, 800] (mm)
 */
uint16_t SEXY_ESP32::getLeftDistance(){
  uint16_t value=analogRead(PIN_VP_LEFT);
  return constrain(map(value,0,4095,800,100),100,800);
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
  return constrain(map(value,0,4095,800,100),100,800);
}

uint16_t SEXY_ESP32::getADCvalue(){
  uint16_t value=gasADC.readADC_SingleEnded(0);
  return value;
}

/**
  @brief Read RFID Tags
*/

bool SEXY_ESP32::readCard(byte target_block, byte read_buffer[], byte length){                     // Buffer size must be 18 bytes
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
int SEXY_ESP32::writeBlock(int blockNumber, byte arrayAddress[]){
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
  return -1;
}

/**
  @brief detect RFID Tags
 */
bool SEXY_ESP32::Tag_Detected(){
	if ( ! RFID_device.PICC_IsNewCardPresent()) {
	return false;
  }
	if ( ! RFID_device.PICC_ReadCardSerial()){
	  //Serial.println("Funcao2");
	return false;
  }

//   WiFiClient client = server.available(); 
//   if (client) {                             // if you get a client,
//     Serial.println("New Client.");           // print a message out the serial port
//     String currentLine = "";                // make a String to hold incoming data from the client
//     while (client.connected()) {            // loop while the client's connected
//       if (client.available()) {             // if there's bytes to read from the client,
//         char c = client.read();             // read a byte, then
//         Serial.write(c);                    // print it out the serial monitor
//         if (c == '\n') {                    // if the byte is a newline character

//           // if the current line is blank, you got two newline characters in a row.
//           // that's the end of the client HTTP request, so send a response:
//           if (currentLine.length() == 0) {
//             // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
//             // and a content-type so the client knows what's coming, then a blank line:
//             client.println("HTTP/1.1 200 OK");
//             client.println("Content-type:text/html");
//             client.println();

//             // the content of the HTTP response follows the header:
//             client.print("Click <a href=\"/H\">here</a> to turn the LED on pin 5 on.<br>");
//             client.print("Click <a href=\"/L\">here</a> to turn the LED on pin 5 off.<br>");

            
//             client.printf("\n Send coordinates: ");
//             robot_pos.x,robot_pos.y,robot_pos.phi = client.read();
//             delay(500);

            
//             // The HTTP response ends with another blank line:
//             client.println();
//             // break out of the while loop:
//             break;
//           } else {    // if you got a newline, then clear currentLine:
//             currentLine = "";
//           }
//         } else if (c != '\r') {  // if you got anything else but a carriage return character,
//           currentLine += c;      // add it to the end of the currentLine
//         }

//         // Check to see if the client request was "GET /H" or "GET /L":
//         if (currentLine.endsWith("GET /H")) {
//           digitalWrite(15, HIGH);               // GET /H turns the LED on
//         }
//         if (currentLine.endsWith("GET /L")) {
//           digitalWrite(15, LOW);                // GET /L turns the LED off
//         }
//       }
//     }
//     // close the connection:
//     client.stop();
//     Serial.println("Client Disconnected.");
//     Serial.printf("Received coordinates: %d %d  %d", robot_pos.x,robot_pos.y,robot_pos.phi);
    
    
  //}
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

/// @brief Get motor velocities measured from encoders in [#PULSES] / [#MILLISECONDS]. Negative values mean inverted direction.
/// @return vec2(LEFT_VELOCITY, RIGHT_VELOCITY)
vec2 SEXY_ESP32::getMotorVelocity() {
    int32_t rxdata[2];
	float raio=r*100;
    const int pulse_n_per_rot=1470;

    digitalWrite(VSPI_SS, 0);
    SPI.write(0xAB);
    SPI.write(0xCD);
    SPI.transfer(rxdata, sizeof(rxdata));
    digitalWrite(VSPI_SS, 1);

    float left_velocity = (rxdata[0] / 1.0f);
    float right_velocity = (rxdata[1] / 1.0f);

	

    float wL = (left_velocity * 2 * PI ) / (pulse_n_per_rot);
	float wR = (right_velocity * 2 * PI ) / (pulse_n_per_rot);
    
    return vec2(wL, wR);
}

/// @brief Set motor velocities in [#PULSES] / [#MILLISECONDS]. Negative values mean inverted direction. [TO BE IMPLEMENTED ON STM32]
void SEXY_ESP32::setMotorVelocity(float left_velocity, float right_velocity) {
	float raio=r*100;
  const int pulse_n_per_rot=1470;
  float left_omega = left_velocity / raio;
	float righ_omega = right_velocity / raio;

  float dptL = constrain((left_omega * pulse_n_per_rot) / (2*PI),-65535,65535);
	float dptR = constrain((righ_omega * pulse_n_per_rot) / (2*PI),-65535,65535);

  int32_t txdata[2] = { (int32_t) dptL, (int32_t) dptR };
	Serial.println(txdata[0]);
	Serial.println(txdata[1]);

    digitalWrite(VSPI_SS, 0);
    SPI.write(0xDE);
    SPI.write(0xAD);
    SPI.transfer(txdata, sizeof(txdata));
    digitalWrite(VSPI_SS, 1);
}

// void SEXY_ESP32::taskServer(void*) {
//     Serial.begin(115200);

//     // Connect to Wi-Fi
//     WiFi.begin(ssid, password);
//     Serial.print("Connecting to WiFi");
//     while (WiFi.status() != WL_CONNECTED) {
//         delay(1000);
//         Serial.print(".");
//     }
//     Serial.println(" connected");

//     // Start the web server
//     server.on("/", handleRoot);
//     server.on("/style.css", handleCSS);
//     server.on("/control", handleCommand);
//     server.on("/mode", handleSwitchMode);
//     server.onNotFound(handleNotFound);

//     server.begin();
//     server.enableCrossOrigin();
//     Serial.println("HTTP server started");

//     // Print the IP address
//     Serial.println("IP address: ");
//     Serial.println(WiFi.localIP());

//     while (1) {
//         server.handleClient();
//     }
// }

void SEXY_ESP32::taskReceiveSPICom(void*){
  while(1){	
	vec2 buffer;
	//Serial.println("Data Transmition");
	//RFID_device.PCD_AntennaOff();
	long current_millis=millis();

	buffer=getMotorVelocity();

	//RFID_device.PCD_AntennaOn();

	dotphiL=(float)buffer.x;	
	dotphiR=(float)buffer.y;
	w=calculatedW(dotphiR,dotphiL,R,r);

	//Serial.println("Data Transmition");

	Serial.println("dotphiL:  "+(String)dotphiL);
	Serial.println("dotphiR:  "+(String)dotphiR);

	distanceMotorL+=2*PI*dotphiL*(current_millis-previous_millis)/1000;
	distanceMotorR+=2*PI*dotphiR*(current_millis-previous_millis)/1000;
	


	//--------------------- ODOMETRIA ------------------------------------------
		if(rotating){
			robot_pos.phi+=PI/2;
		}
		else{
		uint32_t delta_d=((distanceMotorL-previous_distanceMotorL)+(distanceMotorR-previous_distanceMotorR))/2;
		float raio=R;
		float delta_phi=w*(current_millis-previous_millis);	
		//float delta_phi = (delta_d*2*PI)/(2*PI*R);
		robot_pos.x += delta_d * cos(robot_pos.phi + delta_phi/2);
		robot_pos.y += delta_d * sin(robot_pos.phi + delta_phi/2);
		//robot_pos.phi+= delta_phi;
		
		robot_pos.phi+=delta_phi; 	// another way to program or maybe the best way
			// if(robot_pos.phi>=2*PI){
			// 	robot_pos.phi=robot_pos.phi % (2*PI);
			// }
		}

	previous_distanceMotorL=distanceMotorL;
	previous_distanceMotorR=distanceMotorR;

	previous_millis=current_millis;

	delay(10);
  }
}

void SEXY_ESP32:: taskGetworkState(void*){
String header;
unsigned long previousTime =0;
unsigned long currentTime = millis();
const long timeoutTime = 2000;
while(1){
WiFiClient client = server.available();   // Listen for incoming clients

  if (client) {                             // If a new client connects,
    currentTime = millis();
    previousTime = currentTime;
    Serial.println("New Client.");          // print a message out in the serial port
    String currentLine = "";                // make a String to hold incoming data from the client
    while (client.connected() && currentTime - previousTime <= timeoutTime) {  // loop while the client's connected
      currentTime = millis();
      if (client.available()) {             // if there's bytes to read from the client,
        char c = client.read();             // read a byte, then
        Serial.write(c);                    // print it out the serial monitor
        header += c;
        if (c == '\n') {                    // if the byte is a newline character
          // if the current line is blank, you got two newline characters in a row.
          // that's the end of the client HTTP request, so send a response:
          if (currentLine.length() == 0) {
            // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
            // and a content-type so the client knows what's coming, then a blank line:
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/html");
            client.println("Connection: close");
            client.println();
            
            // turns the GPIOs on and off
            if (header.indexOf("GET /26/on") >= 0) {
              Serial.println("GPIO 26 on");
              current_mode = "on";
              digitalWrite(output26, HIGH);
            } else if (header.indexOf("GET /26/off") >= 0) {
              Serial.println("GPIO 26 off");
              current_mode = "off";
              digitalWrite(output26, LOW);
            } 
            
            // Display the HTML web page
            client.println("<!DOCTYPE html><html>");
            client.println("<head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">");
            client.println("<link rel=\"icon\" href=\"data:,\">");
            // CSS to style the on/off buttons 
            // Feel free to change the background-color and font-size attributes to fit your preferences
            client.println("<style>html { font-family: Helvetica; display: inline-block; margin: 0px auto; text-align: center;}");
            client.println(".button { background-color: #4CAF50; border: none; color: white; padding: 16px 40px;");
            client.println("text-decoration: none; font-size: 30px; margin: 2px; cursor: pointer;}");
            client.println(".button2 {background-color: #555555;}</style></head>");
            
            // Web Page Heading
            client.println("<body><h1>Robot Start</h1>");
            
            // Display current state, and ON/OFF buttons for GPIO 26  
            client.println("<p>Start - State " + current_mode + "</p>");
            // If the output26State is off, it displays the ON button       
            if (current_mode=="off") {
              client.println("<p><a href=\"/26/on\"><button class=\"button\">ON</button></a></p>");
            } else {
              client.println("<p><a href=\"/26/off\"><button class=\"button button2\">OFF</button></a></p>");
            } 
               
            
            client.println("</body></html>");
            
            // The HTTP response ends with another blank line
            client.println();
            // Break out of the while loop
            break;
          } else { // if you got a newline, then clear currentLine
            currentLine = "";
          }
        } else if (c != '\r') {  // if you got anything else but a carriage return character,
          currentLine += c;      // add it to the end of the currentLine
        }
      }
    }
    // Clear the header variable
    header = "";
    Serial.println(current_mode);
    // Close the connection
    client.stop();
    Serial.println("Client disconnected.");
    Serial.println("");
  }
   delay(20);
}
 
}
// NOt to use


// void SEXY_ESP32::transmitSPIcom(){
// 	//RFID_device.PCD_AntennaOff();
// 	digitalWrite(VSPI_SS, LOW);
// 	SPI.transferBytes(TxBuffer,NULL,sizeof(TxBuffer));
// 	digitalWrite(VSPI_SS, HIGH);
// 	//RFID_device.PCD_AntennaOn();

// }


void SEXY_ESP32::taskGetPointCloud(void*){

  uint32_t leftDistance=getLeftDistance();
  uint32_t frontDistance=getFrontDistance();
  uint32_t rightDistance=getRightDistance();



  vec2 atual_pos= vec2(robot_pos.x,robot_pos.y);
  vec2 dir_left = vec2(cos(PI/4+robot_pos.phi)*leftDistance,sin(PI/4+robot_pos.phi)*leftDistance);
  vec2 dir_front = vec2(cos(0+robot_pos.phi)*frontDistance,sin(0+robot_pos.phi)*frontDistance);
  vec2 dir_right = vec2(cos(-PI/4+robot_pos.phi)*rightDistance,sin(-PI/4+robot_pos.phi)*rightDistance);

  // Dont need robot phi 
  dir_left+=atual_pos;
  dir_front+=atual_pos;
  dir_right+=atual_pos;

  mapPointCloud.push_back(dir_left);
  mapPointCloud.push_back(dir_front);
  mapPointCloud.push_back(dir_right);

  delay(10);
}





void SEXY_ESP32::taskReadRFID(void*){
  while(1){
	isTagDetected=Tag_Detected();
	delay(25);
  }
}

bool SEXY_ESP32::getTagDetected(){
  return isTagDetected;
}

/**
  @brief Calculate dot phiL  | Maybe unused
 */
float SEXY_ESP32::calculatedDotphiL(const float vx,const float w, const float L,const float r){
  return (vx - (L/2)*w)/r;
}

/**
  @brief Calculate dot phiR  | Maybe unused
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
  vx=(float)calculatedVx(dotphiR,dotphiL,r);
  return vx;
}
/**
  @brief Get W
 */
float SEXY_ESP32::getW(){
  w=(float)calculatedW(dotphiR,dotphiL,L,r);
  return w;
}

// float SEXY_ESP32::getR(float Raio){
//   R=Raio;
//   return R;
// }


float SEXY_ESP32::getDistanceL(){
  return distanceMotorL;
}

float SEXY_ESP32::getDistanceR(){
  return distanceMotorR;
}


bool SEXY_ESP32::getEnableSend(){
  return enable_send;
}