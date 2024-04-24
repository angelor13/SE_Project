#include <Arduino.h>
#include <SPI.h>


#define VSPI_MISO 33
#define VSPI_MOSi 25
#define VSPI_SCLK 26
#define VSPI_SS 27
#define BUFFER_SIZE 10




uint8_t Rxbuffer[4];
uint8_t Txbuffer[4];

void Task1code(void * parameter);
void Task2code(void * parameter);


typedef union{
	uint8_t buff[4];
	int32_t value;
}packet_buffer_t;

packet_buffer_t packet;
TaskHandle_t Task1,Task2;




void setup() {
  pinMode(VSPI_SS,OUTPUT);
  pinMode(VSPI_MISO,OUTPUT);
  pinMode(VSPI_MOSi,OUTPUT);
  pinMode(VSPI_SCLK,OUTPUT);

  SPI.begin(VSPI_SCLK,VSPI_MISO,VSPI_MOSi);
  SPI.setBitOrder(MSBFIRST);
  SPI.setDataMode(SPI_MODE0);


  Serial.begin(115200);
  Serial.setDebugOutput(true);


    xTaskCreatePinnedToCore(
      Task1code, /* Function to implement the task */
      "Task1",   /* Name of the task */
      10000,     /* Stack size in words */
      NULL,      /* Task input parameter */
      0,         /* Priority of the task */
      &Task1,    /* Task handle. */
      0);        /* Core where the task should run */

    xTaskCreatePinnedToCore(
      Task2code, /* Function to implement the task */
      "Task2",   /* Name of the task */
      10000,     /* Stack size in words */
      NULL,      /* Task input parameter */
      1,         /* Priority of the task */
      &Task2,    /* Task handle. */
      1);        /* Core where the task should run */

}


  

void loop() {

}







void Task1code(void *parameter)
{
  while(1)
  {
    //Serial.printf("CORE %d: hello world\n", xPortGetCoreID());
    delay(500);
  }

}



void Task2code(void *parameter){
while(1){
  //get RFID read

  delay(200);
}
}