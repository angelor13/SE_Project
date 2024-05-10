#include <SEXY_ESP32.h>

SEXY_ESP32 Bot;

void setup() {
    Serial.begin(115200);
    Bot.begin();  
}

void loop() {
    uint8_t rxdata[64] = "Hello, world!";
    uint8_t txdata[64] = { 0 };

    digitalWrite(5, 0);

    SPI.transfer(0xAB);
    SPI.transfer(0xCD);
    SPI.transfer(rxdata, sizeof(rxdata));

    do {
        rxdata[0] = SPI.transfer(0x00);
        rxdata[1] = SPI.transfer(0x00);
    } while (rxdata[0] != 0xAB && rxdata[1] != 0xCD);

    SPI.transfer(txdata, 15);

    Serial.printf("RX: %.15s\n", txdata);

    digitalWrite(5, 1);

    delay(100);

}

