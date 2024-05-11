#include <SEXY_ESP32.h>
#include "vec2.hpp"

SEXY_ESP32 Bot;


/// @brief Get motor velocities measured from encoders in [#PULSES] / [#MILLISECONDS]. Negative values mean inverted direction.
/// @return vec2(LEFT_VELOCITY, RIGHT_VELOCITY)
vec2 getMotorDeltas() {
    int32_t rxdata[2];

    digitalWrite(5, 0);
    SPI.transfer(0xAB);
    SPI.transfer(0xCD);
    SPI.transfer(rxdata, sizeof(rxdata));
    digitalWrite(5, 1);

    float left_velocity = (rxdata[0] / 1.0f);
    float right_velocity = (rxdata[1] / 1.0f);
    
    return vec2(left_velocity, right_velocity);
}

/// @brief Set motor velocities in [#PULSES] / [#MILLISECONDS]. Negative values mean inverted direction. [TO BE IMPLEMENTED ON STM32]
void setMotorDeltas(float left_velocity, float right_velocity) {
    int32_t txdata[2] = { (int32_t) left_velocity, (int32_t) right_velocity };

    digitalWrite(5, 0);
    SPI.transfer(0xDE);
    SPI.transfer(0xAD);
    SPI.transfer(txdata, sizeof(txdata));
    digitalWrite(5, 1);
}

void setup() {
    Serial.begin(115200);
    Bot.begin();  
}

void loop() {
    vec2 deltas = getMotorDeltas();

    Serial.printf("Left: %.5f, Right: %.5f\n", deltas.x, deltas.y);

    delay(100);

}

