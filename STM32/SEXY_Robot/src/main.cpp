#include <Arduino.h>

/* USER CODE BEGIN PD */
#define PWM_PERIOD 1000 // Adjust according to your motor specifications

// GPIO pins for motor direction control
#define MOTOR_A_IN1_Pin GPIO_PIN_0
#define MOTOR_A_IN1_GPIO_Port GPIOA
#define MOTOR_A_IN2_Pin GPIO_PIN_1
#define MOTOR_A_IN2_GPIO_Port GPIOA

// PWM pin for motor speed control
#define MOTOR_A_PWM_Pin GPIO_PIN_8
#define MOTOR_A_PWM_GPIO_Port GPIOB

// // Função para definir a direção do motor
// void Motor_Set_Direction(uint8_t dir) {
//   HAL_GPIO_WritePin(MOTOR_A_IN1_GPIO_Port, MOTOR_A_IN1_Pin, (dir & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET);
//   HAL_GPIO_WritePin(MOTOR_A_IN2_GPIO_Port, MOTOR_A_IN2_Pin, (dir & 0x02) ? GPIO_PIN_SET : GPIO_PIN_RESET);
// }

void Motor_Set_Speed(uint16_t speed) {

}


void setup() {
    Serial.begin(9600);
    Serial.println("Hello, world!");

    pinMode(PB1, OUTPUT); // AIN1

    pinMode(PA15, OUTPUT); // AIN1
    pinMode(PB7, OUTPUT); // AIN2
    pinMode(PB6, OUTPUT); // BIN1
    pinMode(PB5, OUTPUT); // BIN2

    pinMode(PB8, OUTPUT); // PWMA
    pinMode(PB9, OUTPUT); // PWMB
}

void loop() {
    digitalWrite(PB1, 0);

    digitalWrite(PA15, 1);
    digitalWrite(PA7, 1);

    digitalWrite(PB6, 1);
    digitalWrite(PB5, 1);

    digitalWrite(PB8, 1);
    digitalWrite(PB9, 1);

    Serial.println("Hello, world!");
}