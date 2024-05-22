#ifndef IMPL_H
#define IMPL_H

#include <stdint.h>
#include <stdarg.h>
#include <stdio.h>
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "stm32f4xx_hal_spi.h"
#include "xprintf.h"
//#include "pid.h"

extern SPI_HandleTypeDef hspi1;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim5;

int32_t delta_left = 0;
int32_t delta_right = 0;

#define PID_OUTPUT_MAX 65535
#define ERROR_INTEGRAL_MAX 10000

float constrain(float x, float min, float max) {
	if (x < min) {
		return min;
	} else if (x > max) {
		return max;
	} else {
		return x;
	}
}

float mapf(float x, float a, float b, float c, float d) {
	return (x - a) * (d - c) / (b - a) + c;
}

float lerpf(float a, float b, float t) {
    return (b - a) * t;
}

static inline float GetMicros() {
    uint32_t ms;
    uint32_t st;

    do {
        ms = HAL_GetTick();
        st = SysTick->VAL;
        asm volatile("nop");
        asm volatile("nop");
    } while (ms != HAL_GetTick());

    return (float) (ms * 1000 - st / ((SysTick->LOAD + 1) / 1000));
}

float PID_Control_L(float Kp, float Ki, float Kd, float setpoint, float value) {
    static float error_integral = 0.0;
    static float error_previous = 0.0;
    static float time_current = 0.0;
    static float time_previous = 0.0;

    // Calculate time difference
    time_current = GetMicros() / 1e6;
    double dt = time_current - time_previous;
    time_previous = time_current;

    // Calculate error
    float error = setpoint - value;
//    xprintf("X:%d, %d\n", (int32_t)(error * 1e6), (int32_t)value);

    // Proportional term
    float P = Kp * error;

    // Integral term
    error_integral = error_integral + error;
    float I = constrain(Ki * error_integral * dt, -ERROR_INTEGRAL_MAX, ERROR_INTEGRAL_MAX);

    // Derivative term
    float D = Kd * (error - error_previous) / dt;
    error_previous = error;

    // PID Output
    float output = constrain(P + I + D, -PID_OUTPUT_MAX, PID_OUTPUT_MAX);

    return output;
}


float PID_Control_R(float Kp, float Ki, float Kd, float setpoint, float value) {
    static float error_integral = 0.0;
    static float error_previous = 0.0;
    static float time_current = 0.0;
    static float time_previous = 0.0;

    // Calculate time difference
    time_current = GetMicros() / 1e6;
    double dt = time_current - time_previous;
    time_previous = time_current;

    // Calculate error
    float error = setpoint - value;
//    xprintf("X:%d, %d\n", (int32_t)(error * 1e6), (int32_t)value);

    // Proportional term
    float P = Kp * error;

    // Integral term
    error_integral = error_integral + error;
    float I = constrain(Ki * error_integral * dt, -ERROR_INTEGRAL_MAX, ERROR_INTEGRAL_MAX);

    // Derivative term
    float D = Kd * (error - error_previous) / dt;
    error_previous = error;

    // PID Output
    float output = constrain(P + I + D, -PID_OUTPUT_MAX, PID_OUTPUT_MAX);

    return output;
}


int32_t getMotorDeltaLeft() {
	static int32_t last_pulse = 0;
	static int32_t last_tick = 0;

	int32_t current_pulse = htim1.Instance->CNT;
	int32_t current_tick = HAL_GetTick();

	int32_t delta_pulse = current_pulse - last_pulse;
	int32_t delta_tick = current_tick - last_tick;

	if (delta_tick > 5) {
		delta_left = delta_pulse * 1000 / delta_tick;
		last_tick = current_tick;
		last_pulse = current_pulse;
	}

	return delta_left;
}

int32_t getMotorDeltaRight() {
	static int32_t last_pulse = 0;
	static int32_t last_tick = 0;

	int32_t current_pulse = htim5.Instance->CNT;
	int32_t current_tick = HAL_GetTick();

	int32_t delta_pulse = current_pulse - last_pulse;
	int32_t delta_tick = current_tick - last_tick;

	if (delta_tick > 5) {
		delta_right = delta_pulse * 1000 / delta_tick;
		last_tick = current_tick;
		last_pulse = current_pulse;
	}

	return delta_right;
}

void setMotorDeltaLeft(int32_t target) {
	static float offset = 0;
	static uint32_t last_tick = 0;
	float pid = PID_Control_L(20, 10, 0, target, delta_left);
	uint32_t tick = GetMicros();

	offset = constrain(offset, -65535, 65535);

	xprintf("PWM:%d\nORIGIN:%d\nTARGET:%d\n", (int32_t) pid, (int32_t) delta_left, (int32_t) target);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);

	if (target < 0) {
		if (delta_left == 0 && tick - last_tick > 500) {
			last_tick = tick;
			offset-=10;
		}

		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
//		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET); // 5, 6, 7, 15
		TIM2->CCR1 = abs(offset + pid);
	} else if (target > 0) {
		if (delta_left == 0 && tick - last_tick > 500) {
			last_tick = tick;
			offset+=10;
		}
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET); // 5, 6, 7, 15
		TIM2->CCR1 = PID_OUTPUT_MAX - abs(offset + pid);
	} else {
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
//		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET); // 5, 6, 7, 15
		TIM2->CCR1 = 65535;
	}
//	xprintf("TM: %d\n", pid);
}


void setMotorDeltaRight(int32_t target) {
	static float offset = 0;
	static uint32_t last_tick = 0;
	float pid = PID_Control_R(20, 10, 0, target, delta_right);
	uint32_t tick = GetMicros();

	offset = constrain(offset, -65535, 65535);

	xprintf("PWM:%d\nORIGIN:%d\nTARGET:%d\n", (int32_t) pid, (int32_t) delta_right, (int32_t) target);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);

	if (target < 0) {
		if (delta_left == 0 && tick - last_tick > 500) {
			last_tick = tick;
			offset-=10;
		}

//		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
//		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET); // 5, 6, 7, 15
		TIM2->CCR1 = abs(offset + pid);
	} else if (target > 0) {
		if (delta_left == 0 && tick - last_tick > 500) {
			last_tick = tick;
			offset+=10;
		}
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, gpio_pin_set);
		hal_gpio_writepin(gpiob, gpio_pin_6, gpio_pin_reset); // 5, 6, 7, 15
		TIM2->CCR1 = PID_OUTPUT_MAX - abs(offset + pid);
	} else {
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET); // 5, 6, 7, 15
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
		TIM2->CCR1 = 65535;
	}
//	xprintf("TM: %d\n", pid);
}

#endif
