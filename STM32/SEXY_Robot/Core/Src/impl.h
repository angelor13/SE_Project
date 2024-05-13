#ifndef IMPL_H
#define IMPL_H

#include <stdint.h>
#include <stdarg.h>
#include <stdio.h>
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "stm32f4xx_hal_spi.h"
#include "pid.h"

extern SPI_HandleTypeDef hspi1;
extern TIM_HandleTypeDef htim5;

int32_t delta_left = 0;
int32_t delta_right = 0;

int xprintf(const char* fmt, ...) {
    uint8_t rc = USBD_OK;
    char tmp[128];
	va_list ptr;

	va_start(ptr, fmt);
	int len = vsprintf(tmp, fmt, ptr);
    va_end(ptr);

    do {
        rc = CDC_Transmit_FS((uint8_t*)tmp, len);
    } while (USBD_BUSY == rc);

    if (USBD_FAIL == rc) {
        return 0;
    }

    return len;
}

int32_t getMotorDeltaLeft() {
	static int32_t last_pulse = 0;
	static int32_t last_tick = 0;

	int32_t current_pulse = htim5.Instance->CNT;
	int32_t current_tick = HAL_GetTick();

	int32_t delta_pulse = current_pulse - last_pulse;
	int32_t delta_tick = current_tick - last_tick;

	last_tick = current_tick;
	last_pulse = current_pulse;

//	xprintf("%d, %d\n\n", delta_pulse, delta_tick);

	delta_left = delta_pulse * 1000 / delta_tick;

	return delta_left;
}

int32_t getMotorDeltaRight() {
	static int32_t last_pulse = 0;
	static int32_t last_tick = 0;

	int32_t current_pulse = htim5.Instance->CNT;
	int32_t current_tick = HAL_GetTick();

	int32_t delta_pulse = current_pulse - last_pulse;
	int32_t delta_tick = current_tick - last_tick;

	last_tick = current_tick;
	last_pulse = current_pulse;

	delta_right = delta_pulse * 1000 / delta_tick;

	return delta_right;
}

void setMotorDeltaLeft(int32_t new_delta) {
	int32_t pid = 65535 - (int32_t)(PID_Control(20, 900, 0.1, new_delta, delta_left));

	if (pid > 65535) {
		pid = 65535;
	} else if (pid < -65535) {
		pid = -65535;
	}

	TIM2->CCR1 = abs(pid);

	if (pid > 0) {
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET); // 5, 6, 7, 15
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
	} else if (pid < 0) {
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET); // 5, 6, 7, 15
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
	} else {
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET); // 5, 6, 7, 15
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
	}

	xprintf("TM: %d\n", TIM2->CCR1);
}

#endif
