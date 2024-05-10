#ifndef IMPL_H
#define IMPL_H

#include <stdint.h>
#include <stdarg.h>
#include <stdio.h>
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "stm32f4xx_hal_spi.h"

extern SPI_HandleTypeDef hspi1;
extern TIM_HandleTypeDef htim2;

int32_t getMotorDeltaLeft() {
	static int32_t last_pulse = 0;
	static uint32_t last_tick = 0;

	int32_t current_pulse = htim2.Instance->CNT;
	uint32_t current_tick = HAL_GetTick();

	int32_t delta_pulse = current_pulse - last_pulse;
	uint32_t delta_tick = current_tick - last_tick;

	last_tick = current_tick;
	last_pulse = current_pulse;

	return delta_pulse / delta_tick;
}

int32_t getMotorDeltaRight() {
	static int32_t last_pulse = 0;
	static uint32_t last_tick = 0;

	int32_t current_pulse = htim2.Instance->CNT;
	uint32_t current_tick = HAL_GetTick();

	int32_t delta_pulse = current_pulse - last_pulse;
	uint32_t delta_tick = current_tick - last_tick;

	last_tick = current_tick;
	last_pulse = current_pulse;

	return delta_pulse / delta_tick;
}

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

#endif
