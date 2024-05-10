#ifndef IMPL_H
#define IMPL_H

#include <stdint.h>
#include <stdarg.h>
#include <stdio.h>
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "stm32f4xx_hal_spi.h"

extern SPI_HandleTypeDef hspi1;

// Stores the last measured deltas between each encoder's detected RISING edges

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

//
//int32_t getEncoderValueLeft() {
//	static uint32_t encoder_tick = 0;
//	static uint32_t encoder_last_state_primary = 0;
//	static uint32_t encoder_last_state_secondary = 0;
//
//	uint32_t encoder_state_primary = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0);
//	uint32_t encoder_state_secondary = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1);
//
//	uint32_t current_tick = getTick();
//	int32_t delta_tick = current_tick - encoder_tick;
//	encoder_tick = current_tick;
//
//	return delta_tick;
//}
//
//int32_t getEncoderValueRight() {
//	return 0;
//}
//
//void update() {
//	int32_t left = getEncoderValueLeft();
//	int32_t right = getEncoderValueRight();
//
//}
//
//void updateEncoderTicks() {
//	static uint32_t encoder_tick = 0;
//	static uint32_t encoder_last_state_primary = 0;
//	static uint32_t encoder_last_state_secondary = 0;
//
//	uint32_t encoder_state_primary = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0);
//	uint32_t encoder_state_secondary = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1);
//
//	char text[32] = { 0 };
//
//	if (encoder_state_primary > encoder_last_state_primary) {
//		sprintf(text, "Tick 1\n");
//		_write(0, text, sizeof(text));
//	}
//
//	if (encoder_state_secondary > encoder_last_state_secondary) {
//		uint32_t current_tick = getTick();
//		sprintf(text, "Tick 2 & DELTA = %u\n", current_tick - encoder_tick);
//		_write(0, text, sizeof(text));
//		encoder_tick = current_tick;
//	}
//
//	encoder_last_state_primary = encoder_state_primary;
//	encoder_last_state_secondary = encoder_state_secondary;
//}

#endif
