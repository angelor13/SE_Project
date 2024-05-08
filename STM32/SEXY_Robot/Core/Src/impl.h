#ifndef IMPL_H
#define IMPL_H

#include <stdint.h>
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "stm32f4xx_hal_spi.h"

extern SPI_HandleTypeDef hspi1;

static inline uint32_t getTick()
{
    uint32_t ms;
    uint32_t st;
    uint32_t up = HAL_GetTick();

    do
    {
        ms = up;
        st = SysTick->VAL;
        asm volatile("nop");
        asm volatile("nop");
    } while (ms != up);

    return ms * 1000 - st / ((SysTick->LOAD + 1) / 1000);
}

// Stores the last measured deltas between each encoder's detected RISING edges

int write(unsigned char *ptr, int len) {
    static uint8_t rc = USBD_OK;

    do {
        rc = CDC_Transmit_FS(ptr, len);
    } while (USBD_BUSY == rc);

    if (USBD_FAIL == rc) {
        /// NOTE: Should never reach here.
        /// TODO: Handle this error.
        return 0;
    }
    return len;
}

void transmitDataSPI(uint32_t value, uint8_t flag) {
#define unpack(x) {(x >> 0) & 0xFF, (x >> 8) & 0xFF, (x >> 16) & 0xFF, (x >> 24) & 0xFF}
#define pack(x) (x[3] << 24) | (x[2] << 16) | (x[1] << 8) (x[0] << 0)

	HAL_SPI_Transmit(&hspi1, &flag, 1, 100);

	uint8_t _value[] = unpack(value);

	for (int i = 0; i < 4; i++) {
		HAL_SPI_Transmit(&hspi1, &_value[i], 1, 100);
		HAL_Delay(10);
	}

#undef unpack
#undef pack
}

uint32_t receiveDataSPI(uint8_t flag) {
#define unpack(x) {(x >> 0) & 0xFF, (x >> 8) & 0xFF, (x >> 16) & 0xFF, (x >> 24) & 0xFF}
#define pack(x) (x[3] << 24) | (x[2] << 16) | (x[1] << 8) (x[0] << 0)

	uint8_t _flag = 0;
	uint8_t _value[4] = { 0 };

	do {
		HAL_SPI_Receive(&hspi1, &_flag, 1, 100);
	} while (_flag != flag);


	for (int i = 0; i < 4; i++) {
		HAL_SPI_Receive(&hspi1, &_value[i], 1, 100);
		HAL_Delay(10);
	}

	return pack(_value);

#undef unpack
#undef pack
}

int32_t getEncoderValueLeft() {
	static uint32_t encoder_tick = 0;
	static uint32_t encoder_last_state_primary = 0;
	static uint32_t encoder_last_state_secondary = 0;

	uint32_t encoder_state_primary = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0);
	uint32_t encoder_state_secondary = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1);

	uint32_t current_tick = getTick();
	int32_t delta_tick = current_tick - encoder_tick;
	encoder_tick = current_tick;

	return delta_tick;
}

int32_t getEncoderValueRight() {
	return 0;
}

void update() {
	int32_t left = getEncoderValueLeft();
	int32_t right = getEncoderValueRight();

	transmitDataSPI(left, 0xAB);
	transmitDataSPI(right, 0xCD);
}

void updateEncoderTicks() {
	static uint32_t encoder_tick = 0;
	static uint32_t encoder_last_state_primary = 0;
	static uint32_t encoder_last_state_secondary = 0;

	uint32_t encoder_state_primary = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0);
	uint32_t encoder_state_secondary = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1);

	char text[32] = { 0 };

	if (encoder_state_primary > encoder_last_state_primary) {
		sprintf(text, "Tick 1\n");
		_write(0, text, sizeof(text));
	}

	if (encoder_state_secondary > encoder_last_state_secondary) {
		uint32_t current_tick = getTick();
		sprintf(text, "Tick 2 & DELTA = %u\n", current_tick - encoder_tick);
		_write(0, text, sizeof(text));
		encoder_tick = current_tick;
	}

	encoder_last_state_primary = encoder_state_primary;
	encoder_last_state_secondary = encoder_state_secondary;
}

#endif
