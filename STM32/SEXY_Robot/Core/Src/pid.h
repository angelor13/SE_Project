#include <stdint.h>
#include "stm32f4xx_hal.h"
#include "xprintf.h"

#define ERROR_INTEGRAL_MAX 10000
#define PID_OUTPUT_MAX 65535

float constrain(float x, float min, float max) {
	if (x < min) {
		return min;
	} else if (x > max) {
		return max;
	} else {
		return x;
	}
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

float PID_Control(float Kp, float Ki, float Kd, float setpoint, float value) {
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
    xprintf("X:%d, %d\n", (int32_t)(error * 1e6), (int32_t)value);

    // Proportional term
    float P = Kp * error;

    // Integral term
    error_integral = constrain(error_integral + error, -ERROR_INTEGRAL_MAX, ERROR_INTEGRAL_MAX);
    float I = Ki * error_integral * dt;

    // Derivative term
    float D = Kd * (error - error_previous) / dt;
    error_previous = error;

    // PID Output
    float output = constrain(P + I + D, -PID_OUTPUT_MAX, PID_OUTPUT_MAX);

    return output;
}
