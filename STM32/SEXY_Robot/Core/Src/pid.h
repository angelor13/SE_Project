#include <stdint.h>
#include "stm32f4xx_hal.h"

#define ERROR_INTEGRAL_MAX 1000
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


float PID_Control(float Kp, float Ki, float Kd, float setpoint, float value) {
    static float error_integral = 0.0;
    static float error_previous = 0.0;
    static float time_current = 0.0;
    static float time_previous = 0.0;

    // Calculate time difference
    time_current += HAL_GetTick() / 1000.0;
    double dt = time_current - time_previous;
    time_previous = time_current;

    // Calculate error
    float error = setpoint - value;

    // Proportional term
    float P = Kp * error;

    // Integral term
    error_integral = constrain(error_integral + error, 0, ERROR_INTEGRAL_MAX);
    float I = Ki * error_integral * dt;

    // Derivative term
    float D = Kd * (error - error_previous) / dt;
    error_previous = error;

    // PID Output
    float output = constrain(P + I + D, -PID_OUTPUT_MAX, PID_OUTPUT_MAX);

    return output;
}
