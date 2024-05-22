#include <stdio.h>
#include <stdint.h>

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

float lerp(a, b, t) {
    return (b - a) * t;
}

float PID_Control(float Kp, float Ki, float Kd, float setpoint, float value) {
    static float error_integral = 0.0;
    static float error_previous = 0.0;
    static float time_current = 0.0;
    static float time_previous = 0.0;

    // Calculate time difference
    time_current += 10 / 1000.0;
    float dt = time_current - time_previous;
    time_previous = time_current;

    // Calculate error
    float error = setpoint - value;

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

int main() {
    FILE *gnuplot = popen("gnuplot", "w");
    fprintf(gnuplot, "plot '-' with lines\n");

    float x = 0;
    float y = 0;
    for (; x < 100; x++) {
        y = .01 * PID_Control(20, 900, 0.10, 60, y);
        printf("%f\n", y);
        fprintf(gnuplot, "%g %g\n", x, y);
    }

    for (; x < 200; x++) {
        y = .01 * PID_Control(20, 900, 0.10, 90, y);
        printf("%f\n", y);
        fprintf(gnuplot, "%g %g\n", x, y);
    }

    for (; x < 300; x++) {
        y = .01 * PID_Control(20, 900, 0.10, -90, y);
        printf("%f\n", y);
        fprintf(gnuplot, "%g %g\n", x, y);
    }
    for (; x < 400; x++) {
        y = .01 * PID_Control(20, 900, 0.10, 0, y);
        printf("%f\n", y);
        fprintf(gnuplot, "%g %g\n", x, y);
    }


    fprintf(gnuplot, "e\n");
    fflush(gnuplot);
    while(1);
}