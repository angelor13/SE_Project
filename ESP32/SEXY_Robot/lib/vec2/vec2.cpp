#include "vec2.hpp"

vec2::vec2(float componentx, float componenty) {
    x = componentx;
    y = componenty;
}

// vec2 += vec2
vec2& vec2::operator+=(vec2 operand) {
    *this = *this + operand;
    return *this;
}

// vec2 -= vec2
vec2& vec2::operator-=(vec2 operand) {
    *this = *this - operand;
    return *this;
}

// vec2 *= vec2
vec2& vec2::operator*=(vec2 operand) {
    *this = *this * operand;
    return *this;
}

// vec2 *= float
vec2& vec2::operator*=(float operand) {
    *this = *this * operand;
    return *this;
}

// vec2 /= vec2
vec2& vec2::operator/=(vec2 operand) {
    *this = *this / operand;
    return *this;
}

// vec2 /= float
vec2& vec2::operator/=(float operand) {
    *this = *this / operand;
    return *this;
}

// vec2 = -vec2
vec2 operator-(vec2 operand) {
    return vec2(
        -operand.x, 
        -operand.y
    );
}

// vec2 = vec2 + vec2
vec2 operator+(vec2 operand1, vec2 operand2) {
    return vec2(
        operand1.x + operand2.x, 
        operand1.y + operand2.y
    );
}

// vec2 = vec2 - vec2
vec2 operator-(vec2 operand1, vec2 operand2) {
    return vec2(
        operand1.x - operand2.x, 
        operand1.y - operand2.y
    );
}

// vec2 = vec2 * vec2
vec2 operator*(vec2 operand1, vec2 operand2) {
    return vec2(
        operand1.x * operand2.x, 
        operand1.y * operand2.y
    );
}

// vec2 = float * vec2
vec2 operator*(float operand1, vec2 operand2) {
    return vec2(
        operand1 * operand2.x, 
        operand1 * operand2.y
    );
}

// vec2 = vec2 * float
vec2 operator*(vec2 operand1, float operand2) {
    return vec2(
        operand1.x * operand2, 
        operand1.y * operand2
    );
}

// vec2 = vec2 / vec2
vec2 operator/(vec2 operand1, vec2 operand2) {
    return vec2(
        operand1.x / operand2.x, 
        operand1.y / operand2.y
    );
}

// vec2 = vec2 / float
vec2 operator/(vec2 operand1, float operand2) {
    return vec2(
        operand1.x / operand2, 
        operand1.y / operand2
    );
}

float vec2::dot(vec2 operand) {
    return ((x * operand.x) + (y * operand.y));
}

float vec2::magnitude_squared() {
    return x * x + y * y;
}

float vec2::magnitude() {
    return sqrt(x * x + y * y);
}

vec2 vec2::unit(){
    float mag = magnitude();

    if (mag == 0) {
        return vec2(0, 0);
    }

    return vec2(x, y) / mag;
}