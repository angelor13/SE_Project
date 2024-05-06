#include "vec3.hpp"

vec3::vec3(float componentx, float componenty, float componentz) {
    x = componentx;
    y = componenty;
    z = componentz;
}

// vec3 += vec3
vec3& vec3::operator+=(vec3 operand) {
    *this = *this + operand;
    return *this;
}

// vec3 -= vec3
vec3& vec3::operator-=(vec3 operand) {
    *this = *this - operand;
    return *this;
}

// vec3 *= vec3
vec3& vec3::operator*=(vec3 operand) {
    *this = *this * operand;
    return *this;
}

// vec3 *= float
vec3& vec3::operator*=(float operand) {
    *this = *this * operand;
    return *this;
}

// vec3 /= vec3
vec3& vec3::operator/=(vec3 operand) {
    *this = *this / operand;
    return *this;
}

// vec3 /= float
vec3& vec3::operator/=(float operand) {
    *this = *this / operand;
    return *this;
}

// vec3 = -vec3
vec3 operator-(vec3 operand) {
    return vec3(
        -operand.x, 
        -operand.y, 
        -operand.z
    );
}

// vec3 = vec3 + vec3
vec3 operator+(vec3 operand1, vec3 operand2) {
    return vec3(
        operand1.x + operand2.x, 
        operand1.y + operand2.y, 
        operand1.z + operand2.z
    );
}

// vec3 = vec3 - vec3
vec3 operator-(vec3 operand1, vec3 operand2) {
    return vec3(
        operand1.x - operand2.x, 
        operand1.y - operand2.y, 
        operand1.z - operand2.z
    );
}

// vec3 = vec3 * vec3
vec3 operator*(vec3 operand1, vec3 operand2) {
    return vec3(
        operand1.x * operand2.x, 
        operand1.y * operand2.y, 
        operand1.z * operand2.z
    );
}

// vec3 = float * vec3
vec3 operator*(float operand1, vec3 operand2) {
    return vec3(
        operand1 * operand2.x, 
        operand1 * operand2.y, 
        operand1 * operand2.z
    );
}

// vec3 = vec3 * float
vec3 operator*(vec3 operand1, float operand2) {
    return vec3(
        operand1.x * operand2, 
        operand1.y * operand2, 
        operand1.z * operand2
    );
}

// vec3 = vec3 / vec3
vec3 operator/(vec3 operand1, vec3 operand2) {
    return vec3(
        operand1.x / operand2.x, 
        operand1.y / operand2.y, 
        operand1.z / operand2.z
    );
}

// vec3 = vec3 / float
vec3 operator/(vec3 operand1, float operand2) {
    return vec3(
        operand1.x / operand2, 
        operand1.y / operand2, 
        operand1.z / operand2
    );
}

float vec3::dot(vec3 operand) {
    return ((x * operand.x) + (y * operand.y) + (z * operand.z));
}

vec3 vec3::cross(vec3 operand) {
    return vec3(
        (y * operand.z - z * operand.y),
        (z * operand.x - x * operand.z),
        (x * operand.y - y * operand.x)
    );
}

float vec3::magnitude_squared() {
    return x * x + y * y + z * z;
}

float vec3::magnitude() {
    return sqrt(x * x + y * y + z * z);
}

vec3 vec3::unit(){
    float mag = magnitude();

    if (mag == 0) {
        return vec3(0, 0, 0);
    }

    return vec3(x, y, z) / mag;
}