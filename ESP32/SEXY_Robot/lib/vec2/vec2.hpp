#ifndef VECTOR3_H
#define VECTOR3_H

#include <cmath>

class vec2 {
public:
    float x, y;

    vec2(float x = 0, float y = 0);

    vec2& operator+=(vec2);
    vec2& operator-=(vec2);

    vec2& operator*=(vec2);
    vec2& operator*=(float);

    vec2& operator/=(vec2);
    vec2& operator/=(float);

    friend vec2 operator+(vec2, vec2);
    friend vec2 operator-(vec2, vec2);
    
    friend vec2 operator*(vec2, vec2);
    friend vec2 operator*(float, vec2);
    friend vec2 operator*(vec2, float);

    friend vec2 operator/(vec2, vec2);
    friend vec2 operator/(vec2, float);

    friend vec2 operator-(vec2);

    float dot(vec2);
    vec2 unit();

    float magnitude_squared();
    float magnitude();  
};

#endif