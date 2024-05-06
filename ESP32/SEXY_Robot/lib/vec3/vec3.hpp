#ifndef VECTOR3_H
#define VECTOR3_H

#include <cmath>

class vec3 {
public:
    float x, y, z;

    vec3(float x = 0, float y = 0, float z = 0);

    vec3& operator+=(vec3);
    vec3& operator-=(vec3);

    vec3& operator*=(vec3);
    vec3& operator*=(float);

    vec3& operator/=(vec3);
    vec3& operator/=(float);

    friend vec3 operator+(vec3, vec3);
    friend vec3 operator-(vec3, vec3);
    
    friend vec3 operator*(vec3, vec3);
    friend vec3 operator*(float, vec3);
    friend vec3 operator*(vec3, float);

    friend vec3 operator/(vec3, vec3);
    friend vec3 operator/(vec3, float);

    friend vec3 operator-(vec3);

    float dot(vec3);
    vec3 cross(vec3);
    vec3 unit();

    float magnitude_squared();
    float magnitude();  
};

#endif