#pragma once

#include "Vec3.h"
class Mat3{
public:
    Mat3();
    Mat3(
        float m00, float m01, float m02,
        float m10, float m11, float m12,
        float m20, float m21, float m22
    );

    static Mat3 RotationX(float angle);
    static Mat3 RotationY(float angle);
    static Mat3 RotationZ(float angle);

    Mat3 operator*(const Mat3& other) const;
    Vec3 operator*(const Vec3& vec) const;

private:
    float m[3][3];
};