#include "Mat3.h"

#include <cmath>

Mat3::Mat3()
{
    m[0][0] = 1.0f;
    m[0][1] = 0.0f;
    m[0][2] = 0.0f;

    m[1][0] = 0.0f;
    m[1][1] = 1.0f;
    m[1][2] = 0.0f;

    m[2][0] = 0.0f;
    m[2][1] = 0.0f;
    m[2][2] = 1.0f;
}

Mat3::Mat3(
    float m00, float m01, float m02,
    float m10, float m11, float m12,
    float m20, float m21, float m22)
{
    m[0][0] = m00;
    m[0][1] = m01;
    m[0][2] = m02;

    m[1][0] = m10;
    m[1][1] = m11;
    m[1][2] = m12;

    m[2][0] = m20;
    m[2][1] = m21;
    m[2][2] = m22;
}

Mat3 Mat3::RotationX(float angle)
{
    float c = std::cos(angle);
    float s = std::sin(angle);

    return Mat3(
        1.0f, 0.0f, 0.0f,
        0.0f, c,   -s,
        0.0f, s,    c
    );
}

Mat3 Mat3::RotationY(float angle)
{
    float c = std::cos(angle);
    float s = std::sin(angle);

    return Mat3(
         c,   0.0f, s,
        0.0f, 1.0f, 0.0f,
        -s,   0.0f, c
    );
}

Mat3 Mat3::RotationZ(float angle)
{
    float c = std::cos(angle);
    float s = std::sin(angle);

    return Mat3(
        c,   -s,   0.0f,
        s,    c,   0.0f,
        0.0f, 0.0f, 1.0f
    );
}

Mat3 Mat3::operator*(const Mat3& other) const
{
    Mat3 result;

    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            result.m[i][j] = 0.0f;

            for (int k = 0; k < 3; ++k)
            {
                result.m[i][j] +=
                    m[i][k] * other.m[k][j];
            }
        }
    }

    return result;
}

Vec3 Mat3::operator*(const Vec3& other) const
{
    return Vec3(
        m[0][0] * other.x +
        m[0][1] * other.y +
        m[0][2] * other.z,

        m[1][0] * other.x +
        m[1][1] * other.y +
        m[1][2] * other.z,

        m[2][0] * other.x +
        m[2][1] * other.y +
        m[2][2] * other.z
    );
}