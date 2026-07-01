#pragma once
class Vec3{
public:
    Vec3();
    Vec3(float x, float y, float z) 
        : x(x), y(y), z(z) {}

    Vec3 operator+(const Vec3& other) const;
    Vec3 operator-(const Vec3& other) const;
    Vec3 operator*(float scalar) const;
    Vec3 operator/(float scalar) const;

    Vec3& operator+=(const Vec3& other);
    Vec3& operator-=(const Vec3& other);

    void Normalize();

    float Dot(const Vec3& other) const;
    Vec3 Cross(const Vec3& other) const;
    float Length() const; 
    float LengthSquared() const;
    float Distance(const Vec3& other) const;

    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
};