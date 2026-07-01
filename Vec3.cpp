#include "Vec3.h"

#include <cmath>
#include <cassert>

Vec3 Vec3::operator+(const Vec3& other) const { 
    return Vec3(x + other.x, y + other.y, z + other.z); 
}

Vec3 Vec3::operator-(const Vec3& other) const {
    return Vec3(x - other.x, y - other.y, z - other.z);
}

Vec3& Vec3::operator+=(const Vec3& other)  { 
    x += other.x;
    y += other.y;
    z += other.z;
    return *this;
}

Vec3& Vec3::operator-=(const Vec3& other)  { 
    x -= other.x;
    y -= other.y;
    z -= other.z;
    return *this;
}

Vec3 Vec3::operator*(float scalar) const { 
    return Vec3(x * scalar, y * scalar, z * scalar); 
}

Vec3 Vec3::operator/(float scalar) const { 
    return Vec3(x / scalar, y / scalar, z / scalar); 
}


void Vec3::Normalize() {
    float length = this->Length();
    assert (length > 0.00001f); 

    float param = 1.0f / length;
    x *= param;
    y *= param;
    z *= param;
}

float Vec3::LengthSquared() const { return x * x + y * y + z * z; }

float Vec3::Distance(const Vec3 &other) const { return Vec3(*this - other).Length(); }

float Vec3::Dot(const Vec3 &other) const { return x * other.x + y * other.y + z * other.z; }

Vec3 Vec3::Cross(const Vec3& other) const { 
    return Vec3(y * other.z - z * other.y, z * other.x - x * other.z, x * other.y - y * other.x); 
} 

float Vec3::Length() const { return std::sqrt(this->LengthSquared()); }
