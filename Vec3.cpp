#include "Vec3.h"

#include <cmath>
#include <cassert>

void Vec3::Normalize() {
    float length = this->Length();
    assert (length > 0.00001f);

    float param = 1.0f / length;
    x *= param;
    y *= param;
    z *= param;
}

double Vec3::Length() const { return std::sqrt(this->LengthSquared()); }

double Vec3::Distance(const Vec3 &other) const { return Vec3(*this - other).Length(); }
