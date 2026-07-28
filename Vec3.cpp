#include "Vec3.h"

#include <cmath>
#include <cassert>

void Vec3::Normalize() {
    double length = this->Length();
    assert (length > 0.00001);

    double param = 1.0 / length;
    x *= param;
    y *= param;
    z *= param;
}

double Vec3::Length() const { return std::sqrt(this->LengthSquared()); }

double Vec3::Distance(const Vec3 &other) const { return Vec3(*this - other).Length(); }
