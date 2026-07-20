#pragma once
class Vec3{
public:
    constexpr Vec3() noexcept = default;
    constexpr Vec3(double x, double y, double z) noexcept
        : x(x), y(y), z(z) {}

    [[nodiscard]] constexpr Vec3 operator+(const Vec3& other) const noexcept {
        return Vec3(x + other.x, y + other.y, z + other.z);
    }
    [[nodiscard]] constexpr Vec3 operator-(const Vec3& other) const noexcept {
        return Vec3(x - other.x, y - other.y, z - other.z);
    }
    [[nodiscard]] constexpr Vec3 operator*(double scalar) const noexcept {
        return Vec3(x * scalar, y * scalar, z * scalar);
    }
    [[nodiscard]] constexpr Vec3 operator/(double scalar) const {
        return Vec3(x / scalar, y / scalar, z / scalar);
    }

    [[nodiscard]] constexpr bool operator==(const Vec3&) const noexcept = default;

    constexpr Vec3& operator+=(const Vec3& other) noexcept {
        x += other.x; y += other.y; z += other.z; return *this;
    }
    constexpr Vec3& operator-=(const Vec3& other) noexcept {
        x -= other.x; y -= other.y; z -= other.z; return *this;
    }

    void Normalize();

    [[nodiscard]] constexpr double Dot(const Vec3& other) const noexcept {
        return x * other.x + y * other.y + z * other.z;
    }
    [[nodiscard]] constexpr Vec3 Cross(const Vec3& other) const noexcept {
        return Vec3(y * other.z - z * other.y, z * other.x - x * other.z, x * other.y - y * other.x);
    }
    [[nodiscard]] double Length() const;
    [[nodiscard]] constexpr double LengthSquared() const noexcept {
        return x * x + y * y + z * z;
    }
    [[nodiscard]] double Distance(const Vec3& other) const;

    double x{};
    double y{};
    double z{};
};
