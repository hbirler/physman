#pragma once
//---------------------------------------------------------------------------
#include "math/Num.hpp"
#include <cassert>
#include <cmath>
//---------------------------------------------------------------------------
/// Raylib
struct Vector3;
//---------------------------------------------------------------------------
namespace physman {
//---------------------------------------------------------------------------
struct Vec3 {
    num x = 0, y = 0, z = 0;

    constexpr num sum() const { return x + y + z; }
    constexpr num sqrlen() const;
    constexpr num len() const;
    operator Vector3() const;
    constexpr Vec3 normalized() const;
};
constexpr Vec3 operator+(const Vec3& a, const Vec3& b) { return Vec3{a.x + b.x, a.y + b.y, a.z + b.z}; }
constexpr Vec3 operator-(const Vec3& a, const Vec3& b) { return Vec3{a.x - b.x, a.y - b.y, a.z - b.z}; }
constexpr Vec3 operator*(const Vec3& a, const Vec3& b) { return Vec3{a.x * b.x, a.y * b.y, a.z * b.z}; }
constexpr Vec3 operator/(const Vec3& a, const Vec3& b) { return Vec3{a.x / b.x, a.y / b.y, a.z / b.z}; }
constexpr Vec3 operator+(const Vec3& a, num b) { return Vec3{a.x + b, a.y + b, a.z + b}; }
constexpr Vec3 operator-(const Vec3& a, num b) { return Vec3{a.x - b, a.y - b, a.z - b}; }
constexpr Vec3 operator*(const Vec3& a, num b) { return Vec3{a.x * b, a.y * b, a.z * b}; }
constexpr Vec3 operator/(const Vec3& a, num b) { return Vec3{a.x / b, a.y / b, a.z / b}; }
constexpr Vec3 operator+(num a, const Vec3& b) { return Vec3{a + b.x, a + b.y, a + b.z}; }
constexpr Vec3 operator-(num a, const Vec3& b) { return Vec3{a - b.x, a - b.y, a - b.z}; }
constexpr Vec3 operator*(num a, const Vec3& b) { return Vec3{a * b.x, a * b.y, a * b.z}; }
constexpr Vec3 operator/(num a, const Vec3& b) { return Vec3{a / b.x, a / b.y, a / b.z}; }
constexpr num Vec3::sqrlen() const { return ((*this) * (*this)).sum(); }
constexpr num Vec3::len() const { return sqrt(sqrlen()); }
constexpr Vec3 Vec3::normalized() const { return (*this) / this->len(); }
//---------------------------------------------------------------------------
}
//---------------------------------------------------------------------------