#pragma once
//---------------------------------------------------------------------------
#include "math/Num.hpp"
#include <numeric>
#include <concepts>
#include <functional>
#include <vector>
//---------------------------------------------------------------------------
namespace physman {
//---------------------------------------------------------------------------
class Vec : public std::vector<num> {
    static num getAt(const Vec& v, size_t index) { return v[index]; }
    static num getAt(num v, size_t) { return v; }

    Vec& applyImpl(auto&& op) {
        for (size_t i = 0; i < size(); i++)
            operator[](i) = op(operator[](i));
        return *this;
    }
    Vec& applyImpl(auto&& op, auto&& v) {
        for (size_t i = 0; i < size(); i++)
            operator[](i) = op(operator[](i), getAt(v, i));
        return *this;
    }
    Vec& applyImplRev(auto&& op, auto&& v) {
        for (size_t i = 0; i < size(); i++)
            operator[](i) = op(getAt(v, i), operator[](i));
        return *this;
    }

    public:
    using std::vector<num>::vector;
    using std::vector<num>::push_back;

    Vec& operator+=(const Vec& v) { return applyImpl(std::plus{}, v); }
    Vec& operator-=(const Vec& v) { return applyImpl(std::minus{}, v); }
    Vec& operator*=(const Vec& v) { return applyImpl(std::multiplies{}, v); }
    Vec& operator/=(const Vec& v) { return applyImpl(std::divides{}, v); }
    Vec& operator+=(num v) { return applyImpl(std::plus{}, v); }
    Vec& operator-=(num v) { return applyImpl(std::minus{}, v); }
    Vec& operator*=(num v) { return applyImpl(std::multiplies{}, v); }
    Vec& operator/=(num v) { return applyImpl(std::divides{}, v); }
    Vec& negate() { return applyImpl(std::negate{}); }

    Vec slice(size_t start, size_t count) const {
        return Vec(std::vector<num>::begin() + start, std::vector<num>::begin() + start + count);
    }

    static Vec concat(const Vec& a, const Vec& b) {
        Vec result;
        result.reserve(a.size() + b.size());
        result.insert(result.end(), a.begin(), a.end());
        result.insert(result.end(), b.begin(), b.end());
        return result;
    }

    num sum() const { return std::accumulate(begin(), end(), num{0}); }

    static Vec apply(auto&& op, Vec&& v1) { return std::move(v1.applyImpl(op)); }
    static Vec apply(auto&& op, const Vec& v1) { return std::move(Vec{v1}.applyImpl(op)); }
    static Vec apply(auto&& op, Vec&& v1, Vec&& v2) { return std::move(v1.applyImpl(op, v2)); }
    static Vec apply(auto&& op, Vec&& v1, const Vec& v2) { return std::move(v1.applyImpl(op, v2)); }
    static Vec apply(auto&& op, const Vec& v1, Vec&& v2) { return std::move(v2.applyImplRev(op, v1)); }
    static Vec apply(auto&& op, const Vec& v1, const Vec& v2) { return std::move(Vec{v1}.applyImpl(op, v2)); }
    static Vec apply(auto&& op, Vec&& v1, num v2) { return std::move(v1.applyImpl(op, v2)); }
    static Vec apply(auto&& op, const Vec& v1, num v2) { return std::move(Vec{v1}.applyImpl(op, v2)); }
    static Vec apply(auto&& op, num v2, Vec&& v1) { return std::move(v1.applyImplRev(op, v2)); }
    static Vec apply(auto&& op, num v2, const Vec& v1) { return std::move(Vec{v1}.applyImplRev(op, v2)); }
};
//---------------------------------------------------------------------------
#define PHYSMAN_VEC_DEFOP(op, func) \
static inline Vec op(Vec&& v1, Vec&& v2) { return Vec::apply(func, std::move(v1), std::move(v2)); } \
static inline Vec op(Vec&& v1, const Vec& v2) { return Vec::apply(func, std::move(v1), v2); } \
static inline Vec op(const Vec& v1, Vec&& v2) { return Vec::apply(func, v1, std::move(v2)); } \
static inline Vec op(const Vec& v1, const Vec& v2) { return Vec::apply(func, v1, v2); } \
static inline Vec op(Vec&& v1, num v2) { return Vec::apply(func, std::move(v1), v2); } \
static inline Vec op(const Vec& v1, num v2) { return Vec::apply(func, v1, v2); } \
static inline Vec op(num v1, Vec&& v2) { return Vec::apply(func, v1, std::move(v2)); } \
static inline Vec op(num v1, const Vec& v2) { return Vec::apply(func, v1, v2); }
PHYSMAN_VEC_DEFOP(operator+, std::plus{})
PHYSMAN_VEC_DEFOP(operator-, std::minus{})
PHYSMAN_VEC_DEFOP(operator*, std::multiplies{})
PHYSMAN_VEC_DEFOP(operator/, std::divides{})
#undef PHYSMAN_VEC_DEFOP
#define PHYSMAN_VEC_DEFOP(op, func) \
inline Vec op(Vec&& v) { return Vec::apply(func, std::move(v)); } \
inline Vec op(const Vec& v) { return Vec::apply(func, v); }
PHYSMAN_VEC_DEFOP(operator-, std::negate{});
PHYSMAN_VEC_DEFOP(sqrt, [](num v) { return std::sqrt(v); });
#undef PHYSMAN_VEC_DEFOP
//---------------------------------------------------------------------------
}
//---------------------------------------------------------------------------