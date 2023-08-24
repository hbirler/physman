#pragma once
//---------------------------------------------------------------------------
#include "math/Num.hpp"
#include "math/Vec.hpp"
#include <array>
#include <cmath>
#include <span>
#include <tuple>
#include <type_traits>
#include <tl/function_ref.hpp>
//---------------------------------------------------------------------------
namespace physman::math {
//---------------------------------------------------------------------------
struct ValScope {
    Vec xs;
    Vec vs;
    Vec ps;
    num t;
    constexpr num getPos(unsigned id) const { return xs[id]; }
    constexpr num getVel(unsigned id) const { return vs[id]; }
    constexpr num getParam(unsigned id) const { return ps[id]; }
    constexpr num getTime() const { return t; }

    /// Helper for tests
    template <size_t Components>
    static auto make(Vec xs, Vec vs, Vec ps, num t);
};
//---------------------------------------------------------------------------
namespace val {
//---------------------------------------------------------------------------
struct Val {
    constexpr virtual num evaluate(const ValScope&) const = 0;
    template <typename T>
    static constexpr unsigned numComponents();
    template <typename T>
    static constexpr unsigned numParams();
};
//---------------------------------------------------------------------------
struct Zero final : Val {
    num evaluate(const ValScope&) const final { return 0; }
    constexpr auto deriveBy(auto) const { return Zero{}; }
    static constexpr void visit(auto f) { f(std::type_identity<Zero>{}); }
};
//---------------------------------------------------------------------------
struct One final : Val {
    num evaluate(const ValScope&) const final { return 1; }
    constexpr auto deriveBy(auto) const { return Zero{}; }
    static constexpr void visit(auto f) { f(std::type_identity<One>{}); }
};
//---------------------------------------------------------------------------
struct Time final : Val {
    num evaluate(const ValScope& vs) const final { return vs.getTime(); }
    constexpr auto deriveBy(Time) const { return One{}; }
    constexpr auto deriveBy(auto) const { return Zero{}; }
    static constexpr void visit(auto f) { f(std::type_identity<Time>{}); }
};
//---------------------------------------------------------------------------
struct Const final : Val {
    num c;
    constexpr Const(num c) : c(c) {}
    constexpr num evaluate(const ValScope&) const final { return c; }
    constexpr auto deriveBy(auto) const { return Zero{}; }
    static constexpr void visit(auto f) { f(std::type_identity<Const>{}); }
};
//---------------------------------------------------------------------------
template <unsigned Id>
struct Param final : Val {
    static constexpr unsigned getParamId() { return Id; }
    constexpr num evaluate(const ValScope& vs) const final { return vs.getParam(Id); }

    constexpr auto deriveBy(Param) const { return One{}; }
    constexpr auto deriveBy(auto) const { return Zero{}; }
    static constexpr void visit(auto f) { f(std::type_identity<Param>{}); }
};
//---------------------------------------------------------------------------
template <unsigned Id>
struct Vel final : Val {
    static constexpr unsigned getComponentId() { return Id; }
    constexpr num evaluate(const ValScope& vs) const final { return vs.getVel(Id); }

    constexpr auto deriveBy(Vel) const { return One{}; }
    constexpr auto deriveBy(auto) const { return Zero{}; }
    static constexpr void visit(auto f) { f(std::type_identity<Vel>{}); }
};
//---------------------------------------------------------------------------
template <unsigned Id>
struct Pos final : Val {
    static constexpr unsigned getComponentId() { return Id; }
    constexpr num evaluate(const ValScope& vs) const final { return vs.getPos(Id); }

    constexpr auto deriveBy(Time) const { return Vel<Id>{}; }
    constexpr auto deriveBy(Pos) const { return One{}; }
    constexpr auto deriveBy(auto) const { return Zero{}; }
    static constexpr void visit(auto f) { f(std::type_identity<Pos>{}); }
};
//---------------------------------------------------------------------------
template <std::derived_from<Val> A, std::derived_from<Val> B>
struct Add : Val {
    [[no_unique_address]] A a;
    [[no_unique_address]] B b;
    constexpr Add(A a, B b) : a(a), b(b) {}
    constexpr num evaluate(const ValScope& vs) const final { return a.evaluate(vs) + b.evaluate(vs); }
    constexpr auto deriveBy(auto v) const { return a.deriveBy(v) + b.deriveBy(v); }
    static constexpr void visit(auto f) { f(std::type_identity<Add>{}); A::visit(f); B::visit(f); }
};
//---------------------------------------------------------------------------
template <std::derived_from<Val> A, std::derived_from<Val> B>
struct Sub : Val {
    [[no_unique_address]] A a;
    [[no_unique_address]] B b;
    constexpr Sub(A a, B b) : a(a), b(b) {}
    constexpr num evaluate(const ValScope& vs) const final { return a.evaluate(vs) - b.evaluate(vs); }
    constexpr auto deriveBy(auto v) const { return a.deriveBy(v) - b.deriveBy(v); }
    static constexpr void visit(auto f) { f(std::type_identity<Sub>{}); A::visit(f); B::visit(f); }
};
//---------------------------------------------------------------------------
template <std::derived_from<Val> A>
struct Neg : Val {
    [[no_unique_address]] A a;
    constexpr Neg(A a) : a(a) {}
    constexpr num evaluate(const ValScope& vs) const final { return -a.evaluate(vs); }
    constexpr auto deriveBy(auto v) const { return -a.deriveBy(v); }
    static constexpr void visit(auto f) { f(std::type_identity<Neg>{}); A::visit(f); }
};
//---------------------------------------------------------------------------
template <std::derived_from<Val> A, std::derived_from<Val> B>
struct Mul : Val {
    [[no_unique_address]] A a;
    [[no_unique_address]] B b;
    constexpr Mul(A a, B b) : a(a), b(b) {}
    constexpr num evaluate(const ValScope& vs) const final { return a.evaluate(vs) * b.evaluate(vs); }
    constexpr auto deriveBy(auto v) const { return a.deriveBy(v) * b + a * b.deriveBy(v); }
    static constexpr void visit(auto f) { f(std::type_identity<Mul>{}); A::visit(f); B::visit(f); }
};
//---------------------------------------------------------------------------
template <std::derived_from<Val> A>
struct Recip : Val {
    [[no_unique_address]] A a;
    constexpr Recip(A a) : a(a) {}
    constexpr num evaluate(const ValScope& vs) const final { return 1.0 / a.evaluate(vs); }
    constexpr auto deriveBy(auto v) const { return -(a.deriveBy(v) * Recip<decltype(a * a)>{a * a}); }
    static constexpr void visit(auto f) { f(std::type_identity<Recip>{}); A::visit(f); }
};
//---------------------------------------------------------------------------
template <std::derived_from<Val> A, std::derived_from<Val> B>
struct Div : Val {
    [[no_unique_address]] A a;
    [[no_unique_address]] B b;
    constexpr Div(A a, B b) : a(a), b(b) {}
    constexpr num evaluate(const ValScope& vs) const final { return a.evaluate(vs) / b.evaluate(vs); }
    constexpr auto deriveBy(auto v) const { return (a * Recip{b}).deriveBy(v); }
    static constexpr void visit(auto f) { f(std::type_identity<Div>{}); A::visit(f); B::visit(f); }
};
//---------------------------------------------------------------------------
template <std::derived_from<Val> A>
struct Sin : Val {
    [[no_unique_address]] A a;
    constexpr Sin(A a) : a(a) {}
    constexpr num evaluate(const ValScope& vs) const final { return std::sin(a.evaluate(vs)); }
    constexpr auto deriveBy(auto v) const;
    static constexpr void visit(auto f) { f(std::type_identity<Sin>{}); A::visit(f); }
};
//---------------------------------------------------------------------------
template <std::derived_from<Val> A>
struct Cos : Val {
    [[no_unique_address]] A a;
    constexpr Cos(A a) : a(a) {}
    constexpr num evaluate(const ValScope& vs) const final { return std::cos(a.evaluate(vs)); }
    constexpr auto deriveBy(auto v) const;
    static constexpr void visit(auto f) { f(std::type_identity<Cos>{}); A::visit(f); }
};
//---------------------------------------------------------------------------
template <std::derived_from<Val> A>
constexpr auto Sin<A>::deriveBy(auto) const { return Cos{a}; }
//---------------------------------------------------------------------------
template <std::derived_from<Val> A>
constexpr auto Cos<A>::deriveBy(auto) const { return -Sin{a}; }
//---------------------------------------------------------------------------
template <std::derived_from<Val> X, std::derived_from<Val> Y, std::derived_from<Val> Z>
struct Vec3 {
    [[no_unique_address]] X x;
    [[no_unique_address]] Y y;
    [[no_unique_address]] Z z;
    constexpr Vec3(X x, Y y, Z z) : x(x), y(y), z(z) {}
    constexpr auto deriveBy(auto v);
    constexpr auto sum() const { return x + y + z; }
};
//---------------------------------------------------------------------------
template <typename Cond, std::derived_from<Val> A, std::derived_from<Val> B, std::derived_from<Val> C>
struct If : Val {
    [[no_unique_address]] Cond cond;
    [[no_unique_address]] A a;
    [[no_unique_address]] B b;
    [[no_unique_address]] C c;
    constexpr If(Cond cond, A a, B b, C c) : a(a), b(b), c(c) {}
    constexpr num evaluate(const ValScope& vs) const final { return cond(a.evaluate(vs)) ? b.evaluate(vs) : c.evaluate(vs); }
    constexpr auto deriveBy(auto v) {
        return If<Cond, A, decltype(b.deriveBy(v)), decltype(c.deriveBy(v))>{cond, a, b.deriveBy(v), c.deriveBy(v)};
    }
    static constexpr void visit(auto f) { f(std::type_identity<If>{}); A::visit(f); B::visit(f); C::visit(f); }
};
//---------------------------------------------------------------------------
template <typename T>
constexpr unsigned Val::numComponents()
// Get number of components
{
    unsigned result = 0;
    T::visit([&]<typename T2>(std::type_identity<T2>) {
        if constexpr (requires { { T2::getComponentId() } -> std::same_as<unsigned>; })
            result = std::max(result, T2::getComponentId() + 1);
    });
    return result;
}
//---------------------------------------------------------------------------
template <typename T>
constexpr unsigned Val::numParams()
// Get number of components
{
    unsigned result = 0;
    T::visit([&]<typename T2>(std::type_identity<T2>) {
        if constexpr (requires { { T2::getParamId() } -> std::same_as<unsigned>; })
            result = std::max(result, T2::getParamId() + 1);
    });
    return result;
}
//---------------------------------------------------------------------------
constexpr auto operator-(Zero) { return Zero{}; }
constexpr auto operator-(Zero, Zero) { return Zero{}; }
constexpr auto operator+(Zero, Zero) { return Zero{}; }
constexpr auto operator-(Zero, One) { return Neg{One{}}; }
constexpr auto operator+(Zero, One) { return One{}; }
constexpr auto operator-(One, Zero) { return One{}; }
constexpr auto operator+(One, Zero) { return One{}; }
constexpr auto operator-(One, One) { return Zero{}; }
template <std::derived_from<Val> A> constexpr auto operator-(A a) { return Neg{a}; }
template <std::derived_from<Val> A> constexpr auto operator+(A a, Zero) { return a; }
template <std::derived_from<Val> A> constexpr auto operator+(Zero, A a) { return a; }
template <std::derived_from<Val> A> constexpr auto operator-(A a, Zero) { return a; }
template <std::derived_from<Val> A> constexpr auto operator-(Zero, A a) { return Neg{a}; }
template <std::derived_from<Val> A> constexpr auto operator*(A a, One) { return a; }
template <std::derived_from<Val> A> constexpr auto operator*(One, A a) { return a; }
template <std::derived_from<Val> A> constexpr auto operator*(A a, Zero) { return Zero{}; }
template <std::derived_from<Val> A> constexpr auto operator/(Zero, A a) { return Zero{}; }
template <std::derived_from<Val> A> constexpr auto operator/(A a, One) { return a; }
template <std::derived_from<Val> A> constexpr auto operator/(One, A a) { return Recip{a}; }
template <std::derived_from<Val> A> constexpr auto operator*(Zero, A a) { return Zero{}; }
template <std::derived_from<Val> A, std::derived_from<Val> B> constexpr auto operator+(A a, B b) { return Add{a, b}; }
template <std::derived_from<Val> A, std::derived_from<Val> B> constexpr auto operator-(A a, B b) { return Sub{a, b}; }
template <std::derived_from<Val> A, std::derived_from<Val> B> constexpr auto operator*(A a, B b) { return Mul{a, b}; }
template <std::derived_from<Val> A, std::derived_from<Val> B> constexpr auto operator/(A a, B b) { return Div{a, b}; }
template <std::derived_from<Val> A> constexpr auto operator+(A a, num b) { return Add{a, Const{b}}; }
template <std::derived_from<Val> A> constexpr auto operator-(A a, num b) { return Sub{a, Const{b}}; }
template <std::derived_from<Val> A> constexpr auto operator*(A a, num b) { return Mul{a, Const{b}}; }
template <std::derived_from<Val> A> constexpr auto operator/(A a, num b) { return Div{a, Const{b}}; }
template <std::derived_from<Val> A> constexpr auto operator+(num b, A a) { return Add{Const{b}, a}; }
template <std::derived_from<Val> A> constexpr auto operator-(num b, A a) { return Sub{Const{b}, a}; }
template <std::derived_from<Val> A> constexpr auto operator*(num b, A a) { return Mul{Const{b}, a}; }
template <std::derived_from<Val> A> constexpr auto operator/(num b, A a) { return Div{Const{b}, a}; }
template <std::derived_from<Val> X1, std::derived_from<Val> Y1, std::derived_from<Val> Z1, std::derived_from<Val> X2, std::derived_from<Val> Y2, std::derived_from<Val> Z2>
constexpr auto operator+(Vec3<X1, Y1, Z1> v1, Vec3<X2, Y2, Z2> v2) { return Vec3{v1.x + v2.x, v1.y + v2.y, v1.z + v2.z}; }
template <std::derived_from<Val> X1, std::derived_from<Val> Y1, std::derived_from<Val> Z1, std::derived_from<Val> X2, std::derived_from<Val> Y2, std::derived_from<Val> Z2>
constexpr auto operator-(Vec3<X1, Y1, Z1> v1, Vec3<X2, Y2, Z2> v2) { return Vec3{v1.x - v2.x, v1.y - v2.y, v1.z - v2.z}; }
template <std::derived_from<Val> X1, std::derived_from<Val> Y1, std::derived_from<Val> Z1, std::derived_from<Val> X2, std::derived_from<Val> Y2, std::derived_from<Val> Z2>
constexpr auto operator*(Vec3<X1, Y1, Z1> v1, Vec3<X2, Y2, Z2> v2) { return Vec3{v1.x * v2.x, v1.y * v2.y, v1.z * v2.z}; }
template <std::derived_from<Val> X1, std::derived_from<Val> Y1, std::derived_from<Val> Z1, std::derived_from<Val> X2, std::derived_from<Val> Y2, std::derived_from<Val> Z2>
constexpr auto operator/(Vec3<X1, Y1, Z1> v1, Vec3<X2, Y2, Z2> v2) { return Vec3{v1.x / v2.x, v1.y / v2.y, v1.z / v2.z}; }
//---------------------------------------------------------------------------
}
//---------------------------------------------------------------------------
template <size_t Components>
auto ValScope::make(Vec xs, Vec vs, Vec ps, num t) {
    return ([&]<size_t... Is>(std::index_sequence<Is...>) {
        return std::make_tuple(ValScope{xs, vs, ps, t}, val::Pos<Is>{}..., val::Vel<Is>{}..., val::Time{});
    })(std::make_index_sequence<Components>{});
}
//---------------------------------------------------------------------------
}