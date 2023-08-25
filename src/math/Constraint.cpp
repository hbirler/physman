#include "math/Constraint.hpp"
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
//---------------------------------------------------------------------------
using namespace std;
//---------------------------------------------------------------------------
namespace physman::math {
//---------------------------------------------------------------------------
const Constraint* math::Constraint::getDistance1()
/// The distance between two Vec3s must be "distance"
{
    static auto myConstraint = []() {
        auto [t, x1, v1, ox, oy, oz, expectedDist] = makeVecComponents<1, 4>();
        val::Vec3 x2{ox, oy, oz};
        auto distVec = x1 - x2;
        auto dist = (distVec * distVec).sum() - (expectedDist * expectedDist);
        return makeConstraint(dist);
    }();
    return &myConstraint;
}
//---------------------------------------------------------------------------
const Constraint* math::Constraint::getDistance2()
/// The distance between two Vec3s must be "distance"
{
    static auto myConstraint = []() {
        auto [t, x1, x2, v1, v2, expectedDist] = makeVecComponents<2, 1>();
        auto distVec = x1 - x2;
        auto dist = (distVec * distVec).sum() - (expectedDist * expectedDist);
        return makeConstraint(dist);
    }();
    return &myConstraint;
}
//---------------------------------------------------------------------------
const Constraint* math::Constraint::getFixed()
/// The distance between two Vec3s must be "distance"
{
    static auto myConstraint = []() {
        auto [t, x1, v1, x, y, z] = makeVecComponents<1, 3>();
        auto distVec = x1 - val::Vec3{x, y, z};
        auto dist = (distVec * distVec).sum();
        return makeConstraint(dist);
    }();
    return &myConstraint;
}
//---------------------------------------------------------------------------
const Constraint* math::Constraint::getSphereCollision1()
/// Sphere collision
{
    static auto myConstraint = []() {
    auto [t, x1, v1, ox, oy, oz, expectedDist] = makeVecComponents<1, 4>();
    val::Vec3 x2{ox, oy, oz};
    auto distVec = x1 - x2;
    auto dist = (distVec * distVec).sum() - (expectedDist * expectedDist);
    auto dif = val::If{[](num v) { return v < -epsilon; }, dist, dist, val::Zero{}};
    return makeConstraint(dif);
    }();
    return &myConstraint;
}
//---------------------------------------------------------------------------
const Constraint* math::Constraint::getSphereCollision2()
/// Sphere collision
{
    static auto myConstraint = []() {
        auto [t, x1, x2, v1, v2, expectedDist] = makeVecComponents<2, 1>();
        auto distVec = x1 - x2;
        auto dist = (distVec * distVec).sum() - (expectedDist * expectedDist);
        auto dif = val::If{[](num v) { return v < -epsilon; }, dist, dist, val::Zero{}};
        return makeConstraint(dif);
    }();
    return &myConstraint;
}
//---------------------------------------------------------------------------
const Constraint* math::Constraint::getPlaneCollision1()
/// Sphere collision
{
    static auto myConstraint = []() {
        auto [t, x1, v1, ux, uy, uz, expectedDist] = makeVecComponents<1, 4>();
        val::Vec3 up(ux, uy, uz);
        // Up should already be normalized
        auto dist = (up * x1).sum() - expectedDist;
        auto dif = val::If{[](num v) { return v < -epsilon; }, dist, dist, val::Zero{}};
        return makeConstraint(dif);
    }();
    return &myConstraint;
}
//---------------------------------------------------------------------------
ValScope Constraint::map(const ValScope& source, std::span<const unsigned> components, unsigned paramStart, unsigned paramCount)
// Extract specific components from larger valscope
{
    ValScope result;
    result.t = source.t;
    result.xs.resize(components.size());
    result.vs.resize(components.size());
    for (unsigned i = 0; i < components.size(); i++ ) {
        result.xs[i] = source.xs[components[i]];
        result.vs[i] = source.vs[components[i]];
    }
    result.ps = source.ps.slice(paramStart, paramCount);
    return result;
}
//---------------------------------------------------------------------------
TEST_CASE("math/Constraint") {
    using Catch::Approx;
    ValScope vs;
    vs.xs = {0.0, 0.0, 0.0, 3.0, 0.0, 0.0};
    vs.vs = {0.0, 0.0, 0.0, 2.0, 0.0, 0.0};
    vs.ps = {3.0};
    vs.t = 0.0;
    auto cs = Constraint::getDistance2();
    REQUIRE(cs->computeC(vs) == Approx(0.0));
    REQUIRE(cs->computeC_dt(vs) == Approx(12.0));
}
//---------------------------------------------------------------------------
}
//---------------------------------------------------------------------------