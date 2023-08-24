#include "math/Constraint.hpp"
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
//---------------------------------------------------------------------------
using namespace std;
//---------------------------------------------------------------------------
namespace physman::math {
//---------------------------------------------------------------------------
unique_ptr<Constraint> math::Constraint::getDistance(num distance)
/// The distance between two Vec3s must be "distance"
{
    auto [t, x1, x2, v1, v2] = makeVecComponents<2>();
    auto distVec = x1 - x2;
    auto dist = (distVec * distVec).sum() - (distance * distance);
    return makeConstraint<6>(dist);
}
//---------------------------------------------------------------------------
unique_ptr<Constraint> math::Constraint::getFixed(num x, num y, num z)
/// The distance between two Vec3s must be "distance"
{
    auto [t, x1, v1] = makeVecComponents<1>();
    auto distVec = x1 - val::Vec3{val::Const{x}, val::Const{y}, val::Const{z}};
    auto dist = (distVec * distVec).sum();
    return makeConstraint<6>(dist);
}
//---------------------------------------------------------------------------
unique_ptr<Constraint> math::Constraint::getSphereCollision(num radius1, num radius2)
/// Sphere collision
{
    auto [t, x1, x2, v1, v2] = makeVecComponents<2>();
    auto distVec = x1 - x2;
    auto expectedDist = radius1 + radius2;
    auto dist = (distVec * distVec).sum() - (expectedDist * expectedDist);
    auto dif = val::If{[](num v) { return v < -epsilon; }, dist, dist, val::Zero{}};
    return makeConstraint<6>(dif);
}
//---------------------------------------------------------------------------
unique_ptr<Constraint> math::Constraint::getAxisCollision(num expectedDist)
/// Sphere collision
{
    auto [t, x1, v1] = makeComponents<1>();
    auto dist = x1 - expectedDist;
    auto dif = val::If{[](num v) { return v < -epsilon; }, dist, dist * dist, val::Zero{}};
    return makeConstraint<1>(dif);
}
//---------------------------------------------------------------------------
ValScope Constraint::map(const ValScope& source, std::vector<unsigned> components)
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
    return result;
}
//---------------------------------------------------------------------------
TEST_CASE("math/Constraint") {
    using Catch::Approx;
    ValScope vs;
    vs.xs = {0.0, 0.0, 0.0, 3.0, 0.0, 0.0};
    vs.vs = {0.0, 0.0, 0.0, 2.0, 0.0, 0.0};
    vs.t = 0.0;
    auto cs = Constraint::getDistance(3.0);
    REQUIRE(cs->computeC(vs) == Approx(0.0));
    REQUIRE(cs->computeC_dt(vs) == Approx(12.0));
}
//---------------------------------------------------------------------------
}
//---------------------------------------------------------------------------