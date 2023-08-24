#include "math/Force.hpp"
#include "math/Val.hpp"
#include <cassert>
//---------------------------------------------------------------------------
using namespace std;
//---------------------------------------------------------------------------
namespace physman::math {
//---------------------------------------------------------------------------
const Force* Force::getConstant() {
    static auto myForce = makeForce(integral_constant<unsigned, 3>{}, [](const ValScope& state, num x, num y, num z) {
        return Vec{x, y, z};
    });
    return &myForce;
}
//---------------------------------------------------------------------------
const Force* Force::getSpring3() {
    static auto myForce = makeForce(integral_constant<unsigned, 3>{}, [](const ValScope& state, num ks, num kd, num r) -> Vec {
        assert(state.xs.size() == 6);

        auto x0 = state.xs.slice(0, 3);
        auto x1 = state.xs.slice(3, 3);
        auto v0 = state.vs.slice(0, 3);
        auto v1 = state.vs.slice(3, 3);

        auto dx = x0 - x1;
        auto dv = v0 - v1;
        auto dist = sqrt(dx * dx);
        auto dxnorm = dx / dist;

        auto f = -(ks * (dist - r) + (kd * dxnorm) * dv) * dxnorm;

        return Vec::concat(f, -f);
    });
    return &myForce;
}
//---------------------------------------------------------------------------
}
//---------------------------------------------------------------------------