#include "math/Force.hpp"
#include "math/Val.hpp"
#include <cassert>
//---------------------------------------------------------------------------
using namespace std;
//---------------------------------------------------------------------------
namespace physman::math {
//---------------------------------------------------------------------------
std::unique_ptr<Force> Force::getGravity(num g) {
    return makeForce([g](const ValScope& state) {
        return Vec(state.xs.size(), g);
    });
}
//---------------------------------------------------------------------------
std::unique_ptr<Force> Force::getSpring3(num ks, num kd, num r) {
    return makeForce([ks, kd, r](const ValScope& state) -> Vec {
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
}
//---------------------------------------------------------------------------
}
//---------------------------------------------------------------------------