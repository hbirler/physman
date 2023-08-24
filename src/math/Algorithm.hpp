#pragma once
//---------------------------------------------------------------------------
#include "math/Num.hpp"
#include "math/Vec.hpp"
#include <tl/function_ref.hpp>
//---------------------------------------------------------------------------
namespace physman::math {
//---------------------------------------------------------------------------
class Algorithm {
    public:
    /// Given x' = f(x, t), x(t), h, compute x(t + h)
    static Vec ode(const Vec& x, num t, num h, tl::function_ref<Vec(const Vec& x, num t)> f);
    /// Solve Ax = b for x given a function for computing Ax
    static Vec solve(const Vec& b, tl::function_ref<Vec(const Vec& x)> A);
};
//---------------------------------------------------------------------------
}
//---------------------------------------------------------------------------