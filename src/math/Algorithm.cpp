#include "math/Algorithm.hpp"
#include <valarray>
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
//---------------------------------------------------------------------------
using namespace std;
//---------------------------------------------------------------------------
namespace physman::math {
//---------------------------------------------------------------------------
Vec Algorithm::ode(const Vec& x, num t, num h, tl::function_ref<Vec(const Vec& x, num t)> f)
// Given x' = f(x, t), x(t), h, compute x(t + h)
{
    // Do Runge Kutta
    auto k1 = h * f(x, t);
    auto k2 = h * f(x + k1 / 2, t + h / 2);
    auto k3 = h * f(x + k2 / 2, t + h / 2);
    auto k4 = h * f(x + k3, t + h);
    return x + (num{1} / 6) * k1 + (num{1} / 3) * k2 + (num{1} / 3) * k3 + (num{1} / 6) * k4;
}
//---------------------------------------------------------------------------
Vec Algorithm::solve(const Vec& b, tl::function_ref<Vec(const Vec& )> A)
// Solve Ax = b for x given a function for computing Ax
{
    auto x = b;
    Vec r = b - A(x);
    Vec d = r;
    auto rdotr = (r * r).sum();

    while (rdotr >= (num{1} / 100000)) {
        auto Ad = A(d);
        auto alpha = rdotr / (d * Ad).sum();
        x += alpha * d;
        r -= alpha * Ad;
        auto rdotrOld = rdotr;
        rdotr = (r * r).sum();
        auto beta = rdotr / rdotrOld;
        d *= beta;
        d += r;
    }

    return x;
}
//---------------------------------------------------------------------------
TEST_CASE("math/Algorithm::ode") {
    using Catch::Approx;
    // x0 = 1
    // x' = x;
    // x = e^t
    REQUIRE(Algorithm::ode({1.0}, 0.0, 0.5, [](const Vec& x, num t) {
        return x;
    })[0] == Approx(exp(0.5)).epsilon(0.1));
    // x0 = 1
    // x' = 1;
    // x = t + 1;
    REQUIRE(Algorithm::ode({1.0}, 0.0, 0.5, [](const Vec& x, num t) -> Vec {
                return {1};
            })[0] == Approx(0.5 + 1).epsilon(0.1));
}
//---------------------------------------------------------------------------
TEST_CASE("math/Algorithm::solve") {
    using Catch::Approx;
    // Fibonacci matrix
    {
        auto v = Algorithm::solve({2, 1}, [&](const Vec& x) -> Vec { return {x[0] + x[1], x[0]}; });
        REQUIRE(v[0] == Approx(1.0));
        REQUIRE(v[1] == Approx(1.0));
    }
    {
        auto v = Algorithm::solve({5, 3}, [&](const Vec& x) -> Vec { return {x[0] + x[1], x[0]}; });
        REQUIRE(v[0] == Approx(3.0));
        REQUIRE(v[1] == Approx(2.0));
    }
}
//---------------------------------------------------------------------------
}
//---------------------------------------------------------------------------