#include "math/Val.hpp"
#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
//---------------------------------------------------------------------------
namespace physman::math::val {
//---------------------------------------------------------------------------
TEST_CASE("math/Val") {
    using Catch::Approx;
    auto [bb, x0, x1, v0, v1, t] = ValScope::make<2>({2, 3}, {4, 5}, 6);
    REQUIRE(x0.deriveBy(t).evaluate(bb) == Approx(4));
    REQUIRE((x0 * x0).deriveBy(x0).evaluate(bb) == Approx(2 * 2));
    REQUIRE((x0 / x1).deriveBy(x0).evaluate(bb) == Approx(1.0 / 3));
    REQUIRE((x0 / x1).deriveBy(x1).evaluate(bb) == Approx(-2.0 / 9));
}
//---------------------------------------------------------------------------
}