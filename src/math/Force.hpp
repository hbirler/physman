#pragma once
//---------------------------------------------------------------------------
#include "math/Val.hpp"
#include "math/Vec.hpp"
#include <functional>
#include <memory>
//---------------------------------------------------------------------------
namespace physman::math {
//---------------------------------------------------------------------------
struct ValScope;
//---------------------------------------------------------------------------
class Force {
    public:
    /// Destructor
    virtual ~Force() = default;
    /// Get the result of the force function (how much force is applied per component)
    virtual Vec computeQ(const ValScope& state) const = 0;
    /// Get the number of parameters
    virtual unsigned numParameters() const = 0;

    /// Get constant force
    static const Force* getConstant();
    /// Get the 3d spring force
    static const Force* getSpring3();

    template <unsigned NumParameters, typename Func>
    static auto makeForce(std::integral_constant<unsigned, NumParameters>, Func&& func) {
        using FF = std::decay_t<Func>;
        class MyForce : public Force, FF {
            public:
            constexpr MyForce(FF&& f) : Func(std::move(f)) {}
            constexpr MyForce(const FF& f) : Func(f) {}
            unsigned numParameters() const final { return NumParameters; };
            Vec computeQ(const ValScope& state) const final {
                return [&]<size_t... Is>(std::index_sequence<Is...>) {
                    return Func::operator()(state, state.getParam(Is)...);
                }(std::make_index_sequence<NumParameters>{});
            }
        };
        return MyForce(std::forward<Func>(func));
    }
};
//---------------------------------------------------------------------------
}
//---------------------------------------------------------------------------