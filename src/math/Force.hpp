#pragma once
//---------------------------------------------------------------------------
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

    /// Get the gravity force
    static std::unique_ptr<Force> getGravity(num g);
    /// Get the 3d spring force
    static std::unique_ptr<Force> getSpring3(num ks, num kd, num r);

    template <typename Func>
    static std::unique_ptr<Force> makeForce(Func&& func) {
        using FF = std::decay_t<Func>;
        class MyForce : public Force, FF {
            public:
            constexpr MyForce(FF&& f) : Func(std::move(f)) {}
            constexpr MyForce(const FF& f) : Func(f) {}
            Vec computeQ(const ValScope& state) const override {
                return Func::operator()(state);
            }
        };
        return std::make_unique<MyForce>(std::forward<Func>(func));
    }
};
//---------------------------------------------------------------------------
}
//---------------------------------------------------------------------------