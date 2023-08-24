#pragma once
//---------------------------------------------------------------------------
#include "math/Num.hpp"
#include "math/Val.hpp"
#include "math/Vec.hpp"
#include <memory>
#include <vector>
//---------------------------------------------------------------------------
namespace physman::math {
//---------------------------------------------------------------------------
struct ConstraintParam {};
//---------------------------------------------------------------------------
class Constraint {
    public:
    /// Destructor
    virtual ~Constraint() = default;
    /// Get the number of components
    virtual unsigned numComponents() const = 0;
    /// Get the result of the constraint function
    virtual num computeC(const ValScope& state) const = 0;
    /// Get the result of the constraint function's time derivative
    virtual num computeC_dt(const ValScope& state) const = 0;

    /// Jacobian. dC / dx
    virtual Vec computeJacobian(const ValScope& state) const = 0;
    /// Jacobian of the time derivative. dC' / dx
    virtual Vec computeJacobian_dt(const ValScope& state) const = 0;

    /// The distance between two Vec3s must be "distance"
    static std::unique_ptr<Constraint> getDistance(num distance);
    /// A vec3 must be fixed at a certain point
    static std::unique_ptr<Constraint> getFixed(num x, num y, num z);
    /// Sphere collision
    static std::unique_ptr<Constraint> getSphereCollision(num radius1, num radius2);
    /// Axis collision
    static std::unique_ptr<Constraint> getAxisCollision(num distance);

    template <unsigned Components>
    static auto makeComponents() {
        return ([&]<size_t... Is>(std::index_sequence<Is...>) {
            return std::make_tuple(val::Time{}, val::Pos<Is>{}..., val::Vel<Is>{}...);
        })(std::make_index_sequence<Components>{});
    }
    template <unsigned Vecs>
    static auto makeVecComponents() {
        return ([&]<size_t... Is>(std::index_sequence<Is...>) {
            return std::make_tuple(val::Time{}, val::Vec3{val::Pos<Is * 3>{}, val::Pos<Is * 3 + 1>{}, val::Pos<Is * 3 + 2>{}}..., val::Vec3{val::Vel<Is * 3>{}, val::Vel<Is * 3 + 1>{}, val::Vel<Is * 3 + 2>{}}...);
        })(std::make_index_sequence<Vecs>{});
    }

    template <unsigned Components>
    static auto makeConstraint(std::derived_from<val::Val> auto c) {
        auto c_dt = c.deriveBy(val::Time{});
        auto [J, J_dt] = ([&]<size_t... Is>(std::index_sequence<Is...>) {
            return std::make_pair(
                std::make_tuple(c.deriveBy(val::Pos<Is>{})...),
                std::make_tuple(c_dt.deriveBy(val::Pos<Is>{})...)
                );
        })(std::make_index_sequence<Components>{});

        using c_type = decltype(c);
        using c_dt_type = decltype(c_dt);
        using J_type = decltype(J);
        using J_dt_type = decltype(J_dt);

        class MyConstraint final : public Constraint {
            [[no_unique_address]] c_type c;
            [[no_unique_address]] c_dt_type c_dt;
            [[no_unique_address]] J_type J;
            [[no_unique_address]] J_dt_type J_dt;

            public:
            constexpr MyConstraint(decltype(c) c, decltype(c_dt) c_dt, decltype(J) J, decltype(J_dt) J_dt)
                : c(c), c_dt(c_dt), J(J), J_dt(J_dt) {}

            unsigned numComponents() const final { return Components; };
            num computeC(const ValScope& state) const final { return c.evaluate(state); }
            num computeC_dt(const ValScope& state) const final { return c_dt.evaluate(state); }
            Vec computeJacobian(const ValScope& state) const final {
                Vec result(Components);
                ([&]<size_t... Is>(std::index_sequence<Is...>) {
                    ((result[Is] = std::get<Is>(J).evaluate(state)), ...);
                })(std::make_index_sequence<Components>{});
                return result;
            }
            Vec computeJacobian_dt(const ValScope& state) const final {
                Vec result(Components);
                ([&]<size_t... Is>(std::index_sequence<Is...>) {
                    ((result[Is] = std::get<Is>(J).evaluate(state)), ...);
                })(std::make_index_sequence<Components>{});
                return result;
            }
        };
        return std::make_unique<MyConstraint>(c, c_dt, J, J_dt);
    }

    /// Extract specific components from larger valscope
    static ValScope map(const ValScope& source, std::vector<unsigned> components);
};
//---------------------------------------------------------------------------
}
//---------------------------------------------------------------------------