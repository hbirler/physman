#pragma once
//---------------------------------------------------------------------------
#include "math/Constraint.hpp"
#include "math/Force.hpp"
#include <unordered_map>
//---------------------------------------------------------------------------
namespace physman::math {
//---------------------------------------------------------------------------
class Physics {
    struct Mapping {
        unsigned componentOffset = 0;
        unsigned componentCount = 0;
        unsigned paramOffset = 0;
        unsigned paramCount = 0;
    };
    std::vector<unsigned> components;
    Vec params;
    size_t numConstraints = 0;
    size_t numForces = 0;

    std::unordered_map<const Constraint*, std::vector<Mapping>> constraints;
    std::unordered_map<const Force*, std::vector<Mapping>> forces;


    public:
    /// xs and vs
    Vec state;
    Vec ms;
    num t = 0;

    Physics(const Vec& xs, const Vec& vs, Vec ms, num t);
    ~Physics() noexcept;
    template <size_t N1, size_t N2>
    void addConstraint(const Constraint* constraint, const unsigned (&components)[N1], const num (&params)[N2]) {
        return addConstraint(constraint, std::span<const unsigned>{components, components + N1}, std::span<const num>{params, params + N2});
    }
    void addConstraint(const Constraint* constraint, std::span<const unsigned> components, std::span<const num> params);
    template <size_t N1, size_t N2>
    void addForce(const Force* force, const unsigned (&components)[N1], const num (&params)[N2]) {
        return addForce(force, std::span<const unsigned>{components, components + N1}, std::span<const num>{params, params + N2});
    }
    void addForce(const Force* force, std::span<const unsigned> components, std::span<const num> params);

    void step(num h);
};
//---------------------------------------------------------------------------
}
//---------------------------------------------------------------------------