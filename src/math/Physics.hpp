#pragma once
//---------------------------------------------------------------------------
#include "math/Constraint.hpp"
#include "math/Force.hpp"
#include <unordered_map>
//---------------------------------------------------------------------------
namespace physman::math {
//---------------------------------------------------------------------------
class Physics {
    public:
    struct MappedConstraint {
        std::unique_ptr<Constraint> constraint;
        std::vector<unsigned> components;
    };
    struct MappedForce {
        std::unique_ptr<Force> force;
        std::vector<unsigned> components;
    };

    std::vector<MappedConstraint> constraints;
    std::vector<MappedForce> forces;

    public:
    /// xs and vs
    Vec state;
    Vec ms;
    num t = 0;

    Physics(const Vec& xs, const Vec& vs, Vec ms, num t);
    ~Physics() noexcept;
    void addConstraint(MappedConstraint constraint);
    void addForce(MappedForce force);

    void step(num h);
};
//---------------------------------------------------------------------------
}
//---------------------------------------------------------------------------