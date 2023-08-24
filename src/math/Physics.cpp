#include "math/Physics.hpp"
#include "math/Algorithm.hpp"
#include <cassert>
#include <unordered_map>
#include <fmt/format.h>
//---------------------------------------------------------------------------
using namespace std;
//---------------------------------------------------------------------------
namespace physman::math {
//---------------------------------------------------------------------------
Physics::Physics(const Vec& xs, const Vec& vs, Vec ms, num t) : state(Vec::concat(xs, vs)), ms(move(ms)), t(t) {
    assert(xs.size() == vs.size());
}
Physics::~Physics() noexcept = default;
//---------------------------------------------------------------------------
void Physics::addConstraint(const Constraint* constraint, std::span<const unsigned> cs, std::span<const num> ps) {
    assert(constraint->numComponents() == cs.size());
    assert(constraint->numParameters() == ps.size());
    auto& myConstraint = constraints[constraint];
    myConstraint.push_back({static_cast<unsigned>(components.size()), static_cast<unsigned>(cs.size()), static_cast<unsigned>(params.size()), static_cast<unsigned>(ps.size())});
    components.insert(components.end(), cs.begin(), cs.end());
    params.insert(params.end(), ps.begin(), ps.end());
    numConstraints++;
}
//---------------------------------------------------------------------------
void Physics::addForce(const Force* force, std::span<const unsigned> cs, std::span<const num> ps) {
    assert(force->numParameters() == ps.size());
    auto& myForce = forces[force];
    myForce.push_back({static_cast<unsigned>(components.size()), static_cast<unsigned>(cs.size()), static_cast<unsigned>(params.size()), static_cast<unsigned>(ps.size())});
    components.insert(components.end(), cs.begin(), cs.end());
    params.insert(params.end(), ps.begin(), ps.end());
    numForces++;
}
//---------------------------------------------------------------------------
struct SparseMatrix {
    vector<unordered_map<unsigned, num>> rows;
    vector<unordered_map<unsigned, num>> cols;

    SparseMatrix(size_t rows, size_t cols) : rows(rows), cols(cols) {}

    void add(unsigned row, unsigned col, num val) {
        rows[row][col] += val;
        cols[col][row] += val;
    }

    Vec dot(Vec v) {
        Vec result(rows.size());
        assert(v.size() == cols.size());
        for (size_t i = 0; i < v.size(); i++)
            for (auto [k, n] : cols[i])
                result[k] += n * v[i];
        return result;
    }
    Vec dotT(Vec v) {
        Vec result(cols.size());
        assert(v.size() == rows.size());
        for (size_t i = 0; i < v.size(); i++)
            for (auto [k, n] : rows[i])
                result[k] += n * v[i];
        return result;
    }
};
//---------------------------------------------------------------------------
void Physics::step(num h) {
    state = Algorithm::ode(state, t, h, [&](const Vec& state, num t) -> Vec {
        ValScope scope;
        scope.xs = state.slice(0, state.size() / 2);
        scope.vs = state.slice(state.size() / 2, state.size() / 2);
        scope.ps = params;
        scope.t = t;

        Vec W = 1.0 / ms;

        Vec Q(scope.xs.size());
        for (auto& [f, mappings] : forces) {
            for (auto& m : mappings) {
                auto fcomponents = span{components}.subspan(m.componentOffset, m.componentCount);
                auto mapped = Constraint::map(scope, fcomponents, m.paramOffset, m.paramCount);
                auto forceVals = f->computeQ(mapped);
                for (size_t i = 0; i < fcomponents.size(); i++)
                    Q[fcomponents[i]] += forceVals[i];
            }
        }

        Vec C(numConstraints);
        Vec C_dt(numConstraints);
        SparseMatrix J(numConstraints, scope.xs.size());
        SparseMatrix J_dt(numConstraints, scope.xs.size());
        {
            size_t i = 0;
            for (auto& [c, mappings] : constraints) {
                for (auto& m : mappings) {
                    auto ccomponents = span{components}.subspan(m.componentOffset, m.componentCount);
                    auto mapped = Constraint::map(scope, ccomponents, m.paramOffset, m.paramCount);

                    auto localC = c->computeC(mapped);
                    auto localC_dt = c->computeC_dt(mapped);
                    auto localJ = c->computeJacobian(mapped);
                    auto localJ_dt = c->computeJacobian_dt(mapped);
                    C[i] = localC;
                    C_dt[i] = localC_dt;
                    for (size_t j = 0; j < ccomponents.size(); j++) {
                        J.add(i, ccomponents[j], localJ[j]);
                        J_dt.add(i, ccomponents[j], localJ_dt[j]);
                    }
                    i++;
                }
            }
        }
        num ks = 500.0;
        num kd = 10.0;
        auto b = -J_dt.dot(scope.vs) - J.dot(W * Q) - ks * C - kd * C_dt;
        auto lamb = Algorithm::solve(b, [&](const Vec& lamb) {
            return J.dot(W * J.dotT(lamb));
        });
        auto Qhat = J.dotT(lamb);

        Vec deriv = Vec::concat(scope.vs, (Q + Qhat) * W);
        return deriv;
    });
    t += h;
}
//---------------------------------------------------------------------------
}