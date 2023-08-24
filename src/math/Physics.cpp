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
void Physics::addConstraint(MappedConstraint constraint) {
    constraints.push_back(move(constraint));
}
//---------------------------------------------------------------------------
void Physics::addForce(MappedForce force) {
    forces.push_back(move(force));
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
        scope.t = t;

        Vec W = 1.0 / ms;

        Vec Q(scope.xs.size());
        for (auto& f : forces) {
            auto mapped = Constraint::map(scope, f.components);
            auto forceVals = f.force->computeQ(mapped);
            for (size_t i = 0; i < f.components.size(); i++)
                Q[f.components[i]] += forceVals[i];
        }

        Vec C(constraints.size());
        Vec C_dt(constraints.size());
        SparseMatrix J(constraints.size(), scope.xs.size());
        SparseMatrix J_dt(constraints.size(), scope.xs.size());
        for (size_t i = 0; i < constraints.size(); i++) {
            auto& c = constraints[i];
            auto mapped = Constraint::map(scope, c.components);
            auto localC = c.constraint->computeC(mapped);
            auto localC_dt = c.constraint->computeC_dt(mapped);
            auto localJ = c.constraint->computeJacobian(mapped);
            auto localJ_dt = c.constraint->computeJacobian_dt(mapped);
            C[i] = localC;
            C_dt[i] = localC_dt;
            for (size_t j = 0; j < c.components.size(); j++) {
                J.add(i, c.components[j], localJ[j]);
                J_dt.add(i, c.components[j], localJ_dt[j]);
            }
        }
        num ks = 100.0;
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