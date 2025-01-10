#pragma once

#include "Model/Node.h"

#include "Algorithms/BCRoutingParams.h"

#include <vector>

namespace VehicleRouting
{
using namespace Model;

namespace Algorithms
{
namespace Cuts
{
class Cut
{
  public:
    std::vector<Arc> Arcs;
    double LHSValue = 0.0;
    double RHS = 0.0;
    double Violation = 0.0;
    CutType Type;

    explicit Cut(CutType type) : Type(type){};

    void AddArc(double coefficient, size_t tail, size_t head, double value)
    {
        Arcs.emplace_back(coefficient, tail, head);

        LHSValue += coefficient * value;
    };

    void CalcViolation() { Violation = RHS - LHSValue; };
};

inline bool ViolationComparer(const Cut& a, const Cut& b) { return (a.Violation > b.Violation); }

}
}
}