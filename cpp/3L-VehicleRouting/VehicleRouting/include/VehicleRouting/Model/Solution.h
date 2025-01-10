#pragma once

#include "Algorithms/BCRoutingParams.h"
#include "Algorithms/Evaluation.h"
#include "Helper/Timer.h"

#include "Model/Instance.h"
#include "Node.h"
#include "Vehicle.h"

#include <ranges>
#include <string>

namespace VehicleRouting
{
using namespace Algorithms;

namespace Model
{

// https://stackoverflow.com/questions/14589417/can-an-enum-class-be-converted-to-the-underlying-type
template <typename E> constexpr auto to_integral(E e) -> typename std::underlying_type<E>::type
{
    return static_cast<typename std::underlying_type<E>::type>(e);
}
// https://stackoverflow.com/questions/69762598/what-are-commonly-used-ways-to-iterate-over-an-enum-class-in-c
constexpr inline auto enum_range = [](auto front, auto back)
{
    return std::views::iota(to_integral(front), to_integral(back) + 1)
           | std::views::transform([](auto e) { return decltype(front)(e); });
};

enum class CallbackElement
{
    // Integer
    None,
    IntegerSolutions,
    DetermineRoutes,
    CheckRoutes,
    IntegerRoutes,
    SingleCustomer,
    MinNumVehicles,
    Disconnected,
    Connected,
    SingleVehicle,
    MinVehApproxInf,
    RoutePrechecked,
    RoutePrecheckedNot,
    CustCombiInf,
    CustCombiInfNot,
    HeuristicFeas,
    HeuristicInf,
    CPCheck,
    ExactLimitFeas,
    ExactLimitInf,
    ExactLimitUnk,
    TwoPathInequality,
    TwoPathInequalityNot,
    RegularPathInequality,
    RegularPathInequalityNot,
    TailPathInequality,
    ExactFeas,
    ExactInf,
    ExactInvalid,
    ReverseSequence,
    RevHeurFeas,
    RevExactFeas,
    RevExactInf,
    InfeasibleTailPathInequality,
    // Fractional
    FractionalSolutions,
    AddFracSolCuts,
    BuildGraph,
    // Heuristic
    SPHeuristic
};

class CallbackTracker
{
  public:
    std::vector<CallbackElement> IntegerElements;
    std::vector<CallbackElement> Fractional;

    std::map<CallbackElement, int> Counter;
    std::map<CallbackElement, uint64_t> Timer;

    std::map<CutType, int> CutCounter;
    std::map<CutType, uint64_t> CutTimer;

    std::map<CutType, int> LazyConstraintCounter;

    std::map<double, std::pair<double, bool>> UpperBounds;
    std::map<double, std::pair<double, double>> LowerBounds;

    int HeuristicSolution = 0;

    double LastSolutionFound = 0.0;

    CallbackTracker()
    {
        for (const auto e: enum_range(CallbackElement::IntegerSolutions, CallbackElement::RevExactInf))
        {
            Counter.insert({e, 0});
            Timer.insert({e, 0});
        }
    };

    void UpdateElement(const CallbackElement element, const uint64_t time)
    {
        Counter[element]++;
        Timer[element] += time;
    }

    void UpdateLowerBound(const double runtime, const double node, const double bound)
    {
        LowerBounds.insert({runtime, {node, bound}});
    }

    void UpdateUpperBound(const double runtime, const double node, const bool spHeur)
    {
        UpperBounds.insert({runtime, {node, spHeur}});
        LastSolutionFound = runtime;
    }
};

class Tour
{
  public:
    Node Depot;
    Model::Vehicle Vehicle;
    std::vector<Node> Route;

    Tour() = default;
    Tour(const Node& depot, const Model::Vehicle& vehicle, std::vector<Node>& route)
    : Depot(depot), Vehicle(vehicle), Route(route)
    {
    }

    Tour(const Node& depot, const Model::Vehicle& vehicle, std::vector<Node>&& route)
    : Depot(depot), Vehicle(vehicle), Route(route)
    {
    }

    std::string Print() const
    {
        std::string sequence;
        sequence.append(" ").append(std::to_string(0)).append("-");
        for (const auto& node: Route)
        {
            sequence.append(std::to_string(node.InternId)).append("-");
        }

        sequence.append(std::to_string(0)).append(" ");

        return sequence;
    };

    std::string PrintRoute() const
    {
        std::string sequence;
        for (const auto& node: Route)
        {
            sequence.append(std::to_string(node.InternId)).append(" ");
        }

        return sequence;
    };
};

class SolverStatistics
{
  public:
    double Runtime = -1.0;
    double Gap = -1.0;
    double NodeCount = -1;
    double SimplexIterationCount = -1;
    size_t DeletedArcs = 0;
    size_t InfeasibleTailPathStart = 0;
    CallbackTracker SubtourTracker;
    Helper::Timer Timer;

    SolverStatistics(double runtime,
                     double gap,
                     double nodeCount,
                     double iterCount,
                     CallbackTracker& subtourTracker,
                     Helper::Timer& timer,
                     size_t deletedArcs,
                     size_t infTailPathStart)
    : Runtime(runtime),
      Gap(gap),
      NodeCount(nodeCount),
      SimplexIterationCount(iterCount),
      DeletedArcs(deletedArcs),
      InfeasibleTailPathStart(infTailPathStart),
      SubtourTracker(subtourTracker),
      Timer(timer)
    {
    }
};

class Solution
{
  public:
    double Costs = 0.0;

    size_t NumberOfRoutes = 0;
    size_t LowerBoundVehicles = 0;

    std::vector<Tour> Tours;

    Solution() = default;

    void DetermineCosts(Instance* instance)
    {
        Costs = 0;
        for (const auto& tour: Tours)
        {
            Costs += Evaluator::CalculateRouteCosts(instance, tour.Route);
        }
    };
};

class SolutionFile
{
  public:
    VehicleRouting::InputParameters InputParameters;
    Model::SolverStatistics SolverStatistics;
    Model::Solution Solution;

    SolutionFile(VehicleRouting::InputParameters& inputParameters,
                 Model::SolverStatistics& statistics,
                 Model::Solution& solution)
    : InputParameters(inputParameters), SolverStatistics(statistics), Solution(solution)
    {
    }
};

}
}