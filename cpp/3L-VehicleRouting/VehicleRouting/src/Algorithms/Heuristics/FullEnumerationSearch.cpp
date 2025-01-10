#include "Algorithms/Heuristics/FullEnumerationSearch.h"

#include <algorithm>

#include "Algorithms/Evaluation.h"
#include "Algorithms/LoadingInterfaceServices.h"

namespace VehicleRouting
{
namespace Algorithms
{
namespace Heuristics
{
namespace Improvement
{
void FullEnumerationSearch::Run(const Instance* const instance,
                                const InputParameters& inputParameters,
                                LoadingChecker* loadingChecker,
                                const Collections::IdVector& newRoute)
{
    auto set = loadingChecker->MakeBitset(instance->Nodes.size(), newRoute);

    auto routeCosts = Evaluator::CalculateRouteCosts(instance, newRoute);

    auto tmpRoute = newRoute;
    std::ranges::sort(tmpRoute);

    using move = std::pair<double, Collections::IdVector>;

    std::vector<move> moves;
    do
    {
        auto costs = Evaluator::CalculateRouteCosts(instance, tmpRoute);

        if (costs >= routeCosts)
        {
            continue;
        }

        moves.emplace_back(costs, tmpRoute);

    } while (std::next_permutation(std::begin(tmpRoute), std::end(tmpRoute)));

    std::ranges::sort(moves);

    for (auto& move: moves)
    {
        const auto& sequence = move.second;

        if (loadingChecker->RouteIsInFeasSequences(sequence))
        {
            break;
        }

        if (!loadingChecker->Parameters.LoadingProblem.EnableLifo && loadingChecker->RouteIsInFeasSequences(newRoute))
        {
            loadingChecker->AddFeasibleSequenceFromOutside(sequence);
            break;
        }

        auto selectedItems = InterfaceConversions::SelectItems(sequence, instance->Nodes, false);

        const auto& container = instance->Vehicles.front().Containers.front();

        double maxRuntime = inputParameters.DetermineMaxRuntime(BranchAndCutParams::CallType::Heuristic);
        auto status = loadingChecker->HeuristicCompleteCheck(container, set, sequence, selectedItems, maxRuntime);

        if (status == LoadingStatus::FeasOpt)
        {
            break;
        }

        if (!loadingChecker->Parameters.LoadingProblem.EnableLifo)
        {
            break;
        }
    }
}

}
}
}
}