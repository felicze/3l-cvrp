#include "Algorithms/Heuristics/TwoOpt.h"
#include "Algorithms/Evaluation.h"
#include "Algorithms/LoadingInterfaceServices.h"
#include "CommonBasics/Helper/ModelServices.h"

#include <algorithm>

namespace VehicleRouting
{
namespace Algorithms
{
namespace Heuristics
{
namespace Improvement
{
using namespace ContainerLoading;

void TwoOpt::Run(const Instance* const instance,
                 const InputParameters& inputParameters,
                 LoadingChecker* loadingChecker,
                 const Collections::IdVector& newRoute)
{
    if (newRoute.size() < 3)
    {
        return;
    }

    if (loadingChecker->SequenceIsCheckedTwoOpt(newRoute))
    {
        return;
    }

    Collections::IdVector tmpRoute = newRoute;
    while (true)
    {
        auto moves = DetermineMoves(instance, tmpRoute);
        auto bestMove = GetBestMove(instance, inputParameters, loadingChecker, tmpRoute, moves);
        if (bestMove.has_value())
        {
            tmpRoute = MakeBestMove(tmpRoute, bestMove.value());
            continue;
        }

        if (tmpRoute.empty())
        {
            break;
        }

        loadingChecker->AddSequenceCheckedTwoOpt(tmpRoute);
        break;
    }
}

std::vector<Move> TwoOpt::DetermineMoves(const Instance* const instance,
                                         const Collections::IdVector& route)
{
    auto routeCosts = Evaluator::CalculateRouteCosts(instance, route);

    std::vector<Move> moves = std::vector<Move>();

    for (size_t i = 0; i < route.size() - 1; ++i)
    {
        for (size_t k = i + 1; k < route.size(); ++k)
        {
            auto newRoute = CreateNewRoute(route, i, k);
            auto newCosts = Evaluator::CalculateRouteCosts(instance, newRoute);

            const double epsilon = 1e-05;
            if (newCosts < routeCosts - epsilon)
            {
                moves.emplace_back(newCosts, i, k);
            }
        }
    }

    return moves;
}

std::optional<Move> TwoOpt::GetBestMove(const Instance* const instance,
                                        const InputParameters& inputParameters,
                                        LoadingChecker* loadingChecker,
                                        const Collections::IdVector& route,
                                        std::vector<Move>& moves)
{
    if (moves.size() == 0)
    {
        return std::nullopt;
    }

    std::ranges::sort(moves);

    auto set = loadingChecker->MakeBitset(instance->Nodes.size(), route);

    for (auto& move: moves)
    {
        if (loadingChecker->Parameters.LoadingProblem.LoadingFlags == LoadingFlag::NoneSet)
        {
            return move;
        }

        auto newRoute = CreateNewRoute(route, std::get<1>(move), std::get<2>(move));

        // If lifo is disabled, feasibility of route is independent from actual sequence
        // -> move is always feasible if route is feasible
        if (!loadingChecker->Parameters.LoadingProblem.EnableLifo && loadingChecker->RouteIsInFeasSequences(route))
        {
            return move;
        }

        auto selectedItems = InterfaceConversions::SelectItems(newRoute, instance->Nodes, false);

        const auto& container = instance->Vehicles.front().Containers.front();

        double maxRuntime = inputParameters.DetermineMaxRuntime(BranchAndCutParams::CallType::Heuristic);
        auto status = loadingChecker->HeuristicCompleteCheck(container, set, newRoute, selectedItems, maxRuntime);

        if (status == LoadingStatus::FeasOpt)
        {
            return move;
        }
    }

    return std::nullopt;
}

Collections::IdVector TwoOpt::MakeBestMove(const Collections::IdVector& route, const Move& bestMove)
{
    return CreateNewRoute(route, std::get<1>(bestMove), std::get<2>(bestMove));
}

Collections::IdVector TwoOpt::CreateNewRoute(const Collections::IdVector& route, size_t i, size_t k)
{
    Collections::IdVector newRoute;
    newRoute.reserve(route.size());

    for (size_t c = 0; c < i; ++c)
    {
        newRoute.push_back(route[c]);
    }

    for (size_t c = k + 1; c-- > i;)
    {
        newRoute.push_back(route[c]);
    }

    for (size_t c = k + 1; c < route.size(); ++c)
    {
        newRoute.push_back(route[c]);
    }

    return newRoute;
}

}
}
}
}