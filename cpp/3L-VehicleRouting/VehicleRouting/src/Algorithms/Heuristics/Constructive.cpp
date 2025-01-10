#include "Algorithms/Heuristics/Constructive.h"

#include <algorithm>

#include "Algorithms/Evaluation.h"
#include "Algorithms/LoadingInterfaceServices.h"
#include "CommonBasics/Helper/ModelServices.h"

namespace VehicleRouting
{
namespace Algorithms
{
namespace Heuristics
{
namespace Constructive
{
std::vector<Route> Savings::Run()
{
    const auto& container = mInstance->Vehicles[0].Containers[0];

    std::vector<std::tuple<double, size_t, size_t>> savingsValues;
    std::vector<Route> routes;
    std::map<size_t, size_t> routeOfNode;

    for (const auto& node: mInstance->GetCustomers())
    {
        routeOfNode[node.InternId] = routes.size();
        auto tmpSequence = Collections::IdVector{node.InternId};
        routes.emplace_back(routes.size(), tmpSequence);
        routes.back().TotalVolume = mInstance->Nodes[node.InternId].TotalVolume;
        routes.back().TotalWeight = mInstance->Nodes[node.InternId].TotalWeight;
    }

    for (const auto& nodeI: mInstance->GetCustomers())
    {
        for (const auto& nodeJ: mInstance->GetCustomers())
        {
            if (nodeI.InternId == nodeJ.InternId)
            {
                continue;
            }

            const auto savings = Evaluator::CalculateSavings(mInstance, nodeI.InternId, nodeJ.InternId);

            savingsValues.emplace_back(savings, nodeI.InternId, nodeJ.InternId);
        }
    }

    std::ranges::sort(savingsValues);

    while (!savingsValues.empty())
    {
        auto nodeI = std::get<1>(savingsValues.front());
        auto nodeJ = std::get<2>(savingsValues.front());

        auto& frontRoute = routes[routeOfNode[nodeI]];
        auto& backRoute = routes[routeOfNode[nodeJ]];

        if (frontRoute.TotalVolume + backRoute.TotalVolume <= container.Volume
            && frontRoute.TotalWeight + backRoute.TotalWeight <= container.WeightLimit)
        {
            auto frontSequence = frontRoute.Sequence; // intended copy as we modify the sequence

            bool feasiblePackingFound = ConcatRoutes(frontSequence, backRoute.Sequence, container);

            if (feasiblePackingFound)
            {
                frontRoute.Sequence = frontSequence;
                frontRoute.TotalVolume += backRoute.TotalVolume;
                frontRoute.TotalWeight += backRoute.TotalWeight;

                for (const auto id: backRoute.Sequence)
                {
                    routeOfNode[id] = routeOfNode[nodeI];
                }

                backRoute.Sequence.clear();
                backRoute.TotalVolume = 0.0;
                backRoute.TotalWeight = 0.0;

                DeleteSavings(savingsValues, nodeI, nodeJ);

                std::erase_if(savingsValues,
                              [&](const auto& savingsTuple)
                              {
                                  return (std::get<1>(savingsTuple) == frontRoute.Sequence.back()
                                          && std::get<2>(savingsTuple) == frontRoute.Sequence.front());
                              });
            }
        }

        std::erase_if(savingsValues,
                      [&](const auto& savingsTuple)
                      { return std::get<1>(savingsTuple) == nodeI && std::get<2>(savingsTuple) == nodeJ; });
    }

    std::vector<Route> startSolution;
    for (auto& route: routes)
    {
        if (route.Sequence.empty())
        {
            continue;
        }

        startSolution.emplace_back(route);
    }

    return startSolution;
}

bool Savings::ConcatRoutes(Collections::IdVector& frontSequence,
                           const Collections::IdVector& backSequence,
                           const Container& container)
{
    frontSequence.insert(std::end(frontSequence), std::begin(backSequence), std::end(backSequence));

    if (mInputParameters->ContainerLoading.LoadingProblem.LoadingFlags == LoadingFlag::NoneSet)
    {
        mLoadingChecker->AddFeasibleSequenceFromOutside(frontSequence);
        return true;
    }
    if (mLoadingChecker->RouteIsInFeasSequences(frontSequence))
    {
        return true;
    }

    auto selectedItems = InterfaceConversions::SelectItems(frontSequence, mInstance->Nodes, false);

    double maxRuntime = mInputParameters->DetermineMaxRuntime(BranchAndCutParams::CallType::Heuristic);
    auto status =
        mLoadingChecker->HeuristicCompleteCheck(container,
                                                mLoadingChecker->MakeBitset(mInstance->Nodes.size(), frontSequence),
                                                frontSequence,
                                                selectedItems,
                                                maxRuntime);

    return status == LoadingStatus::FeasOpt;
}

void Savings::DeleteSavings(std::vector<std::tuple<double, size_t, size_t>>& savingsValues,
                            size_t startNode,
                            size_t endNode)
{
    std::erase_if(savingsValues,
                  [&](const auto& savingsTuple)
                  {
                      return std::get<1>(savingsTuple) == startNode || std::get<2>(savingsTuple) == endNode
                             || (std::get<1>(savingsTuple) == endNode && std::get<2>(savingsTuple) == startNode);
                  });
}

std::vector<Route> ModifiedSavings::Run()
{
    auto startSolution = Savings(mInstance, mInputParameters, mLoadingChecker).Run();

    if (startSolution.size() > mInstance->Vehicles.size())
    {
        RepairProcedure(startSolution);
    }

    return startSolution;
}

void ModifiedSavings::RepairProcedure(std::vector<Route>& solution)
{
    std::ranges::sort(solution,
                      [](const auto& routeA, const auto& routeB) { return routeA.TotalVolume > routeB.TotalVolume; });

    Collections::IdVector notVisitedCustomers;

    for (size_t k = mInstance->Vehicles.size(); k < solution.size(); ++k)
    {
        auto& route = solution[k];
        for (auto nodeId: route.Sequence)
        {
            notVisitedCustomers.emplace_back(nodeId);
        }
    }

    solution.erase(std::begin(solution) + static_cast<int>(mInstance->Vehicles.size()), std::end(solution));

    while (!notVisitedCustomers.empty())
    {
        std::ranges::sort(notVisitedCustomers,
                          [&](const auto& idA, const auto& idB)
                          { return mInstance->Nodes[idA].TotalVolume < mInstance->Nodes[idB].TotalVolume; });

        auto nodeToInsert = notVisitedCustomers.back();

        auto insertionCosts = DetermineInsertionCostsAllRoutes(solution, nodeToInsert);

        bool nodeInserted = false;

        for (const auto& costCollection: insertionCosts)
        {
            auto& route = solution[std::get<1>(costCollection)];
            auto position = std::get<2>(costCollection);

            if (!InsertionFeasible(route, nodeToInsert, position))
            {
                continue;
            }

            notVisitedCustomers.pop_back();
            nodeInserted = true;

            break;
        }

        if (!nodeInserted)
        {
            std::uniform_int_distribution<> distrib(0, static_cast<int>(solution.size()) - 1);
            auto k = static_cast<size_t>(distrib(*mRNG));
            auto& route = solution[k];
            auto removedNodes = InsertInRandomRoute(route, nodeToInsert);

            notVisitedCustomers.pop_back();

            notVisitedCustomers.insert(std::end(notVisitedCustomers), std::begin(removedNodes), std::end(removedNodes));
        }
    }
}

std::vector<std::tuple<double, size_t, size_t>>
    ModifiedSavings::DetermineInsertionCostsAllRoutes(std::vector<Route>& solution, size_t nodeId)
{
    const auto& container = mInstance->Vehicles[0].Containers[0];
    std::vector<std::tuple<double, size_t, size_t>> insertionCosts;

    const auto& node = mInstance->Nodes[nodeId];

    for (size_t k = 0; k < solution.size(); ++k)
    {
        const auto& route = solution[k];
        if (route.TotalVolume + node.TotalVolume <= container.Volume
            && route.TotalWeight + node.TotalWeight <= container.WeightLimit)
        {
            auto newInsertionCosts = DetermineInsertionCosts(route, k, nodeId);

            insertionCosts.insert(std::end(insertionCosts), std::begin(newInsertionCosts), std::end(newInsertionCosts));
        }
    }

    std::ranges::sort(insertionCosts);

    return insertionCosts;
}

std::vector<std::tuple<double, size_t, size_t>>
    ModifiedSavings::DetermineInsertionCosts(const Route& route, size_t routeId, size_t nodeId)
{
    std::vector<std::tuple<double, size_t, size_t>> insertionCosts;

    if (route.Sequence.empty())
    {
        insertionCosts.emplace_back(0.0, routeId, 0);

        return insertionCosts;
    }

    const auto depotId = mInstance->GetDepotId();

    auto deltaCosts = Evaluator::CalculateInsertionCosts(mInstance, depotId, route.Sequence.front(), nodeId);

    insertionCosts.emplace_back(deltaCosts, routeId, depotId);

    for (size_t insertionPosition = 0; insertionPosition < route.Sequence.size() - 1; ++insertionPosition)
    {
        auto nodeA = route.Sequence[insertionPosition];
        auto nodeB = route.Sequence[insertionPosition + 1];
        deltaCosts = Evaluator::CalculateInsertionCosts(mInstance, nodeA, nodeB, nodeId);

        insertionCosts.emplace_back(deltaCosts, routeId, insertionPosition + 1);
    }

    deltaCosts = Evaluator::CalculateInsertionCosts(mInstance, route.Sequence.back(), depotId, nodeId);

    insertionCosts.emplace_back(deltaCosts, routeId, route.Sequence.size());

    std::ranges::sort(insertionCosts);

    return insertionCosts;
}

Collections::IdVector ModifiedSavings::InsertInRandomRoute(Route& route, size_t nodeToInsert)
{
    const auto& container = mInstance->Vehicles[0].Containers[0];
    auto removedNodes = Collections::IdVector();
    bool nodeInserted = false;
    const auto& node = mInstance->Nodes[nodeToInsert];

    while (!nodeInserted)
    {
        std::uniform_int_distribution<> distrib(0, static_cast<int>(route.Sequence.size() - 1));
        auto positionToRemove = distrib(*mRNG);
        auto removedNode = route.Sequence[static_cast<size_t>(positionToRemove)];

        removedNodes.push_back(removedNode);

        route.Sequence.erase(std::begin(route.Sequence) + positionToRemove);
        route.TotalVolume -= mInstance->Nodes[removedNode].TotalVolume;
        route.TotalWeight -= mInstance->Nodes[removedNode].TotalWeight;

        if (route.TotalVolume + node.TotalVolume <= container.Volume
            && route.TotalWeight + node.TotalWeight <= container.WeightLimit)
        {
            auto insertionCosts = DetermineInsertionCosts(route, 0, nodeToInsert);

            for (const auto& costCollection: insertionCosts)
            {
                auto position = std::get<2>(costCollection);

                if (!InsertionFeasible(route, nodeToInsert, position))
                {
                    continue;
                }

                nodeInserted = true;
                break;
            }
        }
    }

    return removedNodes;
}

bool ModifiedSavings::InsertionFeasible(Route& route, size_t nodeToInsert, size_t position)
{
    auto container = mInstance->Vehicles[0].Containers[0];
    auto tmpSequence = route.Sequence;

    tmpSequence.insert(std::begin(tmpSequence) + static_cast<int>(position), nodeToInsert);

    if (mInputParameters->ContainerLoading.LoadingProblem.LoadingFlags == LoadingFlag::NoneSet)
    {
        route.Sequence = tmpSequence;
        route.TotalVolume += mInstance->Nodes[nodeToInsert].TotalVolume;
        route.TotalWeight += mInstance->Nodes[nodeToInsert].TotalWeight;
        mLoadingChecker->AddFeasibleSequenceFromOutside(route.Sequence);

        return true;
    }

    if (mLoadingChecker->RouteIsInFeasSequences(tmpSequence))
    {
        route.Sequence = tmpSequence;
        route.TotalVolume += mInstance->Nodes[nodeToInsert].TotalVolume;
        route.TotalWeight += mInstance->Nodes[nodeToInsert].TotalWeight;

        return true;
    }

    auto selectedItems = InterfaceConversions::SelectItems(tmpSequence, mInstance->Nodes, false);

    double maxRuntime = mInputParameters->DetermineMaxRuntime(BranchAndCutParams::CallType::Heuristic);
    auto status =
        mLoadingChecker->HeuristicCompleteCheck(container,
                                                mLoadingChecker->MakeBitset(mInstance->Nodes.size(), tmpSequence),
                                                tmpSequence,
                                                selectedItems,
                                                maxRuntime);

    if (status != LoadingStatus::FeasOpt)
    {
        return false;
    }

    route.Sequence = tmpSequence;
    route.TotalVolume += mInstance->Nodes[nodeToInsert].TotalVolume;
    route.TotalWeight += mInstance->Nodes[nodeToInsert].TotalWeight;

    return true;
}

}
}
}
}
