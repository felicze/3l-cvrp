#include "Algorithms/Heuristics/SPHeuristic.h"

#include "Algorithms/LoadingStatus.h"
#include "CommonBasics/Algorithms/BaseModels.h"
#include "CommonBasics/Helper/ModelServices.h"

#include "Algorithms/Evaluation.h"
#include "Algorithms/LoadingInterfaceServices.h"

#include "Algorithms/Heuristics/LocalSearch.h"

namespace VehicleRouting
{
namespace Algorithms
{
namespace Heuristics
{
using namespace Improvement;

namespace SetBased
{
std::optional<Collections::SequenceVector> SPHeuristic::Run(double cutoff)
{
    auto costs = CalcCosts(mLoadingChecker->GetFeasibleRoutes());

    auto setCovering = BaseModels::SetCovering<Collections::SequenceVector>(
        mEnv, true, mLoadingChecker->GetFeasibleRoutes(), mInstance->CustomerIds, costs, mInstance->Vehicles.size());

    if (!setCovering.Solve())
    {
        return std::nullopt;
    }

    mSCObjVal = setCovering.GetObjFuncValue();

    if (mSCObjVal > cutoff)
    {
        return std::nullopt;
    }

    auto routes = setCovering.GetSelectedColumns();

    auto newRoutes = CreateRoutesCustomerRemoval(routes);

    AddNewRoutes(newRoutes);

    costs = CalcCosts(mLoadingChecker->GetFeasibleRoutes());

    auto setPartitioning = BaseModels::SetPartitioning<Collections::SequenceVector>(
        mEnv, false, mLoadingChecker->GetFeasibleRoutes(), mInstance->CustomerIds, costs, mInstance->Vehicles.size());

    if (!setPartitioning.Solve())
    {
        return std::nullopt;
    }

    mSPObjVal = setPartitioning.GetObjFuncValue();

    if (mSPObjVal > cutoff)
    {
        return std::nullopt;
    }

    auto newSolution = setPartitioning.GetSelectedColumns();

    return newSolution;
}

std::vector<double> SPHeuristic::CalcCosts(const auto& columns)
{
    auto costs = std::vector<double>();

    for (const auto& route: columns)
    {
        costs.push_back(Evaluator::CalculateRouteCosts(mInstance, route));
    }

    return costs;
}

void SPHeuristic::AddNewRoutes(auto& routes)
{
    const Container& container = mInstance->Vehicles.front().Containers.front();

    for (const auto& route: routes)
    {
        if (mInputParameters->ContainerLoading.LoadingProblem.LoadingFlags == LoadingFlag::NoneSet)
        {
            mLoadingChecker->AddFeasibleSequenceFromOutside(route);
            LocalSearch::RunIntraImprovement(mInstance, mLoadingChecker, mInputParameters, route);
            continue;
        }

        auto items = InterfaceConversions::SelectItems(route, mInstance->Nodes, false);

        double maxRuntime = mInputParameters->DetermineMaxRuntime(BranchAndCutParams::CallType::Heuristic);
        const auto status = mLoadingChecker->HeuristicCompleteCheck(
            container, mLoadingChecker->MakeBitset(mInstance->Nodes.size(), route), route, items, maxRuntime);

        if (status == LoadingStatus::Invalid)
        {
            return;
        }

        // Always check new route with IntraImprovement, even though it is infeasible
        LocalSearch::RunIntraImprovement(mInstance, mLoadingChecker, mInputParameters, route);
    }
}

Collections::SequenceVector SPHeuristic::CreateRoutesCustomerRemoval(auto& routes)
{
    std::map<size_t, Collections::SequenceVector> routesOfNode;
    for (const auto& route: routes)
    {
        for (const auto& node: route)
        {
            routesOfNode[node].emplace_back(route);
        }
    }

    Collections::SequenceVector newRoutes;
    newRoutes.reserve(100);

    for (const auto& [selectedNode, selectedRoutes]: routesOfNode)
    {
        if (selectedRoutes.size() == 1)
        {
            continue;
        }

        for (const auto& route: selectedRoutes)
        {
            if (route.size() == 1)
            {
                continue;
            }

            Collections::IdVector newRoute;
            for (const auto node: route)
            {
                if (selectedNode == node)
                {
                    continue;
                }

                newRoute.emplace_back(node);
            }

            if (mLoadingChecker->RouteIsInFeasSequences(newRoute))
            {
                continue;
            }

            newRoutes.push_back(newRoute);
        }
    }

    return newRoutes;
}

}
}
}
}