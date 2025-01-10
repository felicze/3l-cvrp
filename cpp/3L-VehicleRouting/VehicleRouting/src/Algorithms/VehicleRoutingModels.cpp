#include "Algorithms/VehicleRoutingModels.h"
#include <boost/dynamic_bitset/dynamic_bitset.hpp>

namespace VehicleRouting
{
namespace Algorithms
{
void TwoIndexVehicleFlow::BuildModel(const std::vector<Arc>& startSolutionArcs,
                                     const std::vector<Arc>& infeasibleArcs,
                                     const std::vector<Arc>& infeasibleTailPaths)
{
    try
    {
        mModel = std::make_unique<GRBModel>(*mEnv);

        AddVariables();
        AddConstraints();
        AddObjective();

        SetInfeasibleArcs(infeasibleArcs, infeasibleTailPaths);
        SetStartSolution(startSolutionArcs);
    }
    catch (GRBException& e)
    {
        std::cout << "Error number: " << e.getErrorCode() << "\n";
        std::cout << e.getMessage() << "\n";
    }
    catch (...)
    {
        std::cout << "Error during optimization"
                  << "\n";
    }
}

void TwoIndexVehicleFlow::SetCallback(GRBCallback* callback) { mModel->setCallback(callback); }

void TwoIndexVehicleFlow::Solve(const MIPSolverParams& parameters)
{
    try
    {
        SetParameters(parameters);

        mModel->optimize();
    }
    catch (GRBException& e)
    {
        std::cout << "Error number: " << e.getErrorCode() << "\n";
        std::cout << e.getMessage() << "\n";
    }
    catch (...)
    {
        std::cout << "Error during optimization"
                  << "\n";
    }
}

std::optional<Solution> TwoIndexVehicleFlow::GetSolution()
{
    try
    {
        Solution solution = Solution();
        size_t totalNodesVisited = 0;
        boost::dynamic_bitset<> nodeVisited(mInstance->Nodes.size());
        size_t shipmentCounter = 0;

        while (totalNodesVisited < mInstance->Nodes.size() - 1)
        {
            size_t currentNodeId = mInstance->GetDepotId();
            std::optional<size_t> nextNode = std::nullopt;
            size_t startNode = mInstance->GetDepotId();
            std::vector<Node> route;

            while (nextNode != startNode)
            {
                for (const auto& node: mInstance->Nodes)
                {
                    const auto id = node.InternId;

                    constexpr auto integerThreshold = 0.5;
                    if (!nodeVisited[id] && mVariablesX[currentNodeId][id].get(GRB_DoubleAttr_X) > integerThreshold)
                    {
                        if (id != 0)
                        {
                            nodeVisited.set(id);
                            route.emplace_back(node);
                            currentNodeId = id;
                            totalNodesVisited++;
                        }

                        nextNode = id;
                        break;
                    }
                }
            }

            const Vehicle& vehicle = mInstance->Vehicles[shipmentCounter];
            solution.Tours.emplace_back(mInstance->Nodes[0], vehicle, std::move(route));

            shipmentCounter++;
        }
        return solution;
    }
    catch (GRBException& e)
    {
        std::cout << "Exception: " << e.getMessage() << "No feasible solution found!"
                  << "\n";
        return std::nullopt;
    }
}

void TwoIndexVehicleFlow::SetStartSolution(const std::vector<Arc>& startSolutionArcs)
{
    for (auto const& arc: startSolutionArcs)
    {
        mVariablesX[arc.Tail][arc.Head].set(GRB_DoubleAttr_Start, arc.Coefficient);
    }
}

void TwoIndexVehicleFlow::SetInfeasibleArcs(const std::vector<Arc>& infeasibleArcs,
                                            const std::vector<Arc>& infeasibleTailPaths)
{
    for (auto const& arc: infeasibleArcs)
    {
        mVariablesX[arc.Tail][arc.Head].set(GRB_DoubleAttr_UB, arc.Coefficient);

        ////std::cout << "Infeasible arc: " << std::to_string(arc.Head) << " -> " << std::to_string(arc.Tail) << "\n";
    }

    for (auto const& arc: infeasibleTailPaths)
    {
        mModel->addConstr(mVariablesX[arc.Tail][arc.Head] + mVariablesX[arc.Head][mInstance->GetDepotId()] <= 1.0);

        ////std::cout << "Infeasible path: " << std::to_string(arc.Head) << " -> " << std::to_string(arc.Tail) << "\n";
    }
}

void TwoIndexVehicleFlow::AddVariables()
{
    for (const auto& nodeI: mInstance->Nodes)
    {
        mVariablesX.emplace_back();
        for (const auto& nodeJ: mInstance->Nodes)
        {
            std::string name = "x_" + std::to_string(nodeI.InternId) + "_" + std::to_string(nodeJ.InternId);
            mVariablesX.back().emplace_back(mModel->addVar(0, 1, 0, GRB_BINARY, name));
        }
    }
}

void TwoIndexVehicleFlow::AddConstraints()
{
    for (const auto& nodeI: mInstance->GetCustomers())
    {
        GRBLinExpr sumXOut = 0;
        GRBLinExpr sumXIn = 0;
        for (const auto& nodeJ: mInstance->Nodes)
        {
            sumXOut += mVariablesX[nodeI.InternId][nodeJ.InternId];
            sumXIn += mVariablesX[nodeJ.InternId][nodeI.InternId];
        }

        mModel->addConstr(sumXOut, GRB_EQUAL, 1.0, "Outdegree_" + std::to_string(nodeI.InternId));
        mModel->addConstr(sumXIn, GRB_EQUAL, 1.0, "Indegree_" + std::to_string(nodeI.InternId));
    }

    GRBLinExpr sumXOutDepot = 0;
    for (const auto& node: mInstance->GetCustomers())
    {
        sumXOutDepot += mVariablesX[mInstance->GetDepotId()][node.InternId];
    }

    mModel->addConstr(
        sumXOutDepot, GRB_LESS_EQUAL, static_cast<double>(mInstance->Vehicles.size()), "Max_Outdegree_depot");
    mModel->addConstr(
        sumXOutDepot, GRB_GREATER_EQUAL, static_cast<double>(mInstance->LowerBoundVehicles), "Min_Outdegree_depot");

    for (size_t i = 0; i < mInstance->Nodes.size() - 1; ++i)
    {
        if (i == mInstance->DepotIndex)
        {
            continue;
        }
        const auto iId = mInstance->Nodes[i].InternId;
        for (size_t j = i + 1; j < mInstance->Nodes.size(); ++j)
        {
            if (j == mInstance->DepotIndex)
            {
                continue;
            }

            const auto jId = mInstance->Nodes[j].InternId;
            mModel->addConstr(mVariablesX[iId][jId] + mVariablesX[jId][iId] <= 1);
        }
    }

    for (const auto& node: mInstance->Nodes)
    {
        mVariablesX[node.InternId][node.InternId].set(GRB_DoubleAttr_UB, 0);
    }
}

void TwoIndexVehicleFlow::AddObjective()
{
    GRBLinExpr totalDistance = 0;
    for (const auto& nodeI: mInstance->Nodes)
    {
        for (const auto& nodeJ: mInstance->Nodes)
        {
            totalDistance +=
                mInstance->Distance(nodeI.InternId, nodeJ.InternId) * mVariablesX[nodeI.InternId][nodeJ.InternId];
        }
    }

    mModel->setObjective(totalDistance, GRB_MINIMIZE);
}

void TwoIndexVehicleFlow::SetParameters(const MIPSolverParams& parameters)
{
    mModel->set(GRB_IntParam_Threads, parameters.Threads);
    mModel->set(GRB_IntParam_LazyConstraints, parameters.EnableLazyConstraints);
    mModel->set(GRB_IntParam_PreCrush, parameters.DisablePreCrush);
    mModel->set(GRB_IntParam_Seed, static_cast<int>(parameters.Seed));
    mModel->set(GRB_IntParam_Cuts, parameters.CutGeneration);
    mModel->set(GRB_DoubleParam_TimeLimit, parameters.TimeLimit);
    mModel->set(GRB_IntParam_NumericFocus, parameters.NumericFocus);
    mModel->set(GRB_IntParam_SolutionLimit, parameters.MaxSolutions);
    ////mModel->set(GRB_DoubleParam_WorkLimit, 2.91);
    ////mModel->set(GRB_IntParam_OutputFlag, 0);
    ////mModel->set(GRB_IntParam_Presolve, 0);
}

}
}