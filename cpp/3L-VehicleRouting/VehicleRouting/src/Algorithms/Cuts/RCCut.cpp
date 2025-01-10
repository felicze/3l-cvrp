#include "Algorithms/Cuts/RCCut.h"
#include "cvrpsep/capsep.h"

#include <algorithm>

namespace VehicleRouting
{
namespace Algorithms
{
namespace Cuts
{
std::vector<Cut> RCCut::FindCuts(const std::vector<std::vector<double>>& x)
{
    char integerAndFeasible = 0;
    double maxViolation = 0.0;
    const int maxCuts = InputParameters->UserCut.MaxCutsSeparate.at(Type);

    CMGR_CreateCMgr(&CurrentCuts, maxCuts);
    CMGR_CreateCMgr(&AllCuts, maxCuts);

    CAPSEP_SeparateCapCuts(NumberCustomers,
                           Demand.data(),
                           VehicleCapacity,
                           Graph->CounterEdges - 1,
                           Graph->EdgeTail.data(),
                           Graph->EdgeHead.data(),
                           Graph->RelaxedValueEdge.data(),
                           AllCuts,
                           maxCuts,
                           InputParameters->UserCut.EpsForIntegrality,
                           InputParameters->UserCut.ViolationThreshold.at(Type),
                           &integerAndFeasible,
                           &maxViolation,
                           CurrentCuts);

    std::vector<Cut> cuts;
    cuts.reserve(maxCuts);

    if ((integerAndFeasible == 0) && CurrentCuts->Size > 0
        && maxViolation > InputParameters->UserCut.ViolationThreshold.at(Type))
    {
        for (int iCut = 0; iCut < CurrentCuts->Size; ++iCut)
        {
            if (CurrentCuts->CPL[iCut]->CType != CMGR_CT_CAP)
            {
                continue;
            }
            auto cut = CreateCut(CurrentCuts->CPL[iCut], x);

            if (cut)
            {
                cuts.emplace_back(cut.value());
            }
        }

        if (cuts.size() > InputParameters->UserCut.MaxCutsAdd.at(Type))
        {
            std::ranges::sort(cuts, ViolationComparer);
        }
    }

    AllCuts->Size = 0;
    CurrentCuts->Size = 0;

    CMGR_FreeMemCMgr(&AllCuts);
    CMGR_FreeMemCMgr(&CurrentCuts);

    return cuts;
}

std::optional<Cut> RCCut::CreateCut(CnstrPointer constraint, const std::vector<std::vector<double>>& x) const
{
    Collections::IdVector selectedNodes;
    boost::dynamic_bitset<> customersInSet(NumberCustomers + 1);
    const auto& container = Instance->Vehicles[0].Containers[0];
    double totalWeight = 0;
    double totalVolume = 0;

    ////std::cout << "User cut: ";
    for (size_t iNode = 1; iNode <= static_cast<size_t>(constraint->IntListSize); ++iNode)
    {
        auto node = static_cast<size_t>(constraint->IntList[iNode]);
        selectedNodes.push_back(node);
        totalWeight += Instance->Nodes[node].TotalWeight;
        totalVolume += Instance->Nodes[node].TotalVolume;

        customersInSet.set(node);
    }

    auto r = mLoadingChecker->DetermineMinVehicles(InputParameters->BranchAndCut.EnableMinVehicleLifting,
                                                   InputParameters->BranchAndCut.MinVehicleLiftingThreshold,
                                                   container,
                                                   customersInSet,
                                                   totalWeight,
                                                   totalVolume);

    if (selectedNodes.size() <= Instance->Nodes.size() / 2.0)
    {
        auto cut = Cut(CutType::RCC);

        for (size_t i = 0; i < selectedNodes.size() - 1; ++i)
        {
            const auto nodeI = selectedNodes[i];
            for (size_t j = i + 1; j < selectedNodes.size(); ++j)
            {
                const auto nodeJ = selectedNodes[j];

                cut.AddArc(-1.0, nodeI, nodeJ, x[nodeI][nodeJ]);
                cut.AddArc(-1.0, nodeJ, nodeI, x[nodeJ][nodeI]);
            }
        }

        cut.RHS = -1.0 * static_cast<double>(selectedNodes.size() - r);
        cut.CalcViolation();

        ////logFile << "RCC-I with violation " << violation << "\n";

        if (cut.Violation <= InputParameters->UserCut.ViolationThreshold.at(Type))
        {
            return std::nullopt;
        }

        return cut;
    }
    else
    {
        auto cut = Cut(CutType::RCC);

        for (size_t iNode = 0; iNode < Instance->Nodes.size(); ++iNode)
        {
            if (customersInSet[iNode])
            {
                continue;
            }
            for (const auto jNode: selectedNodes)
            {
                cut.AddArc(1.0, iNode, jNode, x[iNode][jNode]);
            }
        }

        cut.RHS = 1.0 * r;
        cut.CalcViolation();

        ////logFile << "RCC-O with violation " << violation << "\n";

        if (cut.Violation <= InputParameters->UserCut.ViolationThreshold.at(Type))
        {
            return std::nullopt;
        }

        return cut;
    }
}

}
}
}