#include "Algorithms/Cuts/FCI.h"
#include "cvrpsep/fcisep.h"

namespace VehicleRouting
{
namespace Algorithms
{
namespace Cuts
{
std::vector<Cut> FCI::FindCuts(const std::vector<std::vector<double>>& x)
{
    double maxViolation = 0.0;

    int const maxCuts = InputParameters->UserCut.MaxCutsSeparate.at(Type);
    CMGR_CreateCMgr(&CurrentCuts, maxCuts);
    CMGR_CreateCMgr(&AllCuts, maxCuts);

    FCISEP_SeparateFCIs(NumberCustomers,
                        Demand.data(),
                        VehicleCapacity,
                        Graph->CounterEdges - 1,
                        Graph->EdgeTail.data(),
                        Graph->EdgeHead.data(),
                        Graph->RelaxedValueEdge.data(),
                        AllCuts,
                        20000,
                        maxCuts,
                        InputParameters->UserCut.ViolationThreshold.at(Type),
                        &maxViolation,
                        CurrentCuts);

    std::vector<Cut> cuts;
    cuts.reserve(maxCuts);

    if (CurrentCuts->Size > 0 && maxViolation > InputParameters->UserCut.ViolationThreshold.at(Type))
    {
        for (size_t iCut = 0; iCut < CurrentCuts->Size; ++iCut)
        {
            if (CurrentCuts->CPL[iCut]->CType != CMGR_CT_FCI)
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

std::optional<Cut> FCI::CreateCut(CnstrPointer constraint, const std::vector<std::vector<double>>& x) const
{
    const auto& container = Instance->Vehicles[0].Containers[0];
    std::vector<Partition> partitions;
    boost::dynamic_bitset<> inFrame(Instance->Nodes.size());
    int maxIndex = 0;

    for (int subsetNr = 1; subsetNr <= constraint->ExtListSize; ++subsetNr)
    {
        Partition partition(Instance->Nodes.size());

        const auto minIndex = maxIndex + 1;
        maxIndex = minIndex + constraint->ExtList[subsetNr] - 1;

        for (int j = minIndex; j <= maxIndex; ++j)
        {
            const auto k = static_cast<size_t>(constraint->IntList[j]);
            inFrame.set(k);
            partition.AddNode(k, Instance->Nodes[k].TotalVolume, Instance->Nodes[k].TotalWeight);
        }

        partitions.emplace_back(std::move(partition));
    }

    Collections::IdVector frame;
    Collections::IdVector notInFrame;
    for (size_t i = 0; i < Instance->Nodes.size(); ++i)
    {
        if (inFrame[i])
        {
            frame.push_back(i);
        }
        else
        {
            notInFrame.push_back(i);
        }
    }

    auto cut = Cut(CutType::FC);

    // Cut set of frame
    for (const auto i: frame)
    {
        for (const auto j: notInFrame)
        {
            cut.AddArc(0.5, i, j, x[i][j]);
            cut.AddArc(0.5, j, i, x[j][i]);
        }
    }

    // Inside form for partitions
    int liftRHS = 0;
    for (const auto& partition: partitions)
    {
        if (partition.Nodes.size() == 1)
        {
            continue;
        }

        int r = mLoadingChecker->DetermineMinVehicles(InputParameters->BranchAndCut.EnableMinVehicleLifting,
                                                      InputParameters->BranchAndCut.MinVehicleLiftingThreshold,
                                                      container,
                                                      partition.NodesInSet,
                                                      partition.Weight,
                                                      partition.Volume);

        double weightRatio = container.WeightLimit > 1 ? partition.Weight / container.WeightLimit : 0.0;
        int minVehiclesWeight = static_cast<int>(std::ceil(weightRatio));

        int lifting = std::max(0, r - minVehiclesWeight);
        liftRHS += lifting;

        for (size_t iNode = 0; iNode < partition.Nodes.size() - 1; ++iNode)
        {
            auto i = partition.Nodes[iNode];
            for (size_t jNode = iNode + 1; jNode < partition.Nodes.size(); ++jNode)
            {
                auto j = partition.Nodes[jNode];
                cut.AddArc(-1.0, i, j, x[i][j]);
                cut.AddArc(-1.0, j, i, x[j][i]);
            }
        }
    }

    cut.RHS = (constraint->RHS + liftRHS) / 2.0 - frame.size();
    cut.CalcViolation();

    if (cut.Violation <= InputParameters->UserCut.ViolationThreshold.at(Type))
    {
        return std::nullopt;
    }

    return cut;
}

}
}
}