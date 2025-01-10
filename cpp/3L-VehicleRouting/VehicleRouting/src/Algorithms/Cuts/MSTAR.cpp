#include "Algorithms/Cuts/MSTAR.h"
#include "cvrpsep/mstarsep.h"
#include <algorithm>
#include <cstddef>

namespace VehicleRouting
{
namespace Algorithms
{
namespace Cuts
{
std::vector<Cut> MSTAR::FindCuts(const std::vector<std::vector<double>>& x)
{
    double maxViolation = 0.0;
    const int maxCuts = InputParameters->UserCut.MaxCutsSeparate.at(Type);

    CMGR_CreateCMgr(&CurrentCuts, maxCuts);
    CMGR_CreateCMgr(&AllCuts, maxCuts);

    MSTARSEP_SeparateMultiStarCuts(NumberCustomers,
                                   Demand.data(),
                                   VehicleCapacity,
                                   Graph->CounterEdges - 1,
                                   Graph->EdgeTail.data(),
                                   Graph->EdgeHead.data(),
                                   Graph->RelaxedValueEdge.data(),
                                   AllCuts,
                                   maxCuts,
                                   &maxViolation,
                                   CurrentCuts);

    std::vector<Cut> cuts;
    cuts.reserve(maxCuts);

    if (CurrentCuts->Size > 0 && maxViolation > InputParameters->UserCut.ViolationThreshold.at(Type))
    {
        for (int iCut = 0; iCut < CurrentCuts->Size; ++iCut)
        {
            if (CurrentCuts->CPL[iCut]->CType != CMGR_CT_MSTAR)
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

std::optional<Cut> MSTAR::CreateCut(CnstrPointer constraint, const std::vector<std::vector<double>>& x) const
{
    Collections::IdVector nucleus;
    boost::dynamic_bitset<> inNucleus(Instance->Nodes.size());
    nucleus.reserve(static_cast<size_t>(constraint->IntListSize));
    for (size_t iNode = 1; iNode <= static_cast<size_t>(constraint->IntListSize); ++iNode)
    {
        const auto node = static_cast<size_t>(constraint->IntList[iNode]);
        nucleus.push_back(node);
        inNucleus.set(node);
    }

    Collections::IdVector nodesNotInNucleus;
    nodesNotInNucleus.reserve(Instance->Nodes.size() - static_cast<size_t>(constraint->IntListSize));
    for (size_t iNode = 0; iNode < Instance->Nodes.size(); ++iNode)
    {
        if (!inNucleus[iNode])
        {
            nodesNotInNucleus.push_back(iNode);
        }
    }

    Collections::IdVector satellites;
    satellites.reserve(static_cast<size_t>(constraint->ExtListSize));
    for (size_t iNode = 1; iNode <= static_cast<size_t>(constraint->ExtListSize); ++iNode)
    {
        satellites.push_back(static_cast<size_t>(constraint->ExtList[iNode]));
    }

    Collections::IdVector connectors;
    connectors.reserve(static_cast<size_t>(constraint->CListSize));
    for (size_t iNode = 1; iNode <= static_cast<size_t>(constraint->CListSize); ++iNode)
    {
        connectors.push_back(static_cast<size_t>(constraint->CList[iNode]));
    }

    auto coefA = constraint->A;
    auto coefB = constraint->B;
    auto coefL = constraint->L;

    auto cut = Cut(CutType::MST);
    cut.RHS = coefL;

    for (const auto i: nucleus)
    {
        for (const auto j: nodesNotInNucleus)
        {
            cut.AddArc(coefB, i, j, x[i][j]);
            cut.AddArc(coefB, j, i, x[j][i]);
        }
    }

    for (const auto i: connectors)
    {
        for (const auto j: satellites)
        {
            cut.AddArc(-coefA, i, j, x[i][j]);
            cut.AddArc(-coefA, j, i, x[j][i]);
        }
    }

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