#include "Algorithms/Cuts/SCI.h"

#include "cvrpsep/combsep.h"
#include <algorithm>
#include <boost/dynamic_bitset/dynamic_bitset.hpp>

namespace VehicleRouting
{
namespace Algorithms
{
namespace Cuts
{
std::vector<Cut> SCI::FindCuts(const std::vector<std::vector<double>>& x)
{
    double maxViolation = 0.0;

    const int maxCuts = InputParameters->UserCut.MaxCutsSeparate.at(Type);

    CMGR_CreateCMgr(&CurrentCuts, maxCuts);

    COMBSEP_SeparateCombs(NumberCustomers,
                          (int*)Demand.data(),
                          VehicleCapacity,
                          0,
                          Graph->CounterEdges - 1,
                          Graph->EdgeTail.data(),
                          Graph->EdgeHead.data(),
                          Graph->RelaxedValueEdge.data(),
                          maxCuts,
                          &maxViolation,
                          CurrentCuts);

    std::vector<Cut> cuts;
    cuts.reserve(maxCuts);

    if (CurrentCuts->Size > 0 && maxViolation > InputParameters->UserCut.ViolationThreshold.at(Type))
    {
        for (int iCut = 0; iCut < CurrentCuts->Size; ++iCut)
        {
            if (CurrentCuts->CPL[iCut]->CType != CMGR_CT_STR_COMB)
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

    CurrentCuts->Size = 0;

    CMGR_FreeMemCMgr(&CurrentCuts);

    return cuts;
}

std::optional<Cut> SCI::CreateCut(CnstrPointer constraint, const std::vector<std::vector<double>>& x) const
{
    auto numberOfTeeth = static_cast<size_t>(constraint->Key);
    std::vector<boost::dynamic_bitset<>> inTooth(Instance->Nodes.size() + 1,
                                                 boost::dynamic_bitset<>(numberOfTeeth + 1, 0));

    auto cut = Cut(CutType::SC);

    // teeth 0 is handle
    for (size_t k = 1; k <= static_cast<size_t>(constraint->IntListSize); ++k)
    {
        const auto j = static_cast<size_t>(constraint->IntList[k]);
        inTooth[j].set(0);
    }

    for (size_t t = 1; t <= numberOfTeeth; ++t)
    {
        const auto minIdx = static_cast<size_t>(constraint->ExtList[t]);
        const auto maxIdx = (t == numberOfTeeth) ? static_cast<size_t>(constraint->ExtListSize)
                                                 : static_cast<size_t>(constraint->ExtList[t + 1]) - 1;

        for (size_t k = minIdx; k <= maxIdx; ++k)
        {
            const auto j = static_cast<size_t>(constraint->ExtList[k]);
            inTooth[j].set(t);
        }
    }

    for (size_t t = 0; t <= numberOfTeeth; ++t)
    {
        for (size_t iNode = 1; iNode < inTooth.size(); ++iNode) // nodes start numbering at 1
        {
            if (!inTooth[iNode][t]) // not in teeth t, so continue
            {
                continue;
            }

            auto nodeI = iNode == Instance->Nodes.size() ? 0 : iNode;

            // sum all edges going outside/inside the teeth
            for (size_t jNode = 1; jNode < inTooth.size(); ++jNode)
            {
                if (jNode == iNode)
                {
                    continue;
                }

                if (inTooth[jNode][t])
                {
                    continue;
                }

                auto nodeJ = jNode == Instance->Nodes.size() ? 0 : jNode;

                cut.AddArc(1.0, nodeI, nodeJ, x[nodeI][nodeJ]);
                cut.AddArc(1.0, nodeJ, nodeI, x[nodeJ][nodeI]);
            }
        }
    }

    cut.RHS = constraint->RHS;

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