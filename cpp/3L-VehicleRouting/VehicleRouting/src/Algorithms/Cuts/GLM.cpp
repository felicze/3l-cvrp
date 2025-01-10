#include "Algorithms/Cuts/GLM.h"
#include "cvrpsep/glmsep.h"
#include <boost/dynamic_bitset/dynamic_bitset.hpp>

namespace VehicleRouting
{
namespace Algorithms
{
namespace Cuts
{
std::vector<Cut> GLM::FindCuts(const std::vector<std::vector<double>>& x)
{
    int customerListSize = 0;
    double violation = 0.0;
    std::vector<int> customerList(Instance->Nodes.size() + 1);

    std::vector<int> demand;
    demand.reserve(Demand.size());
    for (auto d: Demand)
    {
        demand.push_back(static_cast<int>(d));
    }

    GLMSEP_SeparateGLM(NumberCustomers,
                       demand.data(),
                       VehicleCapacity,
                       Graph->CounterEdges - 1,
                       Graph->EdgeTail.data(),
                       Graph->EdgeHead.data(),
                       Graph->RelaxedValueEdge.data(),
                       customerList.data(),
                       &customerListSize,
                       &violation);

    std::vector<Cut> cuts;

    if (violation < InputParameters->UserCut.ViolationThreshold.at(Type))
    {
        return cuts;
    }

    boost::dynamic_bitset<> inS(Instance->Nodes.size());
    Collections::IdVector selectedNodes;
    for (size_t i = 1; i <= static_cast<size_t>(customerListSize); ++i)
    {
        const auto node = static_cast<size_t>(customerList[i]);
        inS.set(node);
        selectedNodes.emplace_back(node);
    }

    Collections::IdVector notSelectedNodes;
    for (size_t iNode = 1; iNode < Instance->Nodes.size(); ++iNode)
    {
        if (!inS[iNode])
        {
            notSelectedNodes.push_back(iNode);
        }
    }

    auto cut = Cut(CutType::GLM);

    for (const auto nodeI: selectedNodes)
    {
        cut.AddArc(1.0, 0, nodeI, x[0][nodeI]);
        cut.AddArc(1.0, nodeI, 0, x[nodeI][0]);

        for (const auto nodeJ: notSelectedNodes)
        {
            cut.AddArc(1.0, nodeI, nodeJ, x[nodeI][nodeJ]);
            cut.AddArc(1.0, nodeJ, nodeI, x[nodeJ][nodeI]);
        }
    }

    double relativeDemandNucleus = 0.0;
    for (auto node: customerList)
    {
        relativeDemandNucleus +=
            Instance->Nodes[static_cast<size_t>(node)].TotalWeight / Instance->Vehicles[0].Containers[0].WeightLimit;
    }

    for (const auto nodeJ: notSelectedNodes)
    {
        for (const auto nodeI: selectedNodes)
        {
            cut.AddArc(-2.0 * Instance->Nodes[nodeJ].TotalWeight / Instance->Vehicles[0].Containers[0].WeightLimit,
                       nodeI,
                       nodeJ,
                       x[nodeI][nodeJ]);
            cut.AddArc(-2.0 * Instance->Nodes[nodeJ].TotalWeight / Instance->Vehicles[0].Containers[0].WeightLimit,
                       nodeJ,
                       nodeI,
                       x[nodeJ][nodeI]);

            ////relativeDemandOutside += Instance->Nodes[nodeJ].TotalWeight /
            /// Instance->Vehicles[0].Containers[0].WeightLimit /    * (XVars[nodeI][nodeJ] + XVars[nodeJ][nodeI]);
        }
    }

    cut.RHS = 2 * relativeDemandNucleus;

    cut.CalcViolation();

    ////addCut(lhs >= 2 * relativeDemandNucleus + 2 * relativeDemandOutside);

    cuts.emplace_back(std::move(cut));

    return cuts;
}

std::optional<Cut> GLM::CreateCut(CnstrPointer constraint, const std::vector<std::vector<double>>& x) const
{
    return {};
}

}

}
}