#pragma once

#include "Model/Instance.h"

namespace VehicleRouting
{
using namespace Model;
namespace Algorithms
{

class Evaluator
{
  public:
    static double CalculateRouteCosts(const Instance* const instance, const Collections::IdVector& route)
    {
        double costs = 0.0;

        costs += instance->Distance(instance->GetDepotId(), route.front());

        for (size_t iNode = 0; iNode < route.size() - 1; iNode++)
        {
            costs += instance->Distance(route[iNode], route[iNode + 1]);
        }

        costs += instance->Distance(route.back(), instance->GetDepotId());

        return costs;
    };

    static double CalculateRouteCosts(const Instance* const instance, const std::vector<Node>& route)
    {
        double costs = 0.0;

        costs += instance->Distance(instance->GetDepotId(), route.front().InternId);

        for (size_t iNode = 0; iNode < route.size() - 1; iNode++)
        {
            costs += instance->Distance(route[iNode].InternId, route[iNode + 1].InternId);
        }

        costs += instance->Distance(route.back().InternId, instance->GetDepotId());

        return costs;
    };

    static double CalculateInsertionCosts(const Instance* const instance, size_t tailId, size_t headId, size_t nodeId)
    {
        return instance->Distance(tailId, nodeId) + instance->Distance(nodeId, headId)
               - instance->Distance(tailId, headId);
    }

    static double CalculateSavings(const Instance* const instance, size_t tailId, size_t headId)
    {
        return instance->Distance(tailId, headId)
               - (instance->Distance(tailId, instance->GetDepotId())
                  + instance->Distance(instance->GetDepotId(), headId));
    }
};

}
}