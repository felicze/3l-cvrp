#pragma once

#include "ContainerLoading/LoadingChecker.h"

#include "CVRPSEPCut.h"

namespace VehicleRouting
{
namespace Algorithms
{
namespace Cuts
{
using namespace ContainerLoading;

struct Partition
{
  public:
    double Weight = 0.0;
    double Volume = 0.0;
    Collections::IdVector Nodes;
    boost::dynamic_bitset<> NodesInSet;

    explicit Partition(size_t numberOfNodes) : NodesInSet(numberOfNodes) {};

    void AddNode(size_t nodeId, double volume, double weight)
    {
        Weight += weight;
        Volume += volume;
        Nodes.push_back(nodeId);
        NodesInSet.set(nodeId);
    };
};

class FCI : public CVRPSEPCut
{
  public:
    FCI(const VehicleRouting::Algorithms::InputParameters* const inputParameters,
        int numberCustomers,
        int capacity,
        std::vector<double>& demand,
        const Model::Instance* const instance,
        CVRPSEPGraph* graph,
        LoadingChecker* const loadingChecker)
    : CVRPSEPCut(CutType::FC, inputParameters, numberCustomers, capacity, demand, instance, graph),
      mLoadingChecker(loadingChecker) {
          ////CMGR_CreateCMgr(&CurrentCuts, 1000);
          ////CMGR_CreateCMgr(&AllCuts, 1000);
      };

  private:
    LoadingChecker* mLoadingChecker;

    [[nodiscard]] std::vector<Cut> FindCuts(const std::vector<std::vector<double>>& x) final;

    [[nodiscard]] std::optional<Cut> CreateCut(CnstrPointer constraint,
                                               const std::vector<std::vector<double>>& x) const final;
};

}
}
}