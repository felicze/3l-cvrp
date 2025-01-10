#pragma once

#include "ContainerLoading/LoadingChecker.h"

#include "CVRPSEPCut.h"

namespace VehicleRouting
{
using namespace Model;
namespace Algorithms
{
namespace Cuts
{
using namespace ContainerLoading;

class RCCut : public CVRPSEPCut
{
  public:
    RCCut(const VehicleRouting::Algorithms::InputParameters* const inputParameters,
          int numberCustomers,
          int capacity,
          std::vector<double>& demand,
          const Model::Instance* const instance,
          CVRPSEPGraph* graph,
          LoadingChecker* const loadingChecker)
    : CVRPSEPCut(CutType::RCC, inputParameters, numberCustomers, capacity, demand, instance, graph),
      mLoadingChecker(loadingChecker){};

  private:
    LoadingChecker* mLoadingChecker;

    [[nodiscard]] std::vector<Cut> FindCuts(const std::vector<std::vector<double>>& x) final;

    [[nodiscard]] std::optional<Cut> CreateCut(CnstrPointer constraint,
                                               const std::vector<std::vector<double>>& x) const final;
};

}
}
}