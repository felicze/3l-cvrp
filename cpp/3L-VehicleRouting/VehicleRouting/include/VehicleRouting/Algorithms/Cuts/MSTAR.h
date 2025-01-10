#pragma once

#include "CVRPSEPCut.h"

namespace VehicleRouting
{
using namespace Model;
namespace Algorithms
{
namespace Cuts
{
class MSTAR : public CVRPSEPCut
{
  public:
    MSTAR(const VehicleRouting::Algorithms::InputParameters* const inputParameters,
          int numberCustomers,
          int capacity,
          std::vector<double>& demand,
          const Model::Instance* const instance,
          CVRPSEPGraph* graph)
    : CVRPSEPCut(CutType::MST, inputParameters, numberCustomers, capacity, demand, instance, graph) {};

  private:
    [[nodiscard]] std::vector<Cut> FindCuts(const std::vector<std::vector<double>>& x) final;

    [[nodiscard]] std::optional<Cut> CreateCut(CnstrPointer constraint,
                                               const std::vector<std::vector<double>>& x) const final;
};

}
}
}