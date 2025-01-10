#pragma once

#include "Algorithms/Cuts/Graph.h"
#include "BaseCut.h"
#include "Model/Instance.h"

#include "cvrpsep/cnstrmgr.h"

namespace VehicleRouting
{
namespace Algorithms
{
// using namespace Model;
namespace Cuts
{
class CVRPSEPCut : public BaseCut
{
  public:
    int NumberCustomers;
    int VehicleCapacity;
    std::vector<double> Demand;
    const Model::Instance* const Instance;
    CVRPSEPGraph* Graph;
    CnstrMgrPointer CurrentCuts; // must be freed
    CnstrMgrPointer AllCuts; // must be freed

    CVRPSEPCut(const CutType type,
               const VehicleRouting::Algorithms::InputParameters* const inputParameters,
               int numberCustomers,
               int capacity,
               std::vector<double>& demand,
               const Model::Instance* const instance,
               CVRPSEPGraph* graph)
    : BaseCut(type, inputParameters),
      NumberCustomers(numberCustomers),
      VehicleCapacity(capacity),
      Demand(demand),
      Instance(instance),
      Graph(graph) {};

  private:
    [[nodiscard]] virtual std::optional<Cut> CreateCut(CnstrPointer constraint,
                                                       const std::vector<std::vector<double>>& x) const = 0;
};

}
}
}

// This must be at the bottom! Why?
// Because the derived classes cannot exist before the above base class is defined
// NOLINTBEGIN
#include "./FCI.h"
#include "./GLM.h"
#include "./MSTAR.h"
#include "./RCCut.h"
#include "./SCI.h"
// NOLINTEND