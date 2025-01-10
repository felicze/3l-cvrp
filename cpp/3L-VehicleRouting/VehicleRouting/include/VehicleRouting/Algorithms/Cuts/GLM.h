#pragma once

#include "CVRPSEPCut.h"

namespace VehicleRouting
{
using namespace Model;
namespace Algorithms
{
namespace Cuts
{
class GLM : public CVRPSEPCut
{
  public:
    GLM(const VehicleRouting::Algorithms::InputParameters* const inputParameters,
        int numberCustomers,
        int capacity,
        std::vector<double>& demand,
        const Model::Instance* const instance,
        CVRPSEPGraph* graph)
    : CVRPSEPCut(CutType::GLM, inputParameters, numberCustomers, capacity, demand, instance, graph){
        ////CMGR_CreateCMgr(&CurrentCuts, 1000);
        ////CMGR_CreateCMgr(&AllCuts, 1000);
    };

  private:
    [[nodiscard]] std::vector<Cut> FindCuts(const std::vector<std::vector<double>>& x) final;

    [[nodiscard]] std::optional<Cut> CreateCut(CnstrPointer constraint,
                                               const std::vector<std::vector<double>>& x) const final;
};

}
}
}