#pragma once

#include "BaseCut.h"

namespace VehicleRouting::Algorithms::Cuts
{

class CatCut final : public BaseCut
{
  public:
    explicit CatCut(const VehicleRouting::Algorithms::InputParameters* inputParameters)
    : BaseCut(CutType::CAT, inputParameters)
    {
    }

    // Virtual function
    [[nodiscard]] std::vector<Cut> FindCuts(const std::vector<std::vector<double>>& x) final;
};

}
