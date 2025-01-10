#pragma once

/// Source Code is adopted from the supplement material given by
/// Michiel A. J. uit het Broek, Albert H. Schrotenboer, Bolor Jargalsaikhan, Kees Jan Roodbergen, Leandro C. Coelho
/// (2021) Asymmetric Multidepot Vehicle Routing Problems: Valid Inequalities and a Branch-and-Cut Algorithm. Operations
/// Research 69(2):380-409. https://doi.org/10.1287/opre.2020.2033

#include "Cut.h"
#include <vector>

namespace VehicleRouting
{
namespace Algorithms
{
namespace Cuts
{

class BaseCut
{
  public:
    virtual ~BaseCut() = default;

    const Algorithms::InputParameters* const InputParameters;

    const CutType Type;

    BaseCut() = delete;

    BaseCut(const CutType type, const VehicleRouting::Algorithms::InputParameters* const inputParameters)
    : InputParameters(inputParameters), Type(type) {};

    std::vector<Cut> GetCuts(const std::vector<std::vector<double>>& x);

    // Getters
    [[nodiscard]] CutType GetType() const;

  private:
    [[nodiscard]] virtual std::vector<Cut> FindCuts(const std::vector<std::vector<double>>& x) = 0;
};

}
}
}

// This must be at the bottom! Why?
// Because the derived classes cannot exist before the above base class is defined
// NOLINTBEGIN
#include "./CATCut.h"
#include "./CVRPSEPCut.h"
#include "./DK_minus.h"
#include "./DK_plus.h"
// NOLINTEND