#pragma once

#include "ContainerLoading/LoadingChecker.h"

#include "Model/Instance.h"
#include "Model/Solution.h"

#include "Cut.h"

namespace VehicleRouting
{
using namespace Model;

namespace Algorithms
{
namespace Cuts
{
using namespace ContainerLoading;
using namespace ContainerLoading::Model;

class LazyConstraintsGenerator
{
  public:
    LazyConstraintsGenerator(const Instance* const instance,
                             LoadingChecker* loadingChecker,
                             const VehicleRouting::Algorithms::InputParameters* const inputParameters,
                             CallbackTracker* callbackTracker,
                             std::vector<std::vector<double>>* xValues)
    : mInstance(instance),
      mLoadingChecker(loadingChecker),
      mInputParameters(inputParameters),
      mCallbackTracker(callbackTracker),
      mXValues(xValues){};

    std::optional<std::vector<Cut>> TwoPathInequalityLifting(const Collections::IdVector& sequence,
                                                             const boost::dynamic_bitset<>& set,
                                                             Container& container,
                                                             std::vector<Cuboid>& items);

    std::optional<std::vector<Cut>>
        RegularPathLifting(const Collections::IdVector& sequence, Container& container, std::vector<Cuboid>& items);

    std::vector<Cut> CreateRegularPathCuts(const Collections::IdVector& sequence, Container& container);

    std::vector<Cut> CreateTwoPathCuts(const Collections::IdVector& sequence,
                                       const boost::dynamic_bitset<>& set,
                                       const Container& container);

    Cut CreateConstraint(CutType type, const Collections::IdVector& sequence, int minNumberVehicles = 0);

  private:
    const Instance* const mInstance;
    LoadingChecker* mLoadingChecker;
    const VehicleRouting::Algorithms::InputParameters* const mInputParameters;
    CallbackTracker* mCallbackTracker;
    std::vector<std::vector<double>>* mXValues;

    std::optional<Collections::IdVector>
    DetermineMinimalInfeasibleSubset(const Collections::IdVector& sequence,
                                                                     boost::dynamic_bitset<>& set,
                                                                     const Container& container);

    std::optional<Collections::IdVector>
    DetermineMinimalInfeasibleSubPath(const Collections::IdVector& sequence,
                                                                      const Container& container,
                                                                      bool fromFront);

    Cut CreateSubtourEliminationConstraint(CutType type, const Collections::IdVector& sequence, int minNumberVehicles);

    Cut CreateTwoPathTailConstraint(CutType type, const Collections::IdVector& sequence);

    Cut CreateTournamentConstraint(CutType type, const Collections::IdVector& sequence);

    Cut CreateTailTournamentConstraint(CutType type, const Collections::IdVector& sequence);

    Cut CreateUndirectedInfeasiblePathConstraint(CutType type, const Collections::IdVector& sequence);

    Cut CreateUndirectedInfeasibleTailPathConstraint(CutType type, const Collections::IdVector& sequence);

    Cut CreateInfeasibleTailPathConstraint(CutType type, const Collections::IdVector&);
};

}
}
}