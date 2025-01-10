#pragma once

#include "CommonBasics/Helper/ModelServices.h"
#include "ContainerLoading/LoadingChecker.h"

#include "Model/Instance.h"

#include "Algorithms/BCRoutingParams.h"

#include "Algorithms/Heuristics/FullEnumerationSearch.h"
#include "Algorithms/Heuristics/TwoOpt.h"

namespace VehicleRouting
{
using namespace Model;

namespace Algorithms
{
namespace Heuristics
{
namespace Improvement
{
class LocalSearch
{
  public:
    static void RunIntraImprovement(const Instance* const instance,
                                    LoadingChecker* loadingChecker,
                                    const InputParameters* inputParameters,
                                    const Collections::IdVector& sequence)
    {
        if (!inputParameters->BranchAndCut.ActivateIntraRouteImprovement)
        {
            return;
        }

        if (sequence.size() < 3)
        {
            return;
        }

        if (sequence.size() < inputParameters->BranchAndCut.IntraRouteFullEnumThreshold)
        {
            FullEnumerationSearch::Run(instance, *inputParameters, loadingChecker, sequence);
        }
        else
        {
            TwoOpt::Run(instance, *inputParameters, loadingChecker, sequence);
        }
    };
};

}
}
}
}