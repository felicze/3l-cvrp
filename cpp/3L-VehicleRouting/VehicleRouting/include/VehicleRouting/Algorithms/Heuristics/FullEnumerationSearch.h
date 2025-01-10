#pragma once

#include "ContainerLoading/LoadingChecker.h"

#include "Model/Instance.h"

#include "Algorithms/BCRoutingParams.h"

namespace VehicleRouting
{
using namespace Model;

namespace Algorithms
{
namespace Heuristics
{
namespace Improvement
{
using namespace ContainerLoading;

class FullEnumerationSearch
{
  public:
    static void Run(const Instance* instance,
                    const InputParameters& inputParameters,
                    LoadingChecker* loadingChecker,
                    const Collections::IdVector& newRoute);
};

}
}
}
}