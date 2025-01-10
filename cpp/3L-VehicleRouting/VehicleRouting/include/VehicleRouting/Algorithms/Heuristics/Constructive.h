#pragma once

#include "Algorithms/BCRoutingParams.h"
#include "ContainerLoading/LoadingChecker.h"

#include "Model/Instance.h"

#include <random>
#include <vector>

namespace VehicleRouting
{
using namespace Model;

namespace Algorithms
{
namespace Heuristics
{
namespace Constructive
{
using namespace ContainerLoading;
using namespace ContainerLoading::Model;

class Savings
{
  public:
    Savings(const Instance* const instance,
            const InputParameters* const inputParameters,
            LoadingChecker* loadingChecker)
    : mInstance(instance), mInputParameters(inputParameters), mLoadingChecker(loadingChecker){};

    std::vector<Route> Run();

  private:
    const Instance* const mInstance;
    const InputParameters* const mInputParameters;
    LoadingChecker* mLoadingChecker;

    bool ConcatRoutes(Collections::IdVector& frontSequence,
                      const Collections::IdVector& backSequence,
                      const Container& container);
    void
        DeleteSavings(std::vector<std::tuple<double, size_t, size_t>>& savingsValues, size_t startNode, size_t endNode);
};

class ModifiedSavings
{
  public:
    ModifiedSavings(const Instance* const instance,
                    const InputParameters* const inputParameters,
                    LoadingChecker* loadingChecker,
                    std::mt19937* rng)
    : mInstance(instance), mInputParameters(inputParameters), mLoadingChecker(loadingChecker), mRNG(rng){};

    std::vector<Route> Run();

  private:
    const Instance* const mInstance;
    const InputParameters* const mInputParameters;
    LoadingChecker* mLoadingChecker;
    std::mt19937* mRNG;

    void RepairProcedure(std::vector<Route>& solution);
    std::vector<std::tuple<double, size_t, size_t>> DetermineInsertionCostsAllRoutes(std::vector<Route>& solution,
                                                                                     size_t nodeId);
    std::vector<std::tuple<double, size_t, size_t>>
        DetermineInsertionCosts(const Route& route, size_t routeId, size_t nodeId);
    Collections::IdVector InsertInRandomRoute(Route& route, size_t nodeToInsert);
    bool InsertionFeasible(Route& route, size_t nodeToInsert, size_t position);
};

}
}
}
}