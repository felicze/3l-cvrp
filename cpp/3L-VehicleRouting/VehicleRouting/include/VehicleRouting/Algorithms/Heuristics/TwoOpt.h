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

using Move = std::tuple<double, size_t, size_t>;

class TwoOpt
{
  public:
    static void Run(const Instance* instance,
                    const InputParameters& inputParameters,
                    LoadingChecker* loadingChecker,
                    const Collections::IdVector& newRoute);

  private:
    static std::vector<Move> DetermineMoves(const Instance* instance, const Collections::IdVector& route);
    static std::optional<Move> GetBestMove(const Instance* instance,
                                           const InputParameters& inputParameters,
                                           LoadingChecker* loadingChecker,
                                           const Collections::IdVector& route,
                                           std::vector<Move>& moves);
    static Collections::IdVector MakeBestMove(const Collections::IdVector& route, const Move& bestMove);

    static Collections::IdVector CreateNewRoute(const Collections::IdVector& route, size_t i, size_t k);
};

}
}
}
}