#pragma once

#include "ContainerLoading/LoadingChecker.h"

#include "Model/Instance.h"

#include "Algorithms/BCRoutingParams.h"

#include <optional>

namespace VehicleRouting
{
using namespace Model;
namespace Algorithms
{
namespace Heuristics
{
namespace SetBased
{
using namespace ContainerLoading;

class SPHeuristic
{
  public:
    SPHeuristic(const Instance* const instance,
                LoadingChecker* loadingChecker,
                const InputParameters* const inputParameters,
                GRBEnv* env)
    : mEnv(env), mInstance(instance), mLoadingChecker(loadingChecker), mInputParameters(inputParameters)
    {
    }

    std::optional<Collections::SequenceVector> Run(double cutoff);
    [[nodiscard]] double GetSCObjVal() const { return mSCObjVal; };
    [[nodiscard]] double GetSPObjVal() const { return mSPObjVal; };

  private:
    GRBEnv* mEnv;
    const Instance* const mInstance;
    LoadingChecker* mLoadingChecker;
    const InputParameters* const mInputParameters;
    double mSCObjVal = 0.0;
    double mSPObjVal = 0.0;
    std::vector<double> CalcCosts(const auto& columns);
    Collections::SequenceVector CreateRoutesCustomerRemoval(auto& routes);
    void AddNewRoutes(auto& routes);
};

}
}
}
}