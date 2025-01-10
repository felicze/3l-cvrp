#pragma once

#include "CommonBasics/Helper/MIPServices.h"

#include "CommonBasics/Helper/ModelServices.h"
#include "ContainerLoading/LoadingChecker.h"

#include "Algorithms/Heuristics/SPHeuristic.h"
#include "Helper/Timer.h"
#include "Model/Instance.h"

#include "Cuts/BaseCut.h"
#include "Cuts/Cut.h"
#include "Cuts/Graph.h"
#include "Cuts/LazyConstraintsGenerator.h"

#include <boost/dynamic_bitset.hpp>
#include <boost/functional/hash.hpp>
#include <fstream>

namespace VehicleRouting
{
using namespace Model;

namespace Algorithms
{
using namespace Cuts;

using namespace ContainerLoading;
using namespace ContainerLoading::Model;
using namespace ContainerLoading::Algorithms;

struct Subtour
{
  public:
    bool ConnectedToDepot;
    Collections::IdVector Sequence;
    boost::dynamic_bitset<> CustomersInRoute;
    double TotalWeight = 0.0;
    double TotalVolume = 0.0;

    Subtour(bool connectedToDepot,
            Collections::IdVector& nodes,
            boost::dynamic_bitset<>& customersInRoute,
            double weight,
            double volume)
    : ConnectedToDepot(connectedToDepot),
      Sequence(nodes),
      CustomersInRoute(customersInRoute),
      TotalWeight(weight),
      TotalVolume(volume)
    {
    }
};

class SubtourCallback : public GRBCallback
{
  public:
    Model::CallbackTracker CallbackTracker;
    std::unique_ptr<Heuristics::SetBased::SPHeuristic> SPHeuristic = nullptr;

    SubtourCallback(GRBVar2D& vars,
                    const Instance* const instance,
                    LoadingChecker* loadingChecker,
                    const InputParameters* const inputParameters,
                    std::string& outputPath)
    : mVariablesX(vars),
      mInstance(instance),
      mLoadingChecker(loadingChecker),
      mInputParameters(inputParameters),
      mOutputPath(outputPath),
      mCVRPSEPGraph(mInstance->Nodes.size())
    {
        // mLogFile is closed when SubtourCallback object is destroyed
        mLogFile.open(outputPath + "/Log_Callback.log");

        auto nNodes = mInstance->Nodes.size();
        mVariableValuesX = std::vector<std::vector<double>>(nNodes, std::vector<double>(nNodes, 0));

        mLazyConstraintsGenerator = std::make_unique<LazyConstraintsGenerator>(
            mInstance, mLoadingChecker, mInputParameters, &CallbackTracker, &mVariableValuesX);

        InitializeCuts();
    }

  protected:
    GRBVar2D mVariablesX;
    const Instance* const mInstance;
    LoadingChecker* mLoadingChecker;
    const InputParameters* const mInputParameters;

    std::string mOutputPath;
    std::ofstream mLogFile;

    std::vector<Subtour> mSubtours;
    std::vector<std::vector<double>> mVariableValuesX;
    bool mSolSPheuristic = false;

    std::vector<std::shared_ptr<BaseCut>> mCutTypesFractional; // TODO.Performance: can be converted to unique_ptr

    Helper::FunctionTimer<std::chrono::microseconds> mClock;

    unsigned int mLastNodeCount = 0;
    unsigned int mCurrentNode = 0;

    unsigned long mLastSolutionCount = 0;
    double mLastTime = 0.0;
    double mBestSolutionValue = std::numeric_limits<double>::max();

    double mBestSetCoveringRelaxationValue = std::numeric_limits<double>::max();

    bool mCutAdded = false;

    std::unique_ptr<LazyConstraintsGenerator> mLazyConstraintsGenerator;
    CVRPSEPGraph mCVRPSEPGraph;

    void callback() override;

    void InitializeCuts();

    GRBLinExpr ConstructLHS(const std::vector<Arc>& arcs);
    void FillXVarValuesNode();
    void FillXVariableValuesFromSolution();

    void CheckIntegerSolution();
    void FindIntegerSubtours();
    void AddLazyConstraints(const std::vector<Cut>& lazyConstraints);

    virtual bool CheckRoutes() = 0;
    bool RouteCheckedAndFeasible(const Collections::IdVector& sequence);
    bool CustomerCombinationInfeasible(const Collections::IdVector& sequence,
                                       const boost::dynamic_bitset<>& combination);

    void CheckFractionalSolution();
    void AddFractionalCuts();
    void AddCuts(const std::vector<Cut>& cuts);

    bool SolveSetPartitioningHeuristic();
    void SetHeuristicSolution(const Collections::SequenceVector& routes);
    void InjectSolution();
};

class SubtourCallback1D : public SubtourCallback
{
  public:
    SubtourCallback1D(GRBVar2D& vars,
                      const Instance* const instance,
                      LoadingChecker* loadingChecker,
                      const InputParameters* const inputParameters,
                      std::string& outputPath)
    : SubtourCallback(vars, instance, loadingChecker, inputParameters, outputPath) {};

  private:
    bool CheckRoutes() override;
};

class SubtourCallback3D : public SubtourCallback
{
  public:
    SubtourCallback3D(GRBVar2D& vars,
                      const Instance* const instance,
                      LoadingChecker* loadingChecker,
                      const InputParameters* const inputParameters,
                      std::string& outputPath)
    : SubtourCallback(vars, instance, loadingChecker, inputParameters, outputPath) {};

  protected:
    bool CheckRoutes() override;
    LoadingStatus CheckSingleVehicleSubtour(const Subtour& subtour, Container& container);
    bool CheckRouteHeuristic(const Collections::IdVector&, Container& container, std::vector<Cuboid>& items);
    void CheckReversePath(const Collections::IdVector&, Container& container);
    virtual void AddReversePathConstraints(const Collections::IdVector& sequence,
                                           const Collections::IdVector& reverseSequence) = 0;
    virtual LoadingStatus CheckRouteExact(const Subtour& subtour, Container& container, std::vector<Cuboid>& items) = 0;
    virtual bool Lifting(const Subtour& subtour, Container& container, std::vector<Cuboid>& items) = 0;
};

class SubtourCallback3DAllSimple : public SubtourCallback3D
{
  public:
    SubtourCallback3DAllSimple(GRBVar2D& vars,
                               const Instance* const instance,
                               LoadingChecker* loadingChecker,
                               const InputParameters* const inputParameters,
                               std::string& outputPath)
    : SubtourCallback3D(vars, instance, loadingChecker, inputParameters, outputPath) {};

  private:
    bool Lifting(const Subtour& subtour [[maybe_unused]],
                 Container& container [[maybe_unused]],
                 std::vector<Cuboid>& items [[maybe_unused]]) override
    {
        return false;
    }
    LoadingStatus CheckRouteExact(const Subtour& subtour, Container& container, std::vector<Cuboid>& items) override;
    void AddReversePathConstraints(const Collections::IdVector& sequence [[maybe_unused]],
                                   const Collections::IdVector& reverseSequence [[maybe_unused]]) override
    {
    }
};

class SubtourCallback3DAll : public SubtourCallback3D
{
  public:
    SubtourCallback3DAll(GRBVar2D& vars,
                         const Instance* const instance,
                         LoadingChecker* loadingChecker,
                         const InputParameters* const inputParameters,
                         std::string& outputPath)
    : SubtourCallback3D(vars, instance, loadingChecker, inputParameters, outputPath) {};

  private:
    bool Lifting(const Subtour& subtour, Container& container, std::vector<Cuboid>& items) override;
    LoadingStatus CheckRouteExact(const Subtour& subtour, Container& container, std::vector<Cuboid>& items) override;
    void AddReversePathConstraints(const Collections::IdVector& sequence,
                                   const Collections::IdVector& reverseSequence) override;
};

class SubtourCallback3DNoSupport : public SubtourCallback3D
{
  public:
    SubtourCallback3DNoSupport(GRBVar2D& vars,
                               const Instance* const instance,
                               LoadingChecker* loadingChecker,
                               const InputParameters* const inputParameters,
                               std::string& outputPath)
    : SubtourCallback3D(vars, instance, loadingChecker, inputParameters, outputPath) {};

  private:
    bool Lifting(const Subtour& subtour, Container& container, std::vector<Cuboid>& items) override;
    LoadingStatus CheckRouteExact(const Subtour& subtour, Container& container, std::vector<Cuboid>& items) override;
    void AddReversePathConstraints(const Collections::IdVector& sequence,
                                   const Collections::IdVector& reverseSequence) override;
};

class SubtourCallback3DNoLIFO : public SubtourCallback3D
{
  public:
    SubtourCallback3DNoLIFO(GRBVar2D& vars,
                            const Instance* const instance,
                            LoadingChecker* loadingChecker,
                            const InputParameters* const inputParameters,
                            std::string& outputPath)
    : SubtourCallback3D(vars, instance, loadingChecker, inputParameters, outputPath) {};

  private:
    bool Lifting(const Subtour& subtour, Container& container, std::vector<Cuboid>& items) override;
    LoadingStatus CheckRouteExact(const Subtour& subtour, Container& container, std::vector<Cuboid>& items) override;
    void AddReversePathConstraints(const Collections::IdVector& sequence,
                                   const Collections::IdVector& reverseSequence) override;
};

class SubtourCallback3DLoadingOnly : public SubtourCallback3D
{
  public:
    SubtourCallback3DLoadingOnly(GRBVar2D& vars,
                                 const Instance* const instance,
                                 LoadingChecker* loadingChecker,
                                 const InputParameters* const inputParameters,
                                 std::string& outputPath)
    : SubtourCallback3D(vars, instance, loadingChecker, inputParameters, outputPath) {};

  private:
    bool Lifting(const Subtour& subtour, Container& container, std::vector<Cuboid>& items) override;
    LoadingStatus CheckRouteExact(const Subtour& subtour, Container& container, std::vector<Cuboid>& items) override;
    void AddReversePathConstraints(const Collections::IdVector& sequence,
                                   const Collections::IdVector& reverseSequence) override;
};

class CallbackFactory
{
  public:
    static std::unique_ptr<SubtourCallback>
        CreateCallback(ContainerLoading::LoadingProblemParams::VariantType problemVariant,
                       GRBEnv* env,
                       GRBVar2D& vars,
                       const Instance* instance,
                       LoadingChecker* loadingChecker,
                       const InputParameters* inputParameters,
                       std::string& outputPath);
};

}
}