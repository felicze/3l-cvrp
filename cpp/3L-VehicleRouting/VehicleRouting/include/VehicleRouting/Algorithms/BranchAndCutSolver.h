#pragma once

#include "ContainerLoading/LoadingChecker.h"

#include "Helper/Timer.h"
#include "Model/Instance.h"
#include "Model/Solution.h"

#include <fstream>
#include <iostream>
#include <random>

namespace VehicleRouting
{
using namespace Model;
using namespace Helper;

namespace Algorithms
{
using namespace ContainerLoading;
using namespace ContainerLoading::Model;

class BranchAndCutSolver
{
  public:
    BranchAndCutSolver(Instance* instance,
                       GRBEnv* env,
                       const VehicleRouting::InputParameters& inputParameters,
                       const std::string& startSolutionFolderPath,
                       const std::string& outputPath)
    : mEnv(env),
      mInstance(instance),
      mInputParameters(inputParameters),
      mStartSolutionFolderPath(startSolutionFolderPath),
      mOutputPath(outputPath)
    {
        mLogFile.open(env->get(GRB_StringParam_LogFile), std::ios::out | std::ios::app);
    }

    void Solve();

  private:
    GRBEnv* mEnv;
    Instance* mInstance;
    InputParameters mInputParameters;
    std::string mStartSolutionFolderPath;
    std::string mOutputPath;

    std::ofstream mLogFile;
    std::vector<Arc> mInfeasibleArcs;
    std::vector<Arc> mInfeasibleTailPaths;
    Collections::SequenceSet mInfeasibleCombinations;
    std::vector<Arc> mStartSolutionArcs;

    Solution mStartSolution;
    Solution mFinalSolution;

    std::mt19937 mRNG;

    std::unique_ptr<LoadingChecker> mLoadingChecker;

    void InfeasibleArcProcedure();
    void DetermineInfeasiblePaths();
    bool CheckPath(const Collections::IdVector& path, Container& container, std::vector<Cuboid>& items);
    void DetermineExtendedInfeasiblePath();
    void DetermineInfeasibleCustomerCombinations();

    Helper::Timer mTimer = Helper::Timer();

    size_t DetermineLowerBoundVehicles();

    void Initialize();
    void TestProcedure();
    void Preprocessing();
    void DeterminePackingSolution();
    void PrintSolution();

    void WriteSolutionSolutionValidator();

    void StartSolutionProcedure();
    std::vector<Route> GenerateStartSolution();
    std::vector<Route> SetGivenStartSolution();
    std::vector<Route> SetHardCodedStartSolution();
};

}
}