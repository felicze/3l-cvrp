#pragma once

#include "ContainerLoading/ProblemParameters.h"

#include <unordered_map>

// NOLINTBEGIN(readability-magic-numbers)

namespace VehicleRouting
{
namespace Algorithms
{
using namespace ContainerLoading;
using namespace ContainerLoading::Algorithms;

enum class CutType
{
    None = 0,
    RCC,
    RCI,
    RCO,
    MST,
    FC,
    SC,
    GLM,
    CAT,
    DKplus,
    DKminus,
    SEC,
    TwoPath,
    TwoPathMIS,
    TwoPathTail,
    RegularPath,
    RegularPathFront,
    RegularPathBack,
    TailTournament,
    UndirectedPath,
    UndirectedTailPath,
    InfeasibleTailPath
};

struct MIPSolverParams
{
  public:
    int Threads = 8;
    double Seed = 100;
    int EnableLazyConstraints = 1;
    int DisablePreCrush = 1;
    int CutGeneration = -1;
    int NumericFocus = 0;
    double TimeLimit = 12.0 * 3600.0;
    int MaxSolutions = std::numeric_limits<int>::max();

    MIPSolverParams() = default;
};

struct UserCutParams
{
  public:
    double MaxViolationCutLazy = 0.5;
    double EpsForIntegrality = 1e-5;

    // must be int for CVRPSEP
    std::unordered_map<CutType, int> MaxCutsSeparate = {
        {CutType::RCC, 200},
        {CutType::MST, 100},
        {CutType::GLM, 1},
        {CutType::FC, 100},
        {CutType::SC, 0},
        {CutType::DKminus, 10},
        {CutType::DKplus, 10},
        {CutType::CAT, 10},
    };

    // must be int for CVRPSEP
    std::unordered_map<CutType, int> MaxCutsAdd = {
        {CutType::RCC, 100},
        {CutType::MST, 50},
        {CutType::GLM, 1},
        {CutType::FC, 50},
        {CutType::SC, 10},
        {CutType::DKminus, 10},
        {CutType::DKplus, 10},
        {CutType::CAT, 10},
    };

    std::unordered_map<CutType, double> ViolationThreshold = {
        {CutType::RCC, 0.01},
        {CutType::MST, 0.01},
        {CutType::GLM, 0.01},
        {CutType::FC, 0.01},
        {CutType::SC, 0.01},
        {CutType::DKminus, 0.01},
        {CutType::DKplus, 0.01},
        {CutType::CAT, 0.01},
    };
};

struct BranchAndCutParams
{
  public:
    enum class StartSolutionType
    {
        None = 0,
        ModifiedSavings,
        Given,
        HardCoded
    };

    enum class CallType
    {
        None,
        Exact,
        ExactLimit,
        Heuristic,
        TwoPath,
        MinInfSet,
        RegularPath,
        MinInfPath,
        ReversePath
    };

    unsigned int CutSeparationStartNodes = 200;
    unsigned int CutSeparationMaxNodes = std::numeric_limits<unsigned int>::max();
    unsigned int CutSeparationThreshold = 100;

    bool EnableMinVehicleLifting = true;
    double MinVehicleLiftingThreshold = 0.5;

    bool ActivateIntraRouteImprovement = false;
    unsigned int IntraRouteFullEnumThreshold = 0;

    bool ActivateSetPartitioningHeuristic = true;
    unsigned int SetPartitioningHeuristicThreshold = 20;
    StartSolutionType StartSolution = StartSolutionType::None;

    std::unordered_map<CallType, double> TimeLimits = {
        {CallType::Exact, std::numeric_limits<double>::max()},
        {CallType::ExactLimit, 1.0},
        {CallType::Heuristic, 0.0},
        {CallType::TwoPath, 4.0},
        {CallType::MinInfSet, 1.0},
        {CallType::RegularPath, 1.0},
        {CallType::MinInfPath, 1.0},
        {CallType::ReversePath, 1.0},
    };
    bool ActivateHeuristic = false;
    bool ActivateMemoryManagement = false;
    bool SimpleVersion = true;
};

class InputParameters
{
  public:
    MIPSolverParams MIPSolver;
    BranchAndCutParams BranchAndCut;
    UserCutParams UserCut;
    ContainerLoadingParams ContainerLoading;

    void SetLoadingFlags() { ContainerLoading.LoadingProblem.SetFlags(); };

    [[nodiscard]] double DetermineMaxRuntime(BranchAndCutParams::CallType callType,
                                             double residualTime = std::numeric_limits<double>::max()) const
    {
        return std::min(BranchAndCut.TimeLimits.at(callType), residualTime);
    }

    [[nodiscard]] bool IsExact(BranchAndCutParams::CallType callType) const
    {
        return callType == BranchAndCutParams::CallType::Exact;
    }
};

}
}

// NOLINTEND(readability-magic-numbers)