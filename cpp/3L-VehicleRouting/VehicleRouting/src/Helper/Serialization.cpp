#include "Helper/Serialization.h"

#include "ContainerLoading/Algorithms/CPSolverParameters.h"
#include "ContainerLoading/Model/Container.h"
#include "ContainerLoading/ProblemParameters.h"

#include "Algorithms/BCRoutingParams.h"
#include "Model/Solution.h"

namespace VehicleRouting
{
namespace Algorithms
{

NLOHMANN_JSON_SERIALIZE_ENUM(CallbackElement,
                             {{CallbackElement::None, "None"},
                              {CallbackElement::IntegerSolutions, "IntegerSolutions"},
                              {CallbackElement::DetermineRoutes, "DetermineRoutes"},
                              {CallbackElement::IntegerRoutes, "IntegerRoutes"},
                              {CallbackElement::CheckRoutes, "CheckRoutes"},
                              {CallbackElement::Disconnected, "DisconnectedRoutes"},
                              {CallbackElement::Connected, "ConnectedRoutes"},
                              {CallbackElement::SingleCustomer, "SingleCustRoutes"},
                              {CallbackElement::MinNumVehicles, "MinNumVeh"},
                              {CallbackElement::SingleVehicle, "SingleVehicle"},
                              {CallbackElement::MinVehApproxInf, "MinVehApproxInf"},
                              {CallbackElement::RoutePrechecked, "RoutePrechecked"},
                              {CallbackElement::RoutePrecheckedNot, "RoutePrecheckedNot"},
                              {CallbackElement::CustCombiInf, "CustCombiInfeasible"},
                              {CallbackElement::CustCombiInfNot, "CustCombiInfNot"},
                              {CallbackElement::HeuristicFeas, "HeuristicFeas"},
                              {CallbackElement::HeuristicInf, "HeuristicInf"},
                              {CallbackElement::CPCheck, "CPCheck"},
                              {CallbackElement::TwoPathInequality, "TwoPath"},
                              {CallbackElement::TwoPathInequalityNot, "TwoPathNot"},
                              {CallbackElement::RegularPathInequality, "RegPath"},
                              {CallbackElement::RegularPathInequalityNot, "RegPathNot"},
                              {CallbackElement::TailPathInequality, "TailPath"},
                              {CallbackElement::ExactLimitFeas, "ExactLimitFeas"},
                              {CallbackElement::ExactLimitInf, "ExactLimitInf"},
                              {CallbackElement::ExactLimitUnk, "ExactLimitUnk"},
                              {CallbackElement::ExactFeas, "ExactFeas"},
                              {CallbackElement::ExactInf, "ExactInf"},
                              {CallbackElement::ExactInvalid, "ExactInvalid"},
                              {CallbackElement::ReverseSequence, "RevSeq"},
                              {CallbackElement::RevHeurFeas, "RevHeurFeas"},
                              {CallbackElement::RevExactFeas, "RevExactFeas"},
                              {CallbackElement::RevExactInf, "RevInf"},
                              {CallbackElement::FractionalSolutions, "CheckFracSols"},
                              {CallbackElement::AddFracSolCuts, "AddFracSolCuts"},
                              {CallbackElement::BuildGraph, "BuildGraph"},
                              {CallbackElement::InfeasibleTailPathInequality, "InfTailPath"},
                              {CallbackElement::SPHeuristic, "SPHeur"}});

NLOHMANN_JSON_SERIALIZE_ENUM(CutType,
                             {{CutType::None, "None"},
                              {CutType::RCC, "RCC"},
                              {CutType::MST, "MSTAR"},
                              {CutType::FC, "FCI"},
                              {CutType::SC, "SCI"},
                              {CutType::GLM, "GLM"},
                              {CutType::CAT, "CAT"},
                              {CutType::DKplus, "DKplus"},
                              {CutType::DKminus, "DKminus"},
                              {CutType::SEC, "SEC"},
                              {CutType::TwoPath, "TwoPath"},
                              {CutType::TwoPathMIS, "TwoPathMIS"},
                              {CutType::TwoPathTail, "TwoPathTail"},
                              {CutType::RegularPath, "RegularPath"},
                              {CutType::RegularPathFront, "RegularPathFront"},
                              {CutType::RegularPathBack, "RegularPathBack"},
                              {CutType::TailTournament, "TailTournament"},
                              {CutType::UndirectedPath, "UndirectedPath"},
                              {CutType::UndirectedTailPath, "UndirectedTailPath"},
                              {CutType::InfeasibleTailPath, "InfeasibleTailPath"}});

NLOHMANN_JSON_SERIALIZE_ENUM(BranchAndCutParams::CallType,
                             {{BranchAndCutParams::CallType::None, "None"},
                              {BranchAndCutParams::CallType::Exact, "Exact"},
                              {BranchAndCutParams::CallType::ExactLimit, "ExactLimit"},
                              {BranchAndCutParams::CallType::Heuristic, "Heuristic"},
                              {BranchAndCutParams::CallType::TwoPath, "TwoPath"},
                              {BranchAndCutParams::CallType::MinInfSet, "MinInfSet"},
                              {BranchAndCutParams::CallType::RegularPath, "RegularPath"},
                              {BranchAndCutParams::CallType::MinInfPath, "MinInfPath"},
                              {BranchAndCutParams::CallType::ReversePath, "ReversePath"}});

NLOHMANN_JSON_SERIALIZE_ENUM(BranchAndCutParams::StartSolutionType,
                             {{BranchAndCutParams::StartSolutionType::None, "None"},
                              {BranchAndCutParams::StartSolutionType::ModifiedSavings, "ModifiedSavings"},
                              {BranchAndCutParams::StartSolutionType::Given, "Given"},
                              {BranchAndCutParams::StartSolutionType::HardCoded, "HardCoded"}});
}
}

namespace ContainerLoading
{
NLOHMANN_JSON_SERIALIZE_ENUM(LoadingProblemParams::VariantType,
                             {{LoadingProblemParams::VariantType::None, "None"},
                              {LoadingProblemParams::VariantType::AllConstraints, "AllConstraints"},
                              {LoadingProblemParams::VariantType::NoFragility, "NoFragility"},
                              {LoadingProblemParams::VariantType::NoSupport, "NoSupport"},
                              {LoadingProblemParams::VariantType::NoLifo, "NoLifo"},
                              {LoadingProblemParams::VariantType::LoadingOnly, "LoadingOnly"},
                              {LoadingProblemParams::VariantType::VolumeWeightApproximation,
                               "VolumeWeightApproximation"},
                              {LoadingProblemParams::VariantType::Volume, "Volume"},
                              {LoadingProblemParams::VariantType::Weight, "Weight"}});
}
namespace ContainerLoading
{
namespace Algorithms
{

void from_json(const json& j, CPSolverParams& params)
{
    j.at("EnableCumulativeDimensions").get_to(params.EnableCumulativeDimensions);
    j.at("EnableNoOverlap2DFloor").get_to(params.EnableNoOverlap2DFloor);
    j.at("LogFlag").get_to(params.LogFlag);
    j.at("Threads").get_to(params.Threads);
    j.at("Seed").get_to(params.Seed);
}

void to_json(json& j, const CPSolverParams& params)
{
    j = json{{"EnableCumulativeDimensions", params.EnableCumulativeDimensions},
             {"EnableNoOverlap2DFloor", params.EnableNoOverlap2DFloor},
             {"LogFlag", params.LogFlag},
             {"Threads", params.Threads},
             {"Seed", params.Seed}};
}

}
}

namespace ContainerLoading
{
void from_json(const json& j, LoadingProblemParams& params)
{
    j.at("ProblemVariant").get_to(params.Variant);
    j.at("SupportArea").get_to(params.SupportArea);
}

void to_json(json& j, const LoadingProblemParams& params)
{
    j = json{{"ProblemVariant", params.Variant}, {"SupportArea", params.SupportArea}};
}
}

namespace VehicleRouting
{
namespace Algorithms
{
void from_json(const json& j, MIPSolverParams& params)
{
    j.at("Threads").get_to(params.Threads);
    j.at("Seed").get_to(params.Seed);
    j.at("EnableLazyConstraints").get_to(params.EnableLazyConstraints);
    j.at("DisablePreCrush").get_to(params.DisablePreCrush);
    j.at("CutGeneration").get_to(params.CutGeneration);
    j.at("NumericFocus").get_to(params.NumericFocus);
    j.at("TimeLimit").get_to(params.TimeLimit);
    j.at("SolutionLimit").get_to(params.MaxSolutions);
}

void to_json(json& j, const MIPSolverParams& params)
{
    j = json{{"Threads", params.Threads},
             {"Seed", params.Seed},
             {"EnableLazyConstraints", params.EnableLazyConstraints},
             {"DisablePreCrush", params.DisablePreCrush},
             {"CutGeneration", params.CutGeneration},
             {"NumericFocus", params.NumericFocus},
             {"TimeLimit", params.TimeLimit},
             {"SolutionLimit", params.MaxSolutions}};
}

void from_json(const json& j, BranchAndCutParams& params)
{
    j.at("CutSeparationStartNodes").get_to(params.CutSeparationStartNodes);
    j.at("CutSeparationMaxNodes").get_to(params.CutSeparationMaxNodes);
    j.at("CutSeparationThreshold").get_to(params.CutSeparationThreshold);
    j.at("EnableMinVehicleLifting").get_to(params.EnableMinVehicleLifting);
    j.at("MinVehicleLiftingThreshold").get_to(params.MinVehicleLiftingThreshold);
    j.at("SetPartHeurThreshold").get_to(params.SetPartitioningHeuristicThreshold);
    j.at("StartSolution").get_to(params.StartSolution);
    j.at("ActivateSetPartHeur").get_to(params.ActivateSetPartitioningHeuristic);
    j.at("ActivateIntraRouteImprovement").get_to(params.ActivateIntraRouteImprovement);
    j.at("IntraRouteFullEnumThreshold").get_to(params.IntraRouteFullEnumThreshold);
    j.at("TimeLimit").get_to(params.TimeLimits);
    j.at("ActivateHeuristic").get_to(params.ActivateHeuristic);
    j.at("ActivateMemoryManagement").get_to(params.ActivateMemoryManagement);
    j.at("SimpleVersion").get_to(params.SimpleVersion);
}

void to_json(json& j, const BranchAndCutParams& params)
{
    j = json{{"CutSeparationStartNodes", params.CutSeparationStartNodes},
             {"CutSeparationMaxNodes", params.CutSeparationMaxNodes},
             {"CutSeparationThreshold", params.CutSeparationThreshold},
             {"EnableMinVehicleLifting", params.EnableMinVehicleLifting},
             {"MinVehicleLiftingThreshold", params.MinVehicleLiftingThreshold},
             {"SetPartHeurThreshold", params.SetPartitioningHeuristicThreshold},
             {"StartSolution", params.StartSolution},
             {"ActivateSetPartHeur", params.ActivateSetPartitioningHeuristic},
             {"ActivateIntraRouteImprovement", params.ActivateIntraRouteImprovement},
             {"IntraRouteFullEnumThreshold", params.IntraRouteFullEnumThreshold},
             {"TimeLimit", params.TimeLimits},
             {"ActivateHeuristic", params.ActivateHeuristic},
             {"ActivateMemoryManagement", params.ActivateMemoryManagement},
             {"SimpleVersion", params.SimpleVersion}};
}

void from_json(const json& j, UserCutParams& params)
{
    j.at("MaxViolationCutLazy").get_to(params.MaxViolationCutLazy);
    j.at("EpsForIntegrality").get_to(params.EpsForIntegrality);
    j.at("MaxCutsSeparate").get_to(params.MaxCutsSeparate);
    j.at("MaxCutsAdd").get_to(params.MaxCutsAdd);
    j.at("ViolationThreshold").get_to(params.ViolationThreshold);
}

void to_json(json& j, const UserCutParams& params)
{
    j = json{{"MaxViolationCutLazy", params.MaxViolationCutLazy},
             {"EpsForIntegrality", params.EpsForIntegrality},
             {"MaxCutsSeparate", params.MaxCutsSeparate},
             {"MaxCutsAdd", params.MaxCutsAdd},
             {"ViolationThreshold", params.ViolationThreshold}};
}

void from_json(const json& j, InputParameters& inputParameters)
{
    j.at("LoadingProblemParams").get_to(inputParameters.ContainerLoading.LoadingProblem);
    j.at("MIPSolverParams").get_to(inputParameters.MIPSolver);
    j.at("BranchAndCutParams").get_to(inputParameters.BranchAndCut);
    j.at("UserCutParams").get_to(inputParameters.UserCut);
    j.at("CPSolverParams").get_to(inputParameters.ContainerLoading.CPSolver);
}

void to_json(json& j, const InputParameters& inputParameters)
{
    j = json{{"LoadingProblemParams", inputParameters.ContainerLoading.LoadingProblem},
             {"MIPSolverParams", inputParameters.MIPSolver},
             {"BranchAndCutParams", inputParameters.BranchAndCut},
             {"UserCutParams", inputParameters.UserCut},
             {"CPSolverParams", inputParameters.ContainerLoading.CPSolver}};
}

}
}

namespace VehicleRouting
{
namespace Helper
{
void from_json(const json& j, Helper::Timer& timer) {}

void to_json(json& j, const Helper::Timer& timer)
{
    j = json{
        {"InfeasibleArcs", timer.InfeasibleArcs.count()},
        {"LowerBoundVehicles", timer.LowerBoundVehicles.count()},
        {"StartSolution", timer.StartSolution.count()},
        {"Branch&Cut", timer.BranchAndCut.count()},
    };
}
}
}

namespace VehicleRouting
{
namespace Model
{
using namespace ContainerLoading::Model;

void from_json(const json& j, Node& node)
{
    j.at("InternId").get_to(node.InternId);
    j.at("Latitude").get_to(node.PositionY);
    j.at("Longitude").get_to(node.PositionX);
    j.at("TotalWeight").get_to(node.TotalWeight);
    j.at("Items").get_to<std::vector<Cuboid>>(node.Items);
}

void to_json(json& j, const Node& node)
{
    j = json{
        {"InternId", node.InternId},
        {"Latitude", node.PositionY},
        {"Longitude", node.PositionX},
        {"TotalWeight", node.TotalWeight},
        {"Items", node.Items},
    };
}

void from_json(const json& j, Vehicle& vehicle)
{
    j.at("InternId").get_to(vehicle.InternId);
    j.at("Volume").get_to(vehicle.Volume);
    j.at("Containers").get_to<std::vector<Container>>(vehicle.Containers);
}

void to_json(json& j, const Vehicle& vehicle)
{
    j = json{
        {"InternId", vehicle.InternId},
        {"Volume", vehicle.Volume},
        {"Containers", vehicle.Containers},
    };
}

void from_json(const json& j, Tour& tour)
{
    j.at("Depot").get_to<Node>(tour.Depot);
    j.at("Vehicle").get_to<Vehicle>(tour.Vehicle);
    j.at("Route").get_to<std::vector<Node>>(tour.Route);
}

void to_json(json& j, const Tour& tour)
{
    j = json{
        {"Depot", tour.Depot},
        {"Vehicle", tour.Vehicle},
        {"Route", tour.Route},
    };
}

void from_json(const json& j, Solution& solution) { j.at("Tours").get_to<std::vector<Tour>>(solution.Tours); }

void to_json(json& j, const Solution& solution)
{
    j = json{
        {"Tours", solution.Tours},
        {"Costs", solution.Costs},
        {"NumberRoutes", solution.NumberOfRoutes},
        {"LowerBoundVehicles", solution.LowerBoundVehicles},
    };
}

void from_json(const json& j, SolverStatistics& statistics) { j.at("Runtime").get_to(statistics.Runtime); }

void to_json(json& j, const SolverStatistics& statistics)
{
    j = json{
        {"Runtime", statistics.Runtime},
        {"Gap", statistics.Gap},
        {"SimplexIterations", statistics.SimplexIterationCount},
        {"NodeCount", statistics.NodeCount},
        {"DeletedArcs", statistics.DeletedArcs},
        {"InfTailPath", statistics.InfeasibleTailPathStart},
        {"SubtourTracker", statistics.SubtourTracker},
        {"Timer", statistics.Timer},
    };
}

void from_json(const json& j, SolutionFile& solution)
{
    j.at("InputParameters").get_to<InputParameters>(solution.InputParameters);
    ////j.at("SolverStatistics").get_to<SolverStatistics>(solution.SolverStatistics);
    j.at("Solution").get_to<Solution>(solution.Solution);
}

void to_json(json& j, const SolutionFile& solution)
{
    j = json{
        {"InputParameters", solution.InputParameters},
        ////{"SolverStatistics", solution.SolverStatistics},
        {"Solution", solution.Solution},
    };
}

void from_json(const json& j, CallbackTracker& tracker) {}

void to_json(json& j, const CallbackTracker& tracker)
{
    j = json{{"ElementCallCounter", tracker.Counter},
             {"ElementCallTime", tracker.Timer},
             {"CutCounter", tracker.CutCounter},
             {"CutTimer", tracker.CutTimer},
             {"LazyConstraintCounter", tracker.LazyConstraintCounter},
             {"UpperBoundProgress", tracker.UpperBounds},
             {"LowerBoundProgress", tracker.LowerBounds}};
}
}
}