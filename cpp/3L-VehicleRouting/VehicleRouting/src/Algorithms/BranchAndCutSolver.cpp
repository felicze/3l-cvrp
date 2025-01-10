#include "Algorithms/BranchAndCutSolver.h"

#include "Algorithms/Evaluation.h"
#include "CommonBasics/Helper/ModelServices.h"
#include "ContainerLoading/Algorithms/CPSolverParameters.h"
#include "ContainerLoading/Algorithms/LoadingStatus.h"
#include "ContainerLoading/Helper/HelperIO.h"

#include "Algorithms/Heuristics/Constructive.h"
#include "Algorithms/Heuristics/LocalSearch.h"
#include "Algorithms/Heuristics/SPHeuristic.h"
#include "Helper/HelperIO.h"
#include "Helper/Serialization.h"
#include "Model/Instance.h"
#include "Model/Solution.h"

#include "Algorithms/LoadingInterfaceServices.h"
#include "Algorithms/SubtourCallback.h"
#include "Algorithms/VehicleRoutingModels.h"

#include <cstdint>
#include <memory>

namespace VehicleRouting
{
using namespace Model;

namespace Algorithms
{
namespace CLP = ContainerLoading;
using namespace ContainerLoading;
using namespace ContainerLoading::Algorithms;
using namespace Heuristics::Improvement;
using namespace Helper;

void BranchAndCutSolver::Initialize()
{
    mLogFile << "ProblemVariant: " << (int)mInputParameters.ContainerLoading.LoadingProblem.Variant << "\n";

    std::vector<Container> containers;
    containers.reserve(mInstance->Vehicles.size());
    for (const auto& vehicle: mInstance->Vehicles)
    {
        containers.emplace_back(vehicle.Containers[0]);
    }

    std::vector<Group> customerNodes;
    customerNodes.reserve(mInstance->GetCustomers().size());
    for (const auto& node: mInstance->GetCustomers())
    {
        customerNodes.emplace_back(node.InternId,
                                   node.ExternId,
                                   node.PositionX,
                                   node.PositionY,
                                   node.TotalWeight,
                                   node.TotalVolume,
                                   node.TotalArea,
                                   node.Items);
    }

    mLoadingChecker = std::make_unique<LoadingChecker>(mInputParameters.ContainerLoading);
    mLoadingChecker->SetBinPackingModel(mEnv, containers, customerNodes, mOutputPath);

    for (const auto& customer: mInstance->GetCustomers())
    {
        Collections::IdVector route = {customer.InternId};

        auto items = InterfaceConversions::SelectItems(route, mInstance->Nodes, false);

        auto heurStatus = mLoadingChecker->PackingHeuristic(PackingType::Complete, containers[0], route, items);

        if (heurStatus == LoadingStatus::FeasOpt)
        {
            continue;
        }

        auto exactStatus =
            mLoadingChecker->ConstraintProgrammingSolver(PackingType::Complete,
                                                         containers[0],
                                                         mLoadingChecker->MakeBitset(mInstance->Nodes.size(), route),
                                                         route,
                                                         items,
                                                         mInputParameters.IsExact(BranchAndCutParams::CallType::Exact));

        if (exactStatus != LoadingStatus::FeasOpt)
        {
            mLogFile << "Single customer route with " << customer.InternId << "is infeasible.\n";

            auto relStatus = mLoadingChecker->ConstraintProgrammingSolver(
                PackingType::NoSupport,
                containers[0],
                mLoadingChecker->MakeBitset(mInstance->Nodes.size(), route),
                route,
                items,
                mInputParameters.IsExact(BranchAndCutParams::CallType::Exact));

            if (relStatus != LoadingStatus::FeasOpt)
            {
                throw std::runtime_error("Single customer route is infeasible!");
            }
        }
    }

    mRNG.seed(1008);
}

void BranchAndCutSolver::TestProcedure()
{
    LoadingFlag mask = LoadingFlag::NoOverlap | LoadingFlag::Fragility | LoadingFlag::Lifo | LoadingFlag::Support;

    if (!IsSet(mask, LoadingFlag::Sequence))
    {
        mLogFile << "No Sequence";
    }

    /*
    std::vector<int> sequence = {12, 11, 13, 8, 5, 4, 21};
    std::vector<Group> nodes;
    for (int id = 0; id < VRPInstance->Nodes.size(); ++id)
    {
        nodes.emplace_back(VRPInstance->Nodes[id]);
    }

    for (int i = 0; i < 1000; ++i)
    {
        auto items = mLoadingChecker->SelectItems(sequence, nodes, false);

        auto status = mLoadingChecker->ConstraintProgrammingSolver(
            PackingType::NoSupportNoSequenceNoFragility,
            VRPInstance->Vehicles.front().Containers.front(),
            sequence,
            items,
            CPPackingParams::Type::TwoPath);
        mLogFile << (int)status << "\n";
    }

    std::vector<int> sequence = {6, 11, 19, 10, 12, 16};

    std::vector<Group> nodes;
    for (int id = 0; id < VRPInstance->Nodes.size(); ++id)
    {
        nodes.emplace_back(VRPInstance->Nodes[id]);
    }

    auto items = mLoadingChecker->SelectItems(sequence, nodes, false);

    for (int i = 0; i < 0; ++i)
    {
        auto status = mLoadingChecker->ConstraintProgrammingSolver(
            PackingType::NoSupportNoSequenceNoFragility,
            VRPInstance->Vehicles.front().Containers.front(),
            sequence.size(),
            items,
            CPPackingParams::Type::TwoPath);
        mLogFile << "Status" << (int)status << "\n";
    }
    */
}

void BranchAndCutSolver::Preprocessing()
{
    mLogFile << "### START PREPROCESSING ###"
             << "\n";
    switch (mInputParameters.ContainerLoading.LoadingProblem.Variant)
    {
        case CLP::LoadingProblemParams::VariantType::Weight:
        {
            for (auto& node: mInstance->Nodes)
            {
                node.TotalVolume = 0.0;
                for (auto& item: node.Items)
                {
                    item.Volume = 0.0;
                }
            }

            for (auto& vehicle: mInstance->Vehicles)
            {
                for (auto& container: vehicle.Containers)
                {
                    container.Volume = 0.0;
                }
            }

            break;
        }
        case CLP::LoadingProblemParams::VariantType::Volume:
        {
            for (auto& node: mInstance->Nodes)
            {
                node.TotalWeight = 0.0;
                for (auto& item: node.Items)
                {
                    item.Weight = 0.0;
                }
            }

            for (auto& vehicle: mInstance->Vehicles)
            {
                for (auto& container: vehicle.Containers)
                {
                    container.WeightLimit = 0.0;
                }
            }

            break;
        }
        default:
            break;
    };

    InfeasibleArcProcedure();

    mInstance->LowerBoundVehicles = DetermineLowerBoundVehicles();

    StartSolutionProcedure();

    mLogFile << "### END PREPROCESSING ###\n";
}

void BranchAndCutSolver::StartSolutionProcedure()
{
    mLogFile << "## Start start solution procedure ##\n";

    using enum BranchAndCutParams::StartSolutionType;

    std::chrono::time_point<std::chrono::system_clock> start;
    start = std::chrono::system_clock::now();

    std::vector<Route> startRoutes;

    switch (mInputParameters.BranchAndCut.StartSolution)
    {
        case None:
            return;
        case ModifiedSavings:
            startRoutes = GenerateStartSolution();
            break;
        case Given:
            startRoutes = SetGivenStartSolution();
            break;
        case HardCoded:
            startRoutes = SetHardCodedStartSolution();
            break;
        default:
            throw std::runtime_error("Start solution type not implemented.");
    }

    for (const auto& route: startRoutes)
    {
        mStartSolutionArcs.emplace_back(1.0, mInstance->GetDepotId(), route.Sequence.front());

        for (size_t iNode = 0; iNode < route.Sequence.size() - 1; ++iNode)
        {
            auto nodeA = route.Sequence[iNode];
            auto nodeB = route.Sequence[iNode + 1];
            mStartSolutionArcs.emplace_back(1.0, nodeA, nodeB);
        }

        mStartSolutionArcs.emplace_back(1.0, route.Sequence.back(), mInstance->GetDepotId());
    }

    for (size_t k = 0; k < startRoutes.size(); ++k)
    {
        const auto& route = startRoutes[k];

        mStartSolution.Costs += Evaluator::CalculateRouteCosts(mInstance, route.Sequence);

        std::vector<Node> sequence;
        for (const auto id: route.Sequence)
        {
            sequence.emplace_back(mInstance->Nodes[id]);
        }

        Vehicle& vehicle = mInstance->Vehicles[k];
        mStartSolution.Tours.emplace_back(mInstance->Nodes[mInstance->DepotIndex], vehicle, std::move(sequence));
    }

    mStartSolution.NumberOfRoutes = mStartSolution.Tours.size();

    mTimer.StartSolution = std::chrono::system_clock::now() - start;

    mLogFile << "Start solution with " << mStartSolution.NumberOfRoutes << " Vehicles and total costs "
             << mStartSolution.Costs << " in " << mTimer.StartSolution.count() << " s.\n";

    std::string solutionString = "StartSolution-" + mInstance->Name;
    Serializer::WriteToJson(mStartSolution, mOutputPath, solutionString);
}

std::vector<Route> BranchAndCutSolver::GenerateStartSolution()
{
    auto startSolution =
        Heuristics::Constructive::ModifiedSavings(mInstance, &mInputParameters, mLoadingChecker.get(), &mRNG).Run();

    for (const auto& route: mLoadingChecker->GetFeasibleRoutes())
    {
        if (route.size() < 2)
        {
            continue;
        }

        LocalSearch::RunIntraImprovement(mInstance, mLoadingChecker.get(), &mInputParameters, route);
    }

    auto spHeuristic = Heuristics::SetBased::SPHeuristic(mInstance, mLoadingChecker.get(), &mInputParameters, mEnv);

    auto sequences = spHeuristic.Run(std::numeric_limits<double>::max());

    std::vector<Route> solution;
    int id = 0;
    for (auto& sequence: *sequences)
    {
        solution.emplace_back(id++, sequence);
    }

    return solution;
}
std::vector<Route> BranchAndCutSolver::SetGivenStartSolution()
{
    FunctionTimer<std::chrono::milliseconds> clock;

    std::ofstream logFile;
    logFile.open(mOutputPath + "/Log_StartSolution.log");

    std::string problemVariantString = mInputParameters.ContainerLoading.LoadingProblem.GetVariantString();
    std::string startSolutionFilePath =
        mStartSolutionFolderPath + problemVariantString + "/" + mInstance->Name + ".json";

    bool isRegularFile = std::filesystem::is_regular_file(startSolutionFilePath);
    if (!isRegularFile)
    {
        std::string exceptionMessage = startSolutionFilePath + " is not a regular file.";
        throw std::runtime_error(exceptionMessage.c_str());
    }

    auto solutionFile = std::ifstream(startSolutionFilePath);
    auto startSolution = HelperIO::ParseSolutionJson(solutionFile);

    auto& container = mInstance->Vehicles.front().Containers.front();

    uint64_t totalTimeHeur = 0;
    uint64_t totalTimeExact = 0;

    int nFeasibleHeur = 0;
    int nFeasibleExact = 0;

    for (size_t id = 0; id < startSolution.size(); id++)
    {
        auto& route = startSolution[id];
        const auto& sequence = route.Sequence;
        std::vector<Node> nodesInRoute;
        std::vector<Cuboid> selectedItems;
        auto totalWeight = 0.0;
        auto totalVolume = 0.0;
        for (size_t i = 0; i < sequence.size(); ++i)
        {
            const auto& nodeId = sequence[i];
            nodesInRoute.emplace_back(mInstance->Nodes[nodeId]);

            auto& items = mInstance->Nodes[nodeId].Items;
            totalWeight += mInstance->Nodes[nodeId].TotalWeight;
            totalVolume += mInstance->Nodes[nodeId].TotalVolume;
            for (auto& item: items)
            {
                item.GroupId = sequence.size() - 1 - i;
                selectedItems.emplace_back(item);
            }
        }

        auto tour = Tour(mInstance->Nodes[0], mInstance->Vehicles[id], nodesInRoute);

        if (totalWeight > container.WeightLimit)
        {
            throw std::runtime_error("Route " + std::to_string(id) + tour.Print() + " with total weight "
                                     + std::to_string(totalWeight) + " exceeds weight limit "
                                     + std::to_string(container.WeightLimit));
        }

        if (totalVolume > container.Volume)
        {
            throw std::runtime_error("Route " + std::to_string(id) + tour.Print() + " with total volume "
                                     + std::to_string(totalVolume) + " exceeds volume limit "
                                     + std::to_string(container.Volume));
        }

        logFile << "Route " << std::to_string(id) + tour.Print() << ": nItems " << std::to_string(selectedItems.size())
                << " | weight util " + std::to_string(totalWeight / container.WeightLimit)
                << " | volume util " + std::to_string(totalVolume / container.Volume) << " | ";

        if (!mInputParameters.ContainerLoading.LoadingProblem.EnableThreeDimensionalLoading)
        {
            logFile << "\n";
            continue;
        }

        clock.start();
        auto heuristicStatus =
            mLoadingChecker->PackingHeuristic(PackingType::Complete, container, sequence, selectedItems);
        clock.end();

        if (heuristicStatus == LoadingStatus::FeasOpt)
        {
            nFeasibleHeur++;
        }

        std::string feasStatusHeur = heuristicStatus == LoadingStatus::FeasOpt ? "feasible" : "infeasible";
        logFile << feasStatusHeur << " with packing heuristic : " << std::to_string(clock.elapsed()) << " | ";
        totalTimeHeur += clock.elapsed();

        clock.start();
        auto exactStatus =
            mLoadingChecker->ConstraintProgrammingSolver(PackingType::Complete,
                                                         container,
                                                         mLoadingChecker->MakeBitset(mInstance->Nodes.size(), sequence),
                                                         sequence,
                                                         selectedItems,
                                                         mInputParameters.IsExact(BranchAndCutParams::CallType::Exact));
        clock.end();

        if (exactStatus == LoadingStatus::FeasOpt)
        {
            nFeasibleExact++;
        }

        std::string feasStatusCP = exactStatus == LoadingStatus::FeasOpt ? "feasible" : "infeasible";
        logFile << feasStatusCP << " with CP model : " << std::to_string(clock.elapsed()) << "\n";
        totalTimeExact += clock.elapsed();

        if (exactStatus == LoadingStatus::Infeasible)
        {
            throw std::runtime_error("Loading infeasible according to CP model.");
        }
    }

    logFile << "Total heuristic time: " << std::to_string(totalTimeHeur)
            << " | Feasible: " << std::to_string(nFeasibleHeur) << "\n";
    mLogFile << "Total heuristic time: " << std::to_string(totalTimeHeur)
             << " | Feasible: " << std::to_string(nFeasibleHeur) << "\n";
    logFile << "Total exact time: " << std::to_string(totalTimeExact)
            << " | Feasible: " << std::to_string(nFeasibleExact) << "\n";
    logFile.close();

    return startSolution;
}
std::vector<Route> BranchAndCutSolver::SetHardCodedStartSolution()
{
    FunctionTimer<std::chrono::milliseconds> clock;

    std::vector<Collections::IdVector> startSequences = {Collections::IdVector{4, 13, 14},
                                                         Collections::IdVector{5, 10, 15, 12},
                                                         Collections::IdVector{11, 9, 2, 1},
                                                         Collections::IdVector{3, 8, 7, 6}};

    std::vector<Route> startSolution;
    startSolution.reserve(startSequences.size());
    for (size_t id = 0; id < startSequences.size(); ++id)
    {
        startSolution.emplace_back(id, startSequences[id]);
    }

    auto& container = mInstance->Vehicles.front().Containers.front();

    uint64_t totalTimeHeur = 0;
    uint64_t totalTimeExact = 0;

    int nFeasibleHeur = 0;
    int nFeasibleExact = 0;

    for (size_t id = 0; id < startSolution.size(); id++)
    {
        auto& route = startSolution[id];
        const auto& sequence = route.Sequence;
        std::vector<Node> nodesInRoute;
        nodesInRoute.reserve(sequence.size());
        std::vector<Cuboid> selectedItems;
        auto totalWeight = 0.0;
        auto totalVolume = 0.0;
        for (size_t i = 0; i < sequence.size(); ++i)
        {
            const auto& nodeId = static_cast<size_t>(sequence[i]);
            nodesInRoute.emplace_back(mInstance->Nodes[nodeId]);

            auto& items = mInstance->Nodes[nodeId].Items;
            totalWeight += mInstance->Nodes[nodeId].TotalWeight;
            totalVolume += mInstance->Nodes[nodeId].TotalVolume;
            for (auto& item: items)
            {
                item.GroupId = sequence.size() - 1 - i;
                selectedItems.emplace_back(item);
            }
        }

        auto tour = Tour(mInstance->Nodes[0], mInstance->Vehicles[id], nodesInRoute);

        if (totalWeight > container.WeightLimit)
        {
            throw std::runtime_error("Route " + std::to_string(id) + tour.Print() + " with total weight "
                                     + std::to_string(totalWeight) + " exceeds weight limit "
                                     + std::to_string(container.WeightLimit));
        }

        if (totalVolume > container.Volume)
        {
            throw std::runtime_error("Route " + std::to_string(id) + tour.Print() + " with total volume "
                                     + std::to_string(totalVolume) + " exceeds volume limit "
                                     + std::to_string(container.Volume));
        }

        mLogFile << "Route " << std::to_string(id) + tour.Print() << ": nItems " << std::to_string(selectedItems.size())
                 << " | weight util " + std::to_string(totalWeight / container.WeightLimit)
                 << " | volume util " + std::to_string(totalVolume / container.Volume) << " | ";

        if (!mInputParameters.ContainerLoading.LoadingProblem.EnableThreeDimensionalLoading)
        {
            mLogFile << "\n";
            continue;
        }

        clock.start();
        auto heuristicStatus =
            mLoadingChecker->PackingHeuristic(PackingType::Complete, container, sequence, selectedItems);
        clock.end();

        if (heuristicStatus == LoadingStatus::FeasOpt)
        {
            nFeasibleHeur++;
        }

        std::string feasStatusHeur = heuristicStatus == LoadingStatus::FeasOpt ? "feasible" : "infeasible";
        mLogFile << feasStatusHeur << " with packing heuristic : " << std::to_string(clock.elapsed()) << " | ";
        totalTimeHeur += clock.elapsed();

        clock.start();
        auto exactStatus =
            mLoadingChecker->ConstraintProgrammingSolver(PackingType::Complete,
                                                         container,
                                                         mLoadingChecker->MakeBitset(mInstance->Nodes.size(), sequence),
                                                         sequence,
                                                         selectedItems,
                                                         mInputParameters.IsExact(BranchAndCutParams::CallType::Exact));
        clock.end();

        if (exactStatus == LoadingStatus::FeasOpt)
        {
            nFeasibleExact++;
        }

        std::string feasStatusCP = exactStatus == LoadingStatus::FeasOpt ? "feasible" : "infeasible";
        mLogFile << feasStatusCP << " with CP model : " << std::to_string(clock.elapsed()) << "\n";
        totalTimeExact += clock.elapsed();

        if (exactStatus == LoadingStatus::Infeasible)
        {
            throw std::runtime_error("Loading infeasible according to CP model.");
        }
    }

    mLogFile << "Total heuristic time: " << std::to_string(totalTimeHeur)
             << " | Feasible: " << std::to_string(nFeasibleHeur) << "\n";
    mLogFile << "Total heuristic time: " << std::to_string(totalTimeHeur)
             << " | Feasible: " << std::to_string(nFeasibleHeur) << "\n";
    mLogFile << "Total exact time: " << std::to_string(totalTimeExact)
             << " | Feasible: " << std::to_string(nFeasibleExact) << "\n";
    mLogFile.close();

    return startSolution;
};

void BranchAndCutSolver::InfeasibleArcProcedure()
{
    std::chrono::time_point<std::chrono::system_clock> start;
    start = std::chrono::system_clock::now();

    DetermineInfeasiblePaths();
    mLogFile << "Deleted arcs: " << std::to_string(mInfeasibleArcs.size()) << "\n";
    mLogFile << "Infeasible tail paths: " << std::to_string(mInfeasibleTailPaths.size()) << "\n";
    mLogFile << "Infeasible 2-node combinations: " << std::to_string(mLoadingChecker->GetSizeInfeasibleCombinations())
             << "\n";

    DetermineExtendedInfeasiblePath();

    mLogFile << "Deleted arcs: " << std::to_string(mInfeasibleArcs.size()) << "\n";
    mLogFile << "Infeasible tail paths: " << std::to_string(mInfeasibleTailPaths.size()) << "\n";
    mLogFile << "Infeasible 2-node combinations: " << std::to_string(mLoadingChecker->GetSizeInfeasibleCombinations())
             << "\n";

    mTimer.InfeasibleArcs = std::chrono::system_clock::now() - start;
    // DetermineInfeasibleCustomerCombinations();
    // mLogFile << "Infeasible customer combinations size 3: " << std::to_string(mInfeasibleCombinations.size()) <<
    // "\n";
}

void BranchAndCutSolver::DetermineInfeasiblePaths()
{
    mLogFile << "## Start infeasible path procedure ## "
             << "\n";

    auto& container = mInstance->Vehicles.front().Containers.front();
    const auto& nodes = mInstance->Nodes;
    for (size_t iNode = 1; iNode < nodes.size() - 1; ++iNode)
    {
        // mLogFile << "Node: " << std::to_string(iNode) << "\n";
        for (size_t jNode = iNode + 1; jNode < nodes.size(); ++jNode)
        {
            if (nodes[iNode].TotalWeight + nodes[jNode].TotalWeight > container.WeightLimit
                || nodes[iNode].TotalVolume + nodes[jNode].TotalVolume > container.Volume)
            {
                boost::dynamic_bitset<> nodesInSet(nodes.size());
                nodesInSet.set(iNode).set(jNode);
                mLoadingChecker->AddInfeasibleCombination(nodesInSet);
                mInfeasibleArcs.emplace_back(0, iNode, jNode);
                mInfeasibleArcs.emplace_back(0, jNode, iNode);
                continue;
            }

            if (!mInputParameters.ContainerLoading.LoadingProblem.EnableThreeDimensionalLoading)
            {
                continue;
            }

            Collections::IdVector selectedNodes = {iNode, jNode};
            auto selectedItems = InterfaceConversions::SelectItems(selectedNodes, mInstance->Nodes, false);
            bool forwardRelaxedInfeasible = !CheckPath(selectedNodes, container, selectedItems);

            std::swap(selectedNodes[0], selectedNodes[1]);
            selectedItems = InterfaceConversions::SelectItems(selectedNodes, mInstance->Nodes, false);
            bool backwardRelaxedInfeasible = !CheckPath(selectedNodes, container, selectedItems);

            if (forwardRelaxedInfeasible && backwardRelaxedInfeasible)
            {
                boost::dynamic_bitset<> nodesInSet(nodes.size());
                nodesInSet.set(iNode).set(jNode);
                mLoadingChecker->AddInfeasibleCombination(nodesInSet);
            }
        }
    }
}

bool BranchAndCutSolver::CheckPath(const Collections::IdVector& path, Container& container, std::vector<Cuboid>& items)
{
    if (mInputParameters.BranchAndCut.ActivateHeuristic)
    {
        auto heuristicStatus = mLoadingChecker->PackingHeuristic(PackingType::Complete, container, path, items);
        if (heuristicStatus == LoadingStatus::FeasOpt)
        {
            return true;
        }
    }

    auto statusSupportRelaxation =
        mLoadingChecker->ConstraintProgrammingSolver(PackingType::NoSupport,
                                                     container,
                                                     mLoadingChecker->MakeBitset(mInstance->Nodes.size(), path),
                                                     path,
                                                     items,
                                                     mInputParameters.IsExact(BranchAndCutParams::CallType::Exact));

    if (statusSupportRelaxation == LoadingStatus::Infeasible)
    {
        mInfeasibleArcs.emplace_back(0, path.front(), path.back());
        return false;
    }

    auto statusComplete =
        mLoadingChecker->ConstraintProgrammingSolver(PackingType::Complete,
                                                     container,
                                                     mLoadingChecker->MakeBitset(mInstance->Nodes.size(), path),
                                                     path,
                                                     items,
                                                     mInputParameters.IsExact(BranchAndCutParams::CallType::Exact));

    if (statusComplete == LoadingStatus::Infeasible)
    {
        mInfeasibleTailPaths.emplace_back(0, path.front(), path.back());
    }

    return true;
}

void BranchAndCutSolver::DetermineExtendedInfeasiblePath()
{
    if (!mInputParameters.ContainerLoading.LoadingProblem.EnableThreeDimensionalLoading)
    {
        return;
    }

    mLogFile << "## Start extended infeasible path procedure ##\n";

    auto& container = mInstance->Vehicles.front().Containers.front();
    const auto& nodes = mInstance->Nodes;

    std::vector<Arc> tailPathToDelete;

    for (auto& arc: mInfeasibleTailPaths)
    {
        const auto& nodeI = nodes[arc.Tail];
        const auto& nodeJ = nodes[arc.Head];

        auto weight = nodeI.TotalWeight + nodeJ.TotalWeight;
        auto volume = nodeI.TotalVolume + nodeJ.TotalVolume;

        auto extraNodeFeasible = false;
        for (const auto& nodeK: mInstance->GetCustomers())
        {
            if (nodeK.InternId == nodeI.InternId || nodeK.InternId == nodeJ.InternId)
            {
                continue;
            }

            if (weight + nodeK.TotalWeight > container.WeightLimit || volume + nodeK.TotalVolume > container.Volume)
            {
                continue;
            }

            Collections::IdVector path = {nodeI.InternId, nodeJ.InternId, nodeK.InternId};

            auto selectedItems = InterfaceConversions::SelectItems(path, mInstance->Nodes, false);

            auto heuristicStatus = LoadingStatus::Infeasible;

            if (mInputParameters.BranchAndCut.ActivateHeuristic)
            {
                heuristicStatus =
                    mLoadingChecker->PackingHeuristic(PackingType::Complete, container, path, selectedItems);
            }

            if (heuristicStatus == LoadingStatus::Infeasible)
            {
                auto statusSupportRelaxation = mLoadingChecker->ConstraintProgrammingSolver(
                    PackingType::NoSupport,
                    container,
                    mLoadingChecker->MakeBitset(mInstance->Nodes.size(), path),
                    path,
                    selectedItems,
                    mInputParameters.IsExact(BranchAndCutParams::CallType::Exact));

                if (statusSupportRelaxation == LoadingStatus::Infeasible)
                {
                    continue;
                }
            }

            extraNodeFeasible = true;
            break;
        }

        if (extraNodeFeasible)
        {
            continue;
        }

        mInfeasibleArcs.emplace_back(0.0, nodeI.InternId, nodeJ.InternId);
        tailPathToDelete.emplace_back(arc);
    }

    for (const auto& path: tailPathToDelete)
    {
        std::erase_if(mInfeasibleTailPaths,
                      [path](Arc& arc) { return path.Head == arc.Head && path.Tail == arc.Tail; });
    }
}

void BranchAndCutSolver::DetermineInfeasibleCustomerCombinations()
{
    std::chrono::time_point<std::chrono::system_clock> start;
    start = std::chrono::system_clock::now();

    Container& container = mInstance->Vehicles.front().Containers.front();
    auto& nodes = mInstance->Nodes;

    for (size_t iNode = 1; iNode < nodes.size() - 2; ++iNode)
    {
        for (size_t jNode = iNode + 1; jNode < nodes.size() - 1; ++jNode)
        {
            for (size_t kNode = jNode + 1; kNode < nodes.size(); ++kNode)
            {
                double totalWeight = nodes[iNode].TotalWeight + nodes[jNode].TotalWeight + nodes[kNode].TotalWeight;
                double totalVolume = nodes[iNode].TotalVolume + nodes[jNode].TotalVolume + nodes[kNode].TotalVolume;

                if (totalWeight > container.WeightLimit || totalVolume > container.Volume)
                {
                    boost::dynamic_bitset<> nodesInSet(nodes.size());
                    nodesInSet.set(iNode);
                    nodesInSet.set(jNode);
                    nodesInSet.set(kNode);
                    mLoadingChecker->AddInfeasibleCombination(nodesInSet);
                    mInfeasibleCombinations.insert({iNode, jNode, kNode});
                    continue;
                }

                Collections::IdVector path = {iNode, jNode, kNode};

                auto selectedItems = InterfaceConversions::SelectItems(path, mInstance->Nodes, false);

                // Relaxed stability: supportAreaRequirement = 0.0.
                auto heuristicStatus =
                    mLoadingChecker->PackingHeuristic(PackingType::Complete, container, path, selectedItems);

                if (heuristicStatus == LoadingStatus::Infeasible)
                {
                    boost::dynamic_bitset<> nodesInSet(nodes.size());
                    nodesInSet.set(iNode);
                    nodesInSet.set(jNode);
                    nodesInSet.set(kNode);

                    double maxRuntime = mInputParameters.DetermineMaxRuntime(BranchAndCutParams::CallType::Exact);
                    auto status = mLoadingChecker->ConstraintProgrammingSolver(
                        PackingType::NoSupportNoSequence,
                        container,
                        nodesInSet,
                        path,
                        selectedItems,
                        mInputParameters.IsExact(BranchAndCutParams::CallType::Exact),
                        maxRuntime);

                    if (status == LoadingStatus::Infeasible)
                    {
                        mLoadingChecker->AddInfeasibleCombination(nodesInSet);
                        mInfeasibleCombinations.insert(path);
                    }
                }
            }
        }
    }
}

size_t BranchAndCutSolver::DetermineLowerBoundVehicles()
{
    std::chrono::time_point<std::chrono::system_clock> start;
    start = std::chrono::system_clock::now();

    // std::map<std::tuple<int, int>, int> infeasibleCombinations;

    std::vector<Container> containers;
    containers.reserve(mInstance->Vehicles.size());
    for (const auto& vehicle: mInstance->Vehicles)
    {
        containers.emplace_back(vehicle.Containers[0]);
    }

    std::vector<Node> customerNodes;
    customerNodes.reserve(mInstance->GetCustomers().size());
    for (const auto& node: mInstance->GetCustomers())
    {
        customerNodes.emplace_back(node);
    }

    auto groups = InterfaceConversions::NodesToGroup(customerNodes);
    auto binPacking = BinPacking1D(mEnv, containers, groups, mOutputPath);

    auto lowerBound1D = static_cast<size_t>(mLoadingChecker->SolveBinPackingApproximation());

    if (!mInputParameters.ContainerLoading.LoadingProblem.EnableThreeDimensionalLoading)
    {
        while (mInstance->Vehicles.size() > lowerBound1D)
        {
            mInstance->Vehicles.pop_back();
        }
    }

    mTimer.LowerBoundVehicles = std::chrono::system_clock::now() - start;

    return lowerBound1D;
}

void BranchAndCutSolver::Solve()
{
    mLogFile << "### START EXACT APPROACH ###\n";

    std::string parameterString = "Parameters-" + mInstance->Name;
    Serializer::WriteToJson(mInputParameters, mOutputPath, parameterString);

    Initialize();

    ////TestProcedure();

    Preprocessing();

    std::chrono::time_point<std::chrono::system_clock> start;
    start = std::chrono::system_clock::now();
    TwoIndexVehicleFlow branchAndCut(mInstance, mEnv);

    branchAndCut.BuildModel(mStartSolutionArcs, mInfeasibleArcs, mInfeasibleTailPaths);
    auto callback = CallbackFactory::CreateCallback(mInputParameters.ContainerLoading.LoadingProblem.Variant,
                                                    mEnv,
                                                    *branchAndCut.GetXVariables(),
                                                    mInstance,
                                                    mLoadingChecker.get(),
                                                    &mInputParameters,
                                                    mOutputPath);
    branchAndCut.SetCallback(callback.get());
    branchAndCut.Solve(mInputParameters.MIPSolver);

    mTimer.BranchAndCut = std::chrono::system_clock::now() - start;

    auto statistics = SolverStatistics(branchAndCut.GetRuntime(),
                                       branchAndCut.GetMIPGap(),
                                       branchAndCut.GetNodeCount(),
                                       branchAndCut.GetSimIterCount(),
                                       callback->CallbackTracker,
                                       mTimer,
                                       mInfeasibleArcs.size(),
                                       mInfeasibleTailPaths.size());

    std::string solutionStatisticsString = "SolutionStatistics-" + mInstance->Name;
    Serializer::WriteToJson(statistics, mOutputPath, solutionStatisticsString);

    if (const auto optionalSolution = branchAndCut.GetSolution(); optionalSolution.has_value())
    {
        mFinalSolution = optionalSolution.value();
    }
    else
    {
        return;
    }

    DeterminePackingSolution();

    mFinalSolution.LowerBoundVehicles = mInstance->LowerBoundVehicles;
    mFinalSolution.DetermineCosts(mInstance);

    auto solFile = SolutionFile(mInputParameters, statistics, mFinalSolution);

    std::string solutionString = "Solution-" + mInstance->Name;
    Serializer::WriteToJson(solFile, mOutputPath, solutionString);

    WriteSolutionSolutionValidator();
}

void BranchAndCutSolver::DeterminePackingSolution()
{
    mFinalSolution.NumberOfRoutes = mFinalSolution.Tours.size();
    for (size_t tourId = 0; tourId < mFinalSolution.Tours.size(); tourId++)
    {
        auto& tour = mFinalSolution.Tours[tourId];
        auto& route = tour.Route;
        auto& container = tour.Vehicle.Containers.front();
        Collections::IdVector stopIds;
        std::vector<Cuboid> selectedItems;
        auto totalWeight = 0.0;
        auto totalVolume = 0.0;

        for (size_t i = 0; i < route.size(); ++i)
        {
            auto& items = route[i].Items;
            totalWeight += route[i].TotalWeight;
            totalVolume += route[i].TotalVolume;
            stopIds.push_back(route[i].InternId);

            for (auto& item: items)
            {
                item.GroupId = route.size() - 1 - i;
                selectedItems.emplace_back(item);
            }
        }

        if (totalWeight > container.WeightLimit)
        {
            throw std::runtime_error("Route " + std::to_string(tourId) + tour.Print() + " with total weight "
                                     + std::to_string(totalWeight) + " exceeds weight limit "
                                     + std::to_string(container.WeightLimit));
        }

        if (totalVolume > container.Volume)
        {
            throw std::runtime_error("Route " + std::to_string(tourId) + tour.Print() + " with total volume "
                                     + std::to_string(totalVolume) + " exceeds volume limit "
                                     + std::to_string(container.Volume));
        }

        mLogFile << "Route " << std::to_string(tourId) + tour.Print()
                 << ": weight util " + std::to_string(totalWeight / container.WeightLimit)
                 << " | volume util " + std::to_string(totalVolume / container.Volume) << " | ";

        if (!mInputParameters.ContainerLoading.LoadingProblem.EnableThreeDimensionalLoading)
        {
            mLogFile << "\n";
            continue;
        }

        auto heuristicStatus = LoadingStatus::Infeasible;
        if (mInputParameters.BranchAndCut.ActivateHeuristic)
        {
            heuristicStatus =
                mLoadingChecker->PackingHeuristic(PackingType::Complete, container, stopIds, selectedItems);
        }

        std::string feasStatusHeur = heuristicStatus == LoadingStatus::FeasOpt ? "feasible" : "infeasible";
        mLogFile << feasStatusHeur << " with packing heuristic | ";

        double maxRuntime = mInputParameters.DetermineMaxRuntime(BranchAndCutParams::CallType::Exact);
        auto exactStatus = mLoadingChecker->ConstraintProgrammingSolverGetPacking(
            PackingType::Complete, container, stopIds, selectedItems, maxRuntime);

        std::string feasStatusCP = exactStatus == LoadingStatus::FeasOpt ? "feasible" : "infeasible";
        mLogFile << feasStatusCP << " with CP model"
                 << "\n";

        // TODO: packing as return value of loading checker

        if (exactStatus == LoadingStatus::Infeasible)
        {
            throw std::runtime_error("Loading infeasible according to CP model.");
        }

        size_t cItems = 0;
        for (auto& stop: route)
        {
            for (auto& item: stop.Items)
            {
                item = selectedItems[cItems];
                cItems++;
            }
        }
    }
}

void BranchAndCutSolver::PrintSolution()
{
    for (size_t tourId = 0; tourId < mFinalSolution.Tours.size(); tourId++)
    {
        Tour& tour = mFinalSolution.Tours[tourId];
        std::vector<Node>& route = tour.Route;

        mLogFile << "0 -> ";
        for (const auto& node: route)
        {
            mLogFile << node.InternId << " -> ";
        }
        mLogFile << "0"
                 << "\n";
    }
}

void BranchAndCutSolver::WriteSolutionSolutionValidator()
{
    ResultWriter::SolutionValidator::WriteInput(mOutputPath,
                                                mInstance->Name,
                                                InterfaceConversions::NodesToGroup(mInstance->Nodes),
                                                mInstance->Vehicles[0].Containers[0],
                                                mInstance->Vehicles.size());

    std::vector<std::vector<Group>> routes;
    routes.reserve(mFinalSolution.Tours.size());
    for (const auto& tour: mFinalSolution.Tours)
    {
        routes.emplace_back(InterfaceConversions::NodesToGroup(tour.Route));
    }

    ResultWriter::SolutionValidator::WriteOutput(mOutputPath, mInstance->Name, routes, mFinalSolution.Costs);
}

}
}