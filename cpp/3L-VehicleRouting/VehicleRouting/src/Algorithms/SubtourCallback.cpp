#include "Algorithms/Heuristics/LocalSearch.h"

#include "Algorithms/LoadingInterfaceServices.h"
#include "Algorithms/LoadingStatus.h"
#include "Algorithms/SubtourCallback.h"
#include "CommonBasics/Helper/ModelServices.h"

#include <algorithm>
#include <boost/dynamic_bitset/dynamic_bitset.hpp>
#include <cstddef>
#include <functional>
#include <ranges>

namespace VehicleRouting
{
using namespace Helper;

namespace Algorithms
{
using namespace Cuts;
using namespace Heuristics::Improvement;

void SubtourCallback::callback()
{
    try
    {
        switch (where)
        {
            case GRB_CB_MIP:
            {
                double objBound = this->getDoubleInfo(GRB_CB_MIP_OBJBND);
                if (objBound == 0)
                {
                    break;
                }

                CallbackTracker.UpdateLowerBound(
                    this->getDoubleInfo(GRB_CB_RUNTIME), this->getDoubleInfo(GRB_CB_MIP_NODCNT), objBound);
            }
            break;
            case GRB_CB_MIPNODE:
            {
                mCurrentNode = static_cast<unsigned int>(this->getDoubleInfo(GRB_CB_MIPNODE_NODCNT));
                ////mLogFile << mCurrentNode << " | NODE | Feas Routes: " <<
                ////mLoadingChecker->GetFeasibleRoutes().size()<< "\n";

                CallbackTracker.UpdateLowerBound(this->getDoubleInfo(GRB_CB_RUNTIME),
                                                 this->getDoubleInfo(GRB_CB_MIPNODE_NODCNT),
                                                 this->getDoubleInfo(GRB_CB_MIPNODE_OBJBND));

                // Check fractional solution
                auto checkFractionalSolutionTime =
                    measure<>::duration(std::bind_front(&SubtourCallback::CheckFractionalSolution, this));
                CallbackTracker.UpdateElement(CallbackElement::FractionalSolutions,
                                              static_cast<uint64_t>(checkFractionalSolutionTime.count()));

                // Solve set partitioning heuristic
                auto [spHeuristicCalled, heuristicTime] = measure<>::durationWithReturn(
                    std::bind_front(&SubtourCallback::SolveSetPartitioningHeuristic, this));
                if (!spHeuristicCalled)
                {
                    break;
                }

                CallbackTracker.UpdateElement(CallbackElement::SPHeuristic,
                                              static_cast<uint64_t>(heuristicTime.count()));
            }
            break;
            case GRB_CB_MIPSOL:
            {
                mCurrentNode = static_cast<unsigned int>(this->getDoubleInfo(GRB_CB_MIPSOL_NODCNT));
                ////mLogFile << mCurrentNode << " | SOL | Feas Routes: " <<
                ////mLoadingChecker->GetFeasibleRoutes().size()<< "\n";

                // Check integer solution
                auto checkIntegerSolutionTime =
                    measure<>::duration(std::bind_front(&SubtourCallback::CheckIntegerSolution, this));
                CallbackTracker.UpdateElement(CallbackElement::IntegerSolutions,
                                              static_cast<uint64_t>(checkIntegerSolutionTime.count()));
            }
            break;
            default:
                break;
        }
    }
    catch (GRBException& e)
    {
        std::cout << "Error number: " << e.getErrorCode() << "\n";
        std::cout << e.getMessage() << "\n";
    }
    catch (...)
    {
        std::cout << "Error during callback"
                  << "\n";
    }
}

void SubtourCallback::InitializeCuts()
{
    const auto nNodes = mInstance->Nodes.size();
    const auto nCustomerNodes = mInstance->GetCustomers().size();

    std::vector<double> weight;
    std::vector<double> volume;
    for (const auto& node: mInstance->Nodes)
    {
        weight.push_back(node.TotalWeight);
        volume.push_back(node.TotalVolume);
    }

    const auto& container = mInstance->Vehicles[0].Containers[0];
    GraphFunctions::SetValues(1, nCustomerNodes, nNodes, mInputParameters->UserCut.EpsForIntegrality);

    auto weightLimit = container.WeightLimit;
    auto volumeLimit = container.Volume;

    // Cuts
    if (weightLimit > 0)
    {
        mCutTypesFractional.emplace_back(std::make_shared<RCCut>(
            mInputParameters, nCustomerNodes, weightLimit, weight, mInstance, &mCVRPSEPGraph, mLoadingChecker));
    }

    if (volumeLimit > 0)
    {
        mCutTypesFractional.emplace_back(std::make_shared<RCCut>(
            mInputParameters, nCustomerNodes, volumeLimit, volume, mInstance, &mCVRPSEPGraph, mLoadingChecker));
    }

    mCutTypesFractional.emplace_back(std::make_shared<CatCut>(mInputParameters));
    mCutTypesFractional.emplace_back(std::make_shared<DK_plus>(mInputParameters));
    mCutTypesFractional.emplace_back(std::make_shared<DK_min>(mInputParameters));

    mCutTypesFractional.emplace_back(
        std::make_shared<MSTAR>(mInputParameters, nCustomerNodes, weightLimit, weight, mInstance, &mCVRPSEPGraph));
    // // mCutTypesFractional.emplace_back(
    // //     std::make_shared<MSTAR>(mInputParameters, nCustomerNodes, volumeLimit, volume, mInstance,
    // // &mCVRPSEPGraph));

    mCutTypesFractional.emplace_back(std::make_shared<FCI>(
        mInputParameters, nCustomerNodes, weightLimit, weight, mInstance, &mCVRPSEPGraph, mLoadingChecker));
    // // mCutTypesFractional.emplace_back(std::make_shared<FCI>(
    // //     mInputParameters, nCustomerNodes, volumeLimit, volume, mInstance, &mCVRPSEPGraph, mLoadingChecker));

    mCutTypesFractional.emplace_back(
        std::make_shared<SCI>(mInputParameters, nCustomerNodes, weightLimit, weight, mInstance, &mCVRPSEPGraph));

    mCutTypesFractional.emplace_back(
        std::make_shared<GLM>(mInputParameters, nCustomerNodes, weightLimit, weight, mInstance, &mCVRPSEPGraph));
    // // mCutTypesFractional.emplace_back(
    // //     std::make_shared<GLM>(mInputParameters, nCustomerNodes, volumeLimit, volume, mInstance, &mCVRPSEPGraph));
}

GRBLinExpr SubtourCallback::ConstructLHS(const std::vector<Arc>& arcs)
{
    GRBLinExpr lhs;
    for (const auto& arc: arcs)
    {
        lhs += arc.Coefficient * mVariablesX[arc.Tail][arc.Head];
    }

    return lhs;
}

void SubtourCallback::FillXVarValuesNode()
{
    for (size_t iNode = 0, n = mInstance->Nodes.size(); iNode < n - 1; ++iNode)
    {
        for (size_t jNode = iNode + 1; jNode < n; ++jNode)
        {
            double const valueIJ = getNodeRel(mVariablesX[iNode][jNode]);
            mVariableValuesX[iNode][jNode] = valueIJ < mInputParameters->UserCut.EpsForIntegrality ? 0.0 : valueIJ;

            double const valueJI = getNodeRel(mVariablesX[jNode][iNode]);
            mVariableValuesX[jNode][iNode] = valueJI < mInputParameters->UserCut.EpsForIntegrality ? 0.0 : valueJI;
        }
    }
}

void SubtourCallback::FillXVariableValuesFromSolution()
{
    for (size_t iNode = 0, n = mInstance->Nodes.size(); iNode < n - 1; ++iNode)
    {
        for (size_t jNode = iNode + 1; jNode < n; ++jNode)
        {
            double const valueIJ = this->getSolution(mVariablesX[iNode][jNode]);
            mVariableValuesX[iNode][jNode] = valueIJ < mInputParameters->UserCut.EpsForIntegrality ? 0.0 : valueIJ;

            double const valueJI = this->getSolution(mVariablesX[jNode][iNode]);
            mVariableValuesX[jNode][iNode] = valueJI < mInputParameters->UserCut.EpsForIntegrality ? 0.0 : valueJI;
        }
    }
}

void SubtourCallback::CheckIntegerSolution()
{
    FillXVariableValuesFromSolution();

    // Find subtours in integer solution
    auto subtourTime = measure<>::duration(std::bind_front(&SubtourCallback::FindIntegerSubtours, this));
    CallbackTracker.UpdateElement(CallbackElement::DetermineRoutes, static_cast<uint64_t>(subtourTime.count()));

    // Check integer routes for feasibility
    auto [solutionFeasible, checkRoutesTime] =
        measure<>::durationWithReturn(std::bind_front(&SubtourCallback::CheckRoutes, this));
    CallbackTracker.UpdateElement(CallbackElement::CheckRoutes, static_cast<uint64_t>(checkRoutesTime.count()));

    if (!solutionFeasible)
    {
        return;
    }

    double newCosts = this->getDoubleInfo(GRB_CB_MIPSOL_OBJ);
    if (newCosts > mBestSolutionValue - 1e-5)
    {
        return;
    }

    CallbackTracker.UpdateUpperBound(this->getDoubleInfo(GRB_CB_RUNTIME), newCosts, mSolSPheuristic);
    mBestSolutionValue = newCosts;
    mSolSPheuristic = false;
}

void SubtourCallback::FindIntegerSubtours()
{
    mSubtours.clear();

    constexpr auto integerThreshold = 0.5;

    auto nodeVisited = boost::dynamic_bitset<>(mInstance->Nodes.size());

    size_t counterVisitedNodes = 0;

    size_t vehiclesStartingAtDepot = 0;

    for (const auto& node: mInstance->Nodes)
    {
        if (mVariableValuesX[mInstance->GetDepotId()][node.InternId] > integerThreshold)
        {
            vehiclesStartingAtDepot++;
        }
    }

    while (counterVisitedNodes < mInstance->Nodes.size() - 1)
    {
        size_t startNodeId = 0;
        size_t previousNodeId = 0;
        Collections::IdVector customerSequence;
        bool connectedToDepot = true;
        double totalWeight = 0.0;
        double totalVolume = 0.0;
        boost::dynamic_bitset<> customersInRoute(mInstance->Nodes.size());

        if (mSubtours.size() < vehiclesStartingAtDepot)
        {
            startNodeId = mInstance->GetDepotId();
            previousNodeId = mInstance->GetDepotId();
            connectedToDepot = true;
        }
        else
        {
            for (const auto& customer: mInstance->GetCustomers())
            {
                if (!nodeVisited[customer.InternId])
                {
                    const auto id = customer.InternId;

                    startNodeId = id;
                    previousNodeId = id;
                    customerSequence.push_back(id);
                    totalWeight += customer.TotalWeight;
                    totalVolume += customer.TotalVolume;
                    customersInRoute.set(id);
                    connectedToDepot = false;

                    nodeVisited.set(id);
                    counterVisitedNodes++;

                    break;
                }
            }
        }

        std::optional<size_t> currentNode = std::nullopt;
        while (currentNode != startNodeId)
        {
            for (const auto& node: mInstance->Nodes)
            {
                const auto id = node.InternId;
                if (mVariableValuesX[previousNodeId][id] > integerThreshold)
                {
                    if (nodeVisited[id] && id != startNodeId)
                    {
                        continue;
                    }

                    if (id != startNodeId)
                    {
                        customerSequence.push_back(id);
                        totalWeight += node.TotalWeight;
                        totalVolume += node.TotalVolume;
                        customersInRoute.set(id);

                        previousNodeId = id;
                        currentNode = id;
                        counterVisitedNodes++;
                        nodeVisited.set(id);

                        break;
                    }

                    currentNode = id;
                    break;
                }
            }
        }

        mSubtours.emplace_back(connectedToDepot, customerSequence, customersInRoute, totalWeight, totalVolume);
    }

    std::ranges::sort(mSubtours,
                      [](const auto& subtourA, const auto& subtourB) -> bool
                      {
                          auto sizeA = subtourA.Sequence.size();
                          auto sizeB = subtourB.Sequence.size();

                          return std::tie(subtourA.ConnectedToDepot, sizeA)
                                 < std::tie(subtourB.ConnectedToDepot, sizeB);
                      }); // why sort? -->
}

bool SubtourCallback::RouteCheckedAndFeasible(const Collections::IdVector& sequence)
{
    if (!mInputParameters->BranchAndCut.ActivateMemoryManagement)
    {
        return false;
    }

    mClock.start();
    if (mLoadingChecker->RouteIsInFeasSequences(sequence))
    {
        mClock.end();
        CallbackTracker.UpdateElement(CallbackElement::RoutePrechecked, mClock.elapsed());

        return true;
    }

    mClock.end();
    CallbackTracker.UpdateElement(CallbackElement::RoutePrecheckedNot, mClock.elapsed());

    return false;
}

bool SubtourCallback::CustomerCombinationInfeasible(const Collections::IdVector& sequence,
                                                    const boost::dynamic_bitset<>& combination)
{
    if (!mInputParameters->BranchAndCut.ActivateMemoryManagement)
    {
        return false;
    }

    mClock.start();
    if (mLoadingChecker->CustomerCombinationInfeasible(combination))
    {
        AddLazyConstraints({mLazyConstraintsGenerator->CreateConstraint(CutType::SEC, sequence, 2)});
        mLoadingChecker->AddInfeasibleCombination(combination);
        mClock.end();
        CallbackTracker.UpdateElement(CallbackElement::CustCombiInf, mClock.elapsed());

        return true;
    }

    mClock.end();
    CallbackTracker.UpdateElement(CallbackElement::CustCombiInfNot, mClock.elapsed());

    return false;
}

void SubtourCallback::AddLazyConstraints(const std::vector<Cut>& lazyConstraints)
{
    for (const auto& constraint: lazyConstraints)
    {
        auto lhs = ConstructLHS(constraint.Arcs);

        this->addLazy(lhs >= constraint.RHS);

        CallbackTracker.LazyConstraintCounter[constraint.Type]++;
    }
}

void SubtourCallback::CheckFractionalSolution()
{
    auto status = this->getIntInfo(GRB_CB_MIPNODE_STATUS);
    mCurrentNode = static_cast<unsigned int>(this->getDoubleInfo(GRB_CB_MIPNODE_NODCNT));

    if (status != GRB_OPTIMAL)
    {
        return;
    }

    if (mCurrentNode > mInputParameters->BranchAndCut.CutSeparationMaxNodes)
    {
        return;
    }

    if (mCurrentNode > mInputParameters->BranchAndCut.CutSeparationStartNodes
        && (mCurrentNode - mLastNodeCount) < mInputParameters->BranchAndCut.CutSeparationThreshold)
    {
        return;
    }

    auto addFracCutTime = measure<>::duration(std::bind_front(&SubtourCallback::AddFractionalCuts, this)).count();
    CallbackTracker.UpdateElement(CallbackElement::AddFracSolCuts, static_cast<uint64_t>(addFracCutTime));

    /*
    mLogFile << "VRPSEP " << CallbackTracker.Counter[CallbackElement::FractionalSolutions] << ";"
             << mCurrentNode
             << ";RCC: " << CallbackTracker.CutCounter[CutType::RCC] +
    CallbackTracker.LazyConstraintCounter[CutType::RCC]
             << ";FCI: " << CallbackTracker.CutCounter[CutType::FC]
             << ";MST: " << CallbackTracker.CutCounter[CutType::MST]
             << ";GLM: " << CallbackTracker.CutCounter[CutType::GLM]
             << ";SCI: " << CallbackTracker.CutCounter[CutType::SC]
             << ";CAT: " << CallbackTracker.CutCounter[CutType::CAT]
             << ";DK+: " << CallbackTracker.CutCounter[CutType::DKplus]
             << ";DK-: " << CallbackTracker.CutCounter[CutType::DKminus]
             << "\n";
    */

    mLastNodeCount = mCurrentNode;
}

void SubtourCallback::AddFractionalCuts()
{
    mClock.start();
    FillXVarValuesNode();

    mCVRPSEPGraph.Build(mVariableValuesX, mInputParameters->UserCut.EpsForIntegrality);
    mClock.end();
    CallbackTracker.UpdateElement(CallbackElement::BuildGraph, mClock.elapsed());

    for (const auto& cut: mCutTypesFractional)
    {
        if (mInputParameters->UserCut.MaxCutsSeparate.at(cut->Type) == 0)
        {
            continue;
        }

        ////std::cout << "current node: " << currentNode << " | " << "Start Separation " << (int)cut_ptr->Type << "\n";

        mClock.start();
        auto cuts = cut->GetCuts(mVariableValuesX);
        mClock.end();
        CallbackTracker.CutTimer[cut->Type] += mClock.elapsed();

        ////std::cout << "End Separation " << (int)cut_ptr->Type << "\n";
        if (cuts.empty())
        {
            continue;
        }

        mClock.start();
        AddCuts(cuts);
        mClock.end();
        CallbackTracker.CutTimer[cut->Type] += mClock.elapsed();

        if (mCurrentNode > mInputParameters->BranchAndCut.CutSeparationStartNodes)
        {
            return;
        }
    }
}

void SubtourCallback::AddCuts(const std::vector<Cut>& cuts)
{
    size_t maxCutsToAdd =
        std::min(static_cast<size_t>(mInputParameters->UserCut.MaxCutsAdd.at(cuts[0].Type)), cuts.size());

    for (size_t iCut = 0; iCut < maxCutsToAdd; ++iCut)
    {
        const Cut& cut = cuts[iCut];

        auto lhs = ConstructLHS(cut.Arcs);

        if (cut.Type == CutType::RCC)
        {
            if (mCurrentNode == 0)
            {
                this->addCut(lhs >= cut.RHS);
                CallbackTracker.CutCounter[cut.Type]++;
            }
            else
            {
                if (cut.Violation >= mInputParameters->UserCut.MaxViolationCutLazy)
                {
                    this->addLazy(lhs >= cut.RHS);
                    CallbackTracker.LazyConstraintCounter[cut.Type]++;
                }
                else
                {
                    this->addCut(lhs >= cut.RHS);
                    CallbackTracker.CutCounter[cut.Type]++;
                }
            }
        }
        else
        {
            this->addCut(lhs >= cut.RHS);
            CallbackTracker.CutCounter[cut.Type]++;
        }
    }
}

bool SubtourCallback::SolveSetPartitioningHeuristic()
{
    if (SPHeuristic == nullptr)
    {
        return false;
    }

    if (mLoadingChecker->GetNumberOfFeasibleRoutes() < mInstance->Vehicles.size())
    {
        return false;
    }

    if (mLoadingChecker->GetNumberOfFeasibleRoutes() - mLastSolutionCount
        < mInputParameters->BranchAndCut.SetPartitioningHeuristicThreshold)
    {
        return false;
    }

    mLogFile << mCurrentNode << " : SP-Heuristic with " << mLoadingChecker->GetNumberOfFeasibleRoutes() << " routes."
             << "\n";

    auto newSolution = SPHeuristic->Run(this->getDoubleInfo(GRB_CB_MIPNODE_OBJBST) - 1e-5);

    mLogFile << "SC ObjVal:" << SPHeuristic->GetSCObjVal() << " | SP ObjVal: " << SPHeuristic->GetSPObjVal() << "\n";

    mLastSolutionCount = mLoadingChecker->GetNumberOfFeasibleRoutes();
    mLastTime = this->getDoubleInfo(GRB_CB_RUNTIME);

    if (!newSolution)
    {
        return true;
    }

    SetHeuristicSolution(newSolution.value());
    mSolSPheuristic = true;

    return true;
}

void SubtourCallback::SetHeuristicSolution(const Collections::SequenceVector& routes)
{
    for (const auto& route: routes)
    {
        setSolution(mVariablesX[mInstance->GetDepotId()][route.front()], 1.0);
        for (size_t iNode = 0; iNode < route.size() - 1; iNode++)
        {
            setSolution(mVariablesX[route[iNode]][route[iNode + 1]], 1.0);
        }

        setSolution(mVariablesX[route.back()][mInstance->GetDepotId()], 1.0);
    }

    double objVal = useSolution();
    CallbackTracker.HeuristicSolution++;
    std::cout << "Solution found with heuristic with " << routes.size() << " vehicles and costs "
              << std::to_string(objVal) << "\n";
}

void SubtourCallback::InjectSolution()
{
    throw std::runtime_error("Not implemented.");
    // NOLINTBEGIN(readability-magic-numbers)
    Collections::SequenceVector solution{{1, 22, 28},
                                         {4, 2},
                                         {6, 21, 36},
                                         {13, 19, 14},
                                         {20, 37, 5},
                                         {25, 18, 24, 23},
                                         {26, 11},
                                         {27, 15, 29, 30},
                                         {31, 39, 12},
                                         {32, 9},
                                         {33, 16, 3},
                                         {34, 8, 35, 7},
                                         {38, 10},
                                         {40, 17}};
    // NOLINTEND(readability-magic-numbers)

    SetHeuristicSolution(solution);
}

bool SubtourCallback1D::CheckRoutes()
{
    const Vehicle& vehicle = mInstance->Vehicles.front();
    const Container& container = vehicle.Containers.front();

    bool cutAdded = false;

    for (const auto& subtour: mSubtours)
    {
        CallbackTracker.Counter[CallbackElement::IntegerRoutes]++;

        if (subtour.Sequence.size() == 1)
        {
            CallbackTracker.Counter[CallbackElement::SingleCustomer]++;
            continue;
        }

        mClock.start();
        int minVehicles =
            mLoadingChecker->DetermineMinVehicles(mInputParameters->BranchAndCut.EnableMinVehicleLifting,
                                                  mInputParameters->BranchAndCut.MinVehicleLiftingThreshold,
                                                  container,
                                                  subtour.CustomersInRoute,
                                                  subtour.TotalWeight,
                                                  subtour.TotalVolume);
        mClock.end();
        CallbackTracker.UpdateElement(CallbackElement::MinNumVehicles, mClock.elapsed());

        if (subtour.ConnectedToDepot && minVehicles < 2)
        {
            mLoadingChecker->AddFeasibleSequenceFromOutside(subtour.Sequence);

            continue;
        }

        mClock.start();
        AddLazyConstraints({mLazyConstraintsGenerator->CreateConstraint(CutType::SEC, subtour.Sequence, minVehicles)});
        cutAdded = true;
        mClock.end();
        CallbackTracker.UpdateElement(CallbackElement::MinVehApproxInf, mClock.elapsed());

        cutAdded = true;
    }

    return !cutAdded;
}

bool SubtourCallback3D::CheckRoutes()
{
    const auto& vehicle = mInstance->Vehicles.front();
    const auto& container = vehicle.Containers.front();

    mCutAdded = false;

    for (const auto& subtour: mSubtours)
    {
        CallbackTracker.Counter[CallbackElement::IntegerRoutes]++;

        if (subtour.Sequence.size() == 1)
        {
            CallbackTracker.Counter[CallbackElement::SingleCustomer]++;
            continue;
        }

        mClock.start();
        int minVehicles =
            mLoadingChecker->DetermineMinVehicles(mInputParameters->BranchAndCut.EnableMinVehicleLifting,
                                                  mInputParameters->BranchAndCut.MinVehicleLiftingThreshold,
                                                  container,
                                                  subtour.CustomersInRoute,
                                                  subtour.TotalWeight,
                                                  subtour.TotalVolume);
        mClock.end();
        CallbackTracker.UpdateElement(CallbackElement::MinNumVehicles, mClock.elapsed());

        if (!subtour.ConnectedToDepot)
        {
            mClock.start();
            AddLazyConstraints(
                {mLazyConstraintsGenerator->CreateConstraint(CutType::SEC, subtour.Sequence, minVehicles)});
            mCutAdded = true;
            mClock.end();
            CallbackTracker.UpdateElement(CallbackElement::Disconnected, mClock.elapsed());

            continue;
        }

        CallbackTracker.Counter[CallbackElement::Connected]++;

        if (minVehicles == 1)
        {
            auto checkFunc = std::bind_front(&SubtourCallback3D::CheckSingleVehicleSubtour, this, subtour, container);
            auto [routeStatus, singleVehicleTime] = measure<>::durationWithReturn(checkFunc);
            CallbackTracker.UpdateElement(CallbackElement::SingleVehicle,
                                          static_cast<uint64_t>(singleVehicleTime.count()));

            if (routeStatus == LoadingStatus::Invalid)
            {
                mLogFile << "Optimization aborted due to invalid loading status!";
                this->abort();

                ////return false;
            }

            if (routeStatus == LoadingStatus::Infeasible)
            {
                mCutAdded = true;
            }
        }
        else
        {
            mClock.start();
            AddLazyConstraints(
                {mLazyConstraintsGenerator->CreateConstraint(CutType::SEC, subtour.Sequence, minVehicles)});
            mCutAdded = true;
            mClock.end();
            CallbackTracker.UpdateElement(CallbackElement::MinVehApproxInf, mClock.elapsed());
        }
    }

    return !mCutAdded;
}

LoadingStatus SubtourCallback3D::CheckSingleVehicleSubtour(const Subtour& subtour, Container& container)
{
    if (RouteCheckedAndFeasible(subtour.Sequence))
    {
        return LoadingStatus::FeasOpt;
    }

    if (CustomerCombinationInfeasible(subtour.Sequence, subtour.CustomersInRoute))
    {
        return LoadingStatus::Infeasible;
    }

    auto selectedItems = InterfaceConversions::SelectItems(subtour.Sequence, mInstance->Nodes, false);

    if (CheckRouteHeuristic(subtour.Sequence, container, selectedItems))
    {
        return LoadingStatus::FeasOpt;
    }

    return CheckRouteExact(subtour, container, selectedItems);
}

bool SubtourCallback3D::CheckRouteHeuristic(const Collections::IdVector& sequence,
                                            Container& container,
                                            std::vector<Cuboid>& items)
{
    if (!mInputParameters->BranchAndCut.ActivateHeuristic)
    {
        return false;
    }

    mClock.start();
    auto status = mLoadingChecker->PackingHeuristic(PackingType::Complete, container, sequence, items);

    if (status == LoadingStatus::FeasOpt)
    {
        LocalSearch::RunIntraImprovement(mInstance, mLoadingChecker, mInputParameters, sequence);
        mClock.end();
        CallbackTracker.UpdateElement(CallbackElement::HeuristicFeas, mClock.elapsed());
        return true;
    }

    mClock.end();
    CallbackTracker.UpdateElement(CallbackElement::HeuristicInf, mClock.elapsed());

    return false;
}

void SubtourCallback3D::CheckReversePath(const Collections::IdVector& sequence, Container& container)
{
    Collections::IdVector reversedSequence(sequence.size());
    std::reverse_copy(std::begin(sequence), std::end(sequence), std::begin(reversedSequence));

    if (mLoadingChecker->RouteIsInFeasSequences(reversedSequence))
    {
        return;
    }

    auto items = InterfaceConversions::SelectItems(reversedSequence, mInstance->Nodes, false);

    FunctionTimer timer;
    timer.start();

    auto heurStatus = mLoadingChecker->PackingHeuristic(PackingType::Complete, container, reversedSequence, items);

    if (heurStatus == LoadingStatus::FeasOpt)
    {
        LocalSearch::RunIntraImprovement(mInstance, mLoadingChecker, mInputParameters, reversedSequence);
        timer.end();
        CallbackTracker.UpdateElement(CallbackElement::RevHeurFeas, timer.elapsed());

        return;
    }

    timer.start();

    double maxRuntime = mInputParameters->DetermineMaxRuntime(BranchAndCutParams::CallType::ReversePath);
    auto cpStatus = mLoadingChecker->ConstraintProgrammingSolver(
        PackingType::Complete,
        container,
        boost::dynamic_bitset<>(),
        reversedSequence,
        items,
        mInputParameters->IsExact(BranchAndCutParams::CallType::ReversePath),
        maxRuntime);

    switch (cpStatus)
    {
        case LoadingStatus::FeasOpt:
        {
            LocalSearch::RunIntraImprovement(mInstance, mLoadingChecker, mInputParameters, reversedSequence);
            timer.end();
            CallbackTracker.UpdateElement(CallbackElement::RevExactFeas, timer.elapsed());

            break;
        }
        case LoadingStatus::Infeasible:
        {
            AddReversePathConstraints(sequence, reversedSequence);
            timer.end();
            CallbackTracker.UpdateElement(CallbackElement::RevExactInf, timer.elapsed());

            break;
        }
        default:
            break;
    }
}

LoadingStatus SubtourCallback3DAllSimple::CheckRouteExact(const Subtour& subtour,
                                                          Container& container,
                                                          std::vector<Cuboid>& items)
{
    // Solve complete CP model again if unknown to prove feasibility/infeasibility
    mClock.start();
    double maxRuntime = mInputParameters->DetermineMaxRuntime(BranchAndCutParams::CallType::Exact);

    auto exactStatus =
        mLoadingChecker->ConstraintProgrammingSolver(PackingType::Complete,
                                                     container,
                                                     subtour.CustomersInRoute,
                                                     subtour.Sequence,
                                                     items,
                                                     mInputParameters->IsExact(BranchAndCutParams::CallType::Exact),
                                                     maxRuntime);

    switch (exactStatus)
    {
        case LoadingStatus::FeasOpt:
            LocalSearch::RunIntraImprovement(mInstance, mLoadingChecker, mInputParameters, subtour.Sequence);
            mClock.end();
            CallbackTracker.UpdateElement(CallbackElement::ExactFeas, mClock.elapsed());

            return LoadingStatus::FeasOpt;
        case LoadingStatus::Infeasible:
            mClock.end();
            CallbackTracker.UpdateElement(CallbackElement::ExactInf, mClock.elapsed());

            break;
        case LoadingStatus::Invalid:
            mClock.end();
            CallbackTracker.UpdateElement(CallbackElement::ExactInvalid, mClock.elapsed());
            return LoadingStatus::Invalid;
        case LoadingStatus::Unknown:
            mClock.end();
            throw std::runtime_error("LoadingStatus is Unknown after exact CP model in CheckRouteExact().");
    }

    mClock.start();
    AddLazyConstraints({mLazyConstraintsGenerator->CreateConstraint(CutType::InfeasibleTailPath, subtour.Sequence)});
    mClock.end();
    CallbackTracker.UpdateElement(CallbackElement::InfeasibleTailPathInequality, mClock.elapsed());

    return LoadingStatus::Infeasible;
}

bool SubtourCallback3DAll::Lifting(const Subtour& subtour, Container& container, std::vector<Cuboid>& items)
{
    mClock.start();
    auto twoPathInequalities = mLazyConstraintsGenerator->TwoPathInequalityLifting(
        subtour.Sequence, subtour.CustomersInRoute, container, items);
    if (twoPathInequalities)
    {
        AddLazyConstraints(*twoPathInequalities);
        mClock.end();
        CallbackTracker.UpdateElement(CallbackElement::TwoPathInequality, mClock.elapsed());

        return true;
    }

    mClock.end();
    CallbackTracker.UpdateElement(CallbackElement::TwoPathInequalityNot, mClock.elapsed());

    mClock.start();
    auto regularPathInequalities = mLazyConstraintsGenerator->RegularPathLifting(subtour.Sequence, container, items);
    if (regularPathInequalities)
    {
        AddLazyConstraints(*regularPathInequalities);
        mClock.end();
        CallbackTracker.UpdateElement(CallbackElement::RegularPathInequality, mClock.elapsed());

        return true;
    }

    mClock.end();
    CallbackTracker.UpdateElement(CallbackElement::RegularPathInequalityNot, mClock.elapsed());

    return false;
}

LoadingStatus
    SubtourCallback3DAll::CheckRouteExact(const Subtour& subtour, Container& container, std::vector<Cuboid>& items)
{
    // Solve complete CP model with time limit
    // Try lifting although sequence might be feasible (status unknown with time limit)
    // Reasoning: feasibility can be proven quickly -> mabye lifting with relaxed problem is faster than solving
    // complete problem
    mClock.start();
    double maxRuntimeExactLimit = mInputParameters->DetermineMaxRuntime(BranchAndCutParams::CallType::ExactLimit);
    auto exactStatus = mLoadingChecker->ConstraintProgrammingSolver(
        PackingType::Complete,
        container,
        subtour.CustomersInRoute,
        subtour.Sequence,
        items,
        mInputParameters->IsExact(BranchAndCutParams::CallType::ExactLimit),
        maxRuntimeExactLimit);

    switch (exactStatus)
    {
        case LoadingStatus::FeasOpt:
            LocalSearch::RunIntraImprovement(mInstance, mLoadingChecker, mInputParameters, subtour.Sequence);
            mClock.end();
            CallbackTracker.UpdateElement(CallbackElement::ExactLimitFeas, mClock.elapsed());

            return LoadingStatus::FeasOpt;
        case LoadingStatus::Unknown:
            mClock.end();
            CallbackTracker.UpdateElement(CallbackElement::ExactLimitUnk, mClock.elapsed());

            break;
        case LoadingStatus::Infeasible:
            mClock.end();
            CallbackTracker.UpdateElement(CallbackElement::ExactLimitInf, mClock.elapsed());

            break;
        default:
            mClock.end();
            CallbackTracker.UpdateElement(CallbackElement::ExactInvalid, mClock.elapsed());
            return LoadingStatus::Invalid;
    }

    if (Lifting(subtour, container, items))
    {
        return LoadingStatus::Infeasible;
    }

    // Solve complete CP model again if unknown to prove feasibility/infeasibility
    mClock.start();
    if (exactStatus == LoadingStatus::Unknown)
    {
        double residualTime = mInputParameters->MIPSolver.TimeLimit - this->getDoubleInfo(GRB_CB_RUNTIME);
        double maxRuntime = mInputParameters->DetermineMaxRuntime(BranchAndCutParams::CallType::Exact, residualTime);

        exactStatus =
            mLoadingChecker->ConstraintProgrammingSolver(PackingType::Complete,
                                                         container,
                                                         subtour.CustomersInRoute,
                                                         subtour.Sequence,
                                                         items,
                                                         mInputParameters->IsExact(BranchAndCutParams::CallType::Exact),
                                                         maxRuntime);

        switch (exactStatus)
        {
            case LoadingStatus::FeasOpt:
                LocalSearch::RunIntraImprovement(mInstance, mLoadingChecker, mInputParameters, subtour.Sequence);
                mClock.end();
                CallbackTracker.UpdateElement(CallbackElement::ExactFeas, mClock.elapsed());
                return LoadingStatus::FeasOpt;
            case LoadingStatus::Infeasible:
                mClock.end();
                CallbackTracker.UpdateElement(CallbackElement::ExactInf, mClock.elapsed());
                break;
            case LoadingStatus::Invalid:
                mClock.end();
                CallbackTracker.UpdateElement(CallbackElement::ExactInvalid, mClock.elapsed());
                return LoadingStatus::Invalid;
            case LoadingStatus::Unknown:
                mClock.end();
                throw std::runtime_error("LoadingStatus is Unknown after exact CP model in CheckRouteExact().");
        }
    }

    mClock.start();
    AddLazyConstraints({mLazyConstraintsGenerator->CreateConstraint(CutType::TailTournament, subtour.Sequence)});
    mClock.end();
    CallbackTracker.UpdateElement(CallbackElement::TailPathInequality, mClock.elapsed());

    // Check reverse path to
    //   - create new feasible route, or
    //   - create stronger cuts.
    mClock.start();
    CheckReversePath(subtour.Sequence, container);
    mClock.end();
    CallbackTracker.UpdateElement(CallbackElement::ReverseSequence, mClock.elapsed());

    return LoadingStatus::Infeasible;
}

void SubtourCallback3DAll::AddReversePathConstraints(const Collections::IdVector& sequence,
                                                     const Collections::IdVector& reverseSequence)
{
    AddLazyConstraints({mLazyConstraintsGenerator->CreateConstraint(CutType::UndirectedTailPath, sequence),
                        mLazyConstraintsGenerator->CreateConstraint(CutType::TailTournament, reverseSequence)});
}

bool SubtourCallback3DNoSupport::Lifting(const Subtour& subtour, Container& container, std::vector<Cuboid>& items)
{
    mClock.start();
    auto twoPathInequalities = mLazyConstraintsGenerator->TwoPathInequalityLifting(
        subtour.Sequence, subtour.CustomersInRoute, container, items);
    if (twoPathInequalities)
    {
        AddLazyConstraints(*twoPathInequalities);
        mClock.end();
        CallbackTracker.UpdateElement(CallbackElement::TwoPathInequality, mClock.elapsed());

        return true;
    }

    mClock.end();
    CallbackTracker.UpdateElement(CallbackElement::TwoPathInequalityNot, mClock.elapsed());

    return false;
}

LoadingStatus SubtourCallback3DNoSupport::CheckRouteExact(const Subtour& subtour,
                                                          Container& container,
                                                          std::vector<Cuboid>& items)
{
    mClock.start();
    double maxRuntimeExactLimit = mInputParameters->DetermineMaxRuntime(BranchAndCutParams::CallType::ExactLimit);
    auto exactStatus = mLoadingChecker->ConstraintProgrammingSolver(
        PackingType::Complete,
        container,
        subtour.CustomersInRoute,
        subtour.Sequence,
        items,
        mInputParameters->IsExact(BranchAndCutParams::CallType::ExactLimit),
        maxRuntimeExactLimit);

    switch (exactStatus)
    {
        case LoadingStatus::FeasOpt:
            LocalSearch::RunIntraImprovement(mInstance, mLoadingChecker, mInputParameters, subtour.Sequence);
            mClock.end();
            CallbackTracker.UpdateElement(CallbackElement::ExactLimitFeas, mClock.elapsed());

            return LoadingStatus::FeasOpt;
        case LoadingStatus::Unknown:
            mClock.end();
            CallbackTracker.UpdateElement(CallbackElement::ExactLimitUnk, mClock.elapsed());

            break;
        case LoadingStatus::Infeasible:
            mClock.end();
            CallbackTracker.UpdateElement(CallbackElement::ExactLimitInf, mClock.elapsed());

            break;
        default:
            mClock.end();
            CallbackTracker.UpdateElement(CallbackElement::ExactInvalid, mClock.elapsed());
            return LoadingStatus::Invalid;
    }

    if (Lifting(subtour, container, items))
    {
        return LoadingStatus::Infeasible;
    }

    mClock.start();
    if (exactStatus == LoadingStatus::Unknown)
    {
        double residualTime = mInputParameters->MIPSolver.TimeLimit - this->getDoubleInfo(GRB_CB_RUNTIME);
        double maxRuntime = mInputParameters->DetermineMaxRuntime(BranchAndCutParams::CallType::Exact, residualTime);

        exactStatus =
            mLoadingChecker->ConstraintProgrammingSolver(PackingType::Complete,
                                                         container,
                                                         subtour.CustomersInRoute,
                                                         subtour.Sequence,
                                                         items,
                                                         mInputParameters->IsExact(BranchAndCutParams::CallType::Exact),
                                                         maxRuntime);

        switch (exactStatus)
        {
            case LoadingStatus::FeasOpt:
                LocalSearch::RunIntraImprovement(mInstance, mLoadingChecker, mInputParameters, subtour.Sequence);
                mClock.end();
                CallbackTracker.UpdateElement(CallbackElement::ExactFeas, mClock.elapsed());

                return LoadingStatus::FeasOpt;
            case LoadingStatus::Infeasible:
                mClock.end();
                CallbackTracker.UpdateElement(CallbackElement::ExactInf, mClock.elapsed());

                break;
            case LoadingStatus::Invalid:
                mClock.end();
                CallbackTracker.UpdateElement(CallbackElement::ExactInvalid, mClock.elapsed());
                return LoadingStatus::Invalid;
            case LoadingStatus::Unknown:
                mClock.end();
                throw std::runtime_error("LoadingStatus is Unknown after exact CP model in CheckRouteExact().");
        }
    }

    mClock.start();
    AddLazyConstraints(mLazyConstraintsGenerator->CreateRegularPathCuts(subtour.Sequence, container));
    mClock.end();
    CallbackTracker.UpdateElement(CallbackElement::RegularPathInequality, mClock.elapsed());

    mClock.start();
    CheckReversePath(subtour.Sequence, container);
    mClock.end();
    CallbackTracker.UpdateElement(CallbackElement::ReverseSequence, mClock.elapsed());

    return LoadingStatus::Infeasible;
}

void SubtourCallback3DNoSupport::AddReversePathConstraints(const Collections::IdVector& sequence,
                                                           const Collections::IdVector& reverseSequence)
{
    AddLazyConstraints({mLazyConstraintsGenerator->CreateConstraint(CutType::UndirectedPath, sequence),
                        mLazyConstraintsGenerator->CreateConstraint(CutType::RegularPath, reverseSequence)});
}

bool SubtourCallback3DNoLIFO::Lifting(const Subtour& subtour, Container& container, std::vector<Cuboid>& items)
{
    mClock.start();
    auto twoPathInequalities = mLazyConstraintsGenerator->TwoPathInequalityLifting(
        subtour.Sequence, subtour.CustomersInRoute, container, items);
    if (twoPathInequalities.has_value())
    {
        AddLazyConstraints(twoPathInequalities.value());
        mClock.end();
        CallbackTracker.UpdateElement(CallbackElement::TwoPathInequality, mClock.elapsed());

        return true;
    }

    mClock.end();
    CallbackTracker.UpdateElement(CallbackElement::TwoPathInequalityNot, mClock.elapsed());

    return false;
}

LoadingStatus
    SubtourCallback3DNoLIFO::CheckRouteExact(const Subtour& subtour, Container& container, std::vector<Cuboid>& items)
{
    using enum BranchAndCutParams::CallType;

    double residualTime = mInputParameters->MIPSolver.TimeLimit - this->getDoubleInfo(GRB_CB_RUNTIME);

    mClock.start();

    auto callType = mCutAdded ? ExactLimit : Exact;
    double maxRuntime = mInputParameters->DetermineMaxRuntime(callType, residualTime);

    auto exactStatus = mLoadingChecker->ConstraintProgrammingSolver(PackingType::Complete,
                                                                    container,
                                                                    subtour.CustomersInRoute,
                                                                    subtour.Sequence,
                                                                    items,
                                                                    mInputParameters->IsExact(callType),
                                                                    maxRuntime);

    auto element = CallbackElement::None;

    switch (exactStatus)
    {
        case LoadingStatus::FeasOpt:
            LocalSearch::RunIntraImprovement(mInstance, mLoadingChecker, mInputParameters, subtour.Sequence);
            mClock.end();
            element = callType == Exact ? CallbackElement::ExactFeas : CallbackElement::ExactLimitFeas;

            CallbackTracker.UpdateElement(element, mClock.elapsed());

            return LoadingStatus::FeasOpt;
        case LoadingStatus::Infeasible:
            mClock.end();
            element = callType == Exact ? CallbackElement::ExactInf : CallbackElement::ExactLimitInf;
            CallbackTracker.UpdateElement(element, mClock.elapsed());

            break;
        case LoadingStatus::Unknown:
            if (callType == ExactLimit)
            {
                mClock.end();
                CallbackTracker.UpdateElement(CallbackElement::ExactLimitUnk, mClock.elapsed());

                return LoadingStatus::Unknown;
            }
        default:
            mClock.end();
            CallbackTracker.UpdateElement(CallbackElement::ExactInvalid, mClock.elapsed());
            return LoadingStatus::Invalid;
    }

    if (Lifting(subtour, container, items))
    {
        return LoadingStatus::Infeasible;
    }

    AddLazyConstraints({mLazyConstraintsGenerator->CreateConstraint(CutType::TwoPathTail, subtour.Sequence)});

    return LoadingStatus::Infeasible;
}

void SubtourCallback3DNoLIFO::AddReversePathConstraints(const Collections::IdVector& sequence [[maybe_unused]],
                                                        const Collections::IdVector& reverseSequence [[maybe_unused]])
{
    // No constraint must be added.
}

bool SubtourCallback3DLoadingOnly::Lifting(const Subtour& subtour,
                                           Container& container,
                                           std::vector<Cuboid>& items [[maybe_unused]])
{
    mClock.start();
    auto twoPathInequalities =
        mLazyConstraintsGenerator->CreateTwoPathCuts(subtour.Sequence, subtour.CustomersInRoute, container);
    AddLazyConstraints(twoPathInequalities);
    mClock.end();
    CallbackTracker.UpdateElement(CallbackElement::TwoPathInequality, mClock.elapsed());

    return true;
}

LoadingStatus SubtourCallback3DLoadingOnly::CheckRouteExact(const Subtour& subtour,
                                                            Container& container,
                                                            std::vector<Cuboid>& items)
{
    using enum BranchAndCutParams::CallType;

    double residualTime = mInputParameters->MIPSolver.TimeLimit - this->getDoubleInfo(GRB_CB_RUNTIME);
    mClock.start();

    auto callType = mCutAdded ? ExactLimit : Exact;
    double maxRuntime = mInputParameters->DetermineMaxRuntime(callType, residualTime);

    auto exactStatus = mLoadingChecker->ConstraintProgrammingSolver(PackingType::Complete,
                                                                    container,
                                                                    subtour.CustomersInRoute,
                                                                    subtour.Sequence,
                                                                    items,
                                                                    mInputParameters->IsExact(callType),
                                                                    maxRuntime);

    auto element = CallbackElement::None;

    switch (exactStatus)
    {
        case LoadingStatus::FeasOpt:
            LocalSearch::RunIntraImprovement(mInstance, mLoadingChecker, mInputParameters, subtour.Sequence);
            mClock.end();
            element = callType == Exact ? CallbackElement::ExactFeas : CallbackElement::ExactLimitFeas;
            CallbackTracker.UpdateElement(element, mClock.elapsed());

            return LoadingStatus::FeasOpt;
        case LoadingStatus::Infeasible:
            mClock.end();
            element = callType == Exact ? CallbackElement::ExactInf : CallbackElement::ExactLimitInf;
            CallbackTracker.UpdateElement(element, mClock.elapsed());

            break;
        case LoadingStatus::Unknown:
            if (callType == ExactLimit)
            {
                mClock.end();
                CallbackTracker.UpdateElement(CallbackElement::ExactLimitUnk, mClock.elapsed());

                return LoadingStatus::Unknown;
            }
        default:
            mClock.end();
            CallbackTracker.UpdateElement(CallbackElement::ExactInvalid, mClock.elapsed());
            return LoadingStatus::Invalid;
    }

    Lifting(subtour, container, items);

    return LoadingStatus::Infeasible;
}

void SubtourCallback3DLoadingOnly::AddReversePathConstraints(const Collections::IdVector& sequence [[maybe_unused]],
                                                             const Collections::IdVector& reverseSequence
                                                             [[maybe_unused]])
{
    // No cut must be added.
}

std::unique_ptr<SubtourCallback> CallbackFactory::CreateCallback(LoadingProblemParams::VariantType variant,
                                                                 GRBEnv* env,
                                                                 GRBVar2D& vars,
                                                                 const Instance* const instance,
                                                                 LoadingChecker* loadingChecker,
                                                                 const InputParameters* const inputParameters,
                                                                 std::string& outputPath)
{
    std::unique_ptr<SubtourCallback> callback = nullptr;

    switch (variant)
    {
        case LoadingProblemParams::VariantType::Volume:
        case LoadingProblemParams::VariantType::Weight:
        case LoadingProblemParams::VariantType::VolumeWeightApproximation:
            callback = std::make_unique<SubtourCallback1D>(vars, instance, loadingChecker, inputParameters, outputPath);
            break;
        case LoadingProblemParams::VariantType::LoadingOnly:
            callback = std::make_unique<SubtourCallback3DLoadingOnly>(
                vars, instance, loadingChecker, inputParameters, outputPath);
            break;
        case LoadingProblemParams::VariantType::NoLifo:
            callback =
                std::make_unique<SubtourCallback3DNoLIFO>(vars, instance, loadingChecker, inputParameters, outputPath);
            break;
        case LoadingProblemParams::VariantType::NoSupport:
            callback = std::make_unique<SubtourCallback3DNoSupport>(
                vars, instance, loadingChecker, inputParameters, outputPath);
            break;
        case LoadingProblemParams::VariantType::NoFragility:
            [[fallthrough]];
        case LoadingProblemParams::VariantType::AllConstraints:
            callback =
                std::make_unique<SubtourCallback3DAll>(vars, instance, loadingChecker, inputParameters, outputPath);
            break;
        default:
            throw std::runtime_error("Unknown variant.");
    }

    if (inputParameters->BranchAndCut.SimpleVersion)
    {
        if (variant != LoadingProblemParams::VariantType::AllConstraints)
        {
            throw std::runtime_error("Simple branch-and-cut version only for the all constraint variant implemented.");
        }

        callback =
            std::make_unique<SubtourCallback3DAllSimple>(vars, instance, loadingChecker, inputParameters, outputPath);
    }

    if (inputParameters->BranchAndCut.ActivateSetPartitioningHeuristic)
    {
        callback->SPHeuristic =
            std::make_unique<Heuristics::SetBased::SPHeuristic>(instance, loadingChecker, inputParameters, env);
    }

    return callback;
}

}
}
