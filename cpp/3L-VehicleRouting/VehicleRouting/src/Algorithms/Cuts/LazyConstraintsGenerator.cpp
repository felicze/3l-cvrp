#include "Algorithms/Cuts/LazyConstraintsGenerator.h"

#include "CommonBasics/Helper/ModelServices.h"
#include "Helper/Timer.h"

#include "Algorithms/Evaluation.h"
#include "Algorithms/Heuristics/LocalSearch.h"
#include "Algorithms/Heuristics/TwoOpt.h"
#include "Algorithms/LoadingInterfaceServices.h"

namespace VehicleRouting
{
namespace Algorithms
{
using namespace Heuristics::Improvement;

namespace Cuts
{
std::optional<std::vector<Cut>>
    LazyConstraintsGenerator::TwoPathInequalityLifting(const Collections::IdVector& sequence,
                                                       const boost::dynamic_bitset<>& set,
                                                       Container& container,
                                                       std::vector<Cuboid>& items)
{
    double maxRuntime = mInputParameters->DetermineMaxRuntime(BranchAndCutParams::CallType::TwoPath);
    auto status =
        mLoadingChecker->ConstraintProgrammingSolver(PackingType::LifoNoSequence,
                                                     container,
                                                     set,
                                                     sequence,
                                                     items,
                                                     mInputParameters->IsExact(BranchAndCutParams::CallType::TwoPath),
                                                     maxRuntime);

    if (status != LoadingStatus::Infeasible)
    {
        return std::nullopt;
    }

    return CreateTwoPathCuts(sequence, set, container);
}

std::optional<std::vector<Cut>> LazyConstraintsGenerator::RegularPathLifting(const Collections::IdVector& sequence,
                                                                             Container& container,
                                                                             std::vector<Cuboid>& items)
{
    double maxRuntime = mInputParameters->DetermineMaxRuntime(BranchAndCutParams::CallType::RegularPath);
    auto status = mLoadingChecker->ConstraintProgrammingSolver(
        PackingType::NoSupport,
        container,
        boost::dynamic_bitset<>(),
        sequence,
        items,
        mInputParameters->IsExact(BranchAndCutParams::CallType::RegularPath),
        maxRuntime);

    if (status != LoadingStatus::Infeasible)
    {
        return std::nullopt;
    }

    return CreateRegularPathCuts(sequence, container);
}

std::optional<Collections::IdVector>
    LazyConstraintsGenerator::DetermineMinimalInfeasibleSubset(const Collections::IdVector& sequence,
                                                               boost::dynamic_bitset<>& set,
                                                               const Container& container)
{
    std::vector<std::pair<double, size_t>> volumes;
    for (const auto nodeI: sequence)
    {
        volumes.push_back(std::make_pair(mInstance->Nodes[nodeI].TotalVolume, nodeI));
    }

    std::ranges::sort(volumes);

    Collections::IdVector sortedSubset;
    for (size_t i = 0; i < sequence.size(); ++i)
    {
        sortedSubset.push_back(volumes[i].second);
    }

    size_t indexLastNode = 0;

    while (indexLastNode < sortedSubset.size())
    {
        auto subSet =
            std::vector(std::begin(sortedSubset) + static_cast<long>(indexLastNode) + 1, std::end(sortedSubset));

        if (mLoadingChecker->RouteIsInFeasSequences(subSet))
        {
            break;
        }

        auto subSetItems = InterfaceConversions::SelectItems(subSet, mInstance->Nodes, false);

        auto heuristicStatus = mLoadingChecker->PackingHeuristic(PackingType::Complete, container, subSet, subSetItems);

        if (heuristicStatus == LoadingStatus::FeasOpt)
        {
            LocalSearch::RunIntraImprovement(mInstance, mLoadingChecker, mInputParameters, subSet);
            break;
        }

        set.reset(sortedSubset[indexLastNode]); // remove node from set

        double maxRuntime = mInputParameters->DetermineMaxRuntime(BranchAndCutParams::CallType::MinInfSet);
        auto cpStatus = mLoadingChecker->ConstraintProgrammingSolver(
            PackingType::LifoNoSequence,
            container,
            set,
            subSet,
            subSetItems,
            mInputParameters->IsExact(BranchAndCutParams::CallType::MinInfSet),
            maxRuntime);

        if (cpStatus == LoadingStatus::Infeasible)
        {
            indexLastNode++;
            continue;
        }

        set.set(sortedSubset[indexLastNode]); // add node to set again
        break;
    }

    if (indexLastNode == 0)
    {
        return std::nullopt;
    }

    return std::vector(std::begin(sortedSubset) + static_cast<long>(indexLastNode), std::end(sortedSubset));
}

std::optional<Collections::IdVector>
    LazyConstraintsGenerator::DetermineMinimalInfeasibleSubPath(const Collections::IdVector& sequence,
                                                                const Container& container,
                                                                bool fromFront)
{
    size_t indexLastNode = 0;

    while (indexLastNode < sequence.size())
    {
        Collections::IdVector subPath =
            fromFront ? std::vector(std::begin(sequence) + static_cast<long>(indexLastNode) + 1, std::end(sequence))
                      : std::vector(std::begin(sequence), std::end(sequence) - 1 - static_cast<long>(indexLastNode));

        if (mLoadingChecker->RouteIsInFeasSequences(subPath))
        {
            break;
        }

        auto items = InterfaceConversions::SelectItems(subPath, mInstance->Nodes, false);

        auto heuristicStatus = mLoadingChecker->PackingHeuristic(PackingType::Complete, container, subPath, items);

        if (heuristicStatus == LoadingStatus::FeasOpt)
        {
            LocalSearch::RunIntraImprovement(mInstance, mLoadingChecker, mInputParameters, subPath);
            break;
        }

        double maxRuntime = mInputParameters->DetermineMaxRuntime(BranchAndCutParams::CallType::MinInfPath);
        auto cpStatus = mLoadingChecker->ConstraintProgrammingSolver(
            PackingType::NoSupport,
            container,
            mLoadingChecker->MakeBitset(mInstance->Nodes.size(), subPath),
            subPath,
            items,
            mInputParameters->IsExact(BranchAndCutParams::CallType::MinInfPath),
            maxRuntime);

        ////logFile << "MIFP " << std::to_string((int)status) << "\n";

        if (cpStatus == LoadingStatus::Infeasible)
        {
            indexLastNode++;
            continue;
        }

        if (!mInputParameters->ContainerLoading.LoadingProblem.EnableSupport && cpStatus == LoadingStatus::FeasOpt)
        {
            LocalSearch::RunIntraImprovement(mInstance, mLoadingChecker, mInputParameters, subPath);
        }

        break;
    }

    if (indexLastNode == 0)
    {
        return std::nullopt;
    }

    return fromFront ? std::vector(std::begin(sequence) + static_cast<long>(indexLastNode), std::end(sequence))
                     : std::vector(std::begin(sequence), std::end(sequence) - static_cast<long>(indexLastNode));
}

std::vector<Cut> LazyConstraintsGenerator::CreateTwoPathCuts(const Collections::IdVector& sequence,
                                                             const boost::dynamic_bitset<>& set,
                                                             const Container& container)
{
    auto tmpSet = set;

    auto cuts = std::vector<Cut>{CreateConstraint(CutType::TwoPath, sequence, 2)};

    auto subSet = DetermineMinimalInfeasibleSubset(sequence, tmpSet, container);
    if (subSet.has_value())
    {
        cuts.emplace_back(CreateConstraint(CutType::TwoPathMIS, subSet.value(), 2));
    }

    mLoadingChecker->AddInfeasibleCombination(tmpSet);

    return cuts;
}

std::vector<Cut> LazyConstraintsGenerator::CreateRegularPathCuts(const Collections::IdVector& sequence,
                                                                 Container& container)
{
    std::vector<Cut> cuts;
    cuts.reserve(3);

    auto minimalPathFromFront = DetermineMinimalInfeasibleSubPath(sequence, container, true);
    if (minimalPathFromFront.has_value())
    {
        cuts.emplace_back(CreateConstraint(CutType::RegularPathFront, minimalPathFromFront.value()));
    }

    auto minimalPathFromBack = DetermineMinimalInfeasibleSubPath(sequence, container, false);
    if (minimalPathFromBack.has_value() && (minimalPathFromFront != minimalPathFromBack))
    {
        cuts.emplace_back(CreateConstraint(CutType::RegularPathBack, minimalPathFromBack.value()));
    }

    if (!minimalPathFromFront.has_value() && !minimalPathFromBack.has_value())
    {
        cuts.emplace_back(CreateConstraint(CutType::RegularPath, sequence));
    }

    return cuts;
}

Cut LazyConstraintsGenerator::CreateConstraint(CutType type,
                                               const Collections::IdVector& sequence,
                                               int minNumberVehicles)
{
    switch (type)
    {
        case CutType::TwoPathMIS:
            // Fallthrough
        case CutType::TwoPath:
            // Fallthrough
        case CutType::SEC:
            assert(minNumberVehicles != 0);
            return CreateSubtourEliminationConstraint(type, sequence, minNumberVehicles);
        case CutType::TwoPathTail:
            return CreateTwoPathTailConstraint(type, sequence);
        case CutType::RegularPathFront:
            // Fallthrough
        case CutType::RegularPathBack:
            // Fallthrough
        case CutType::RegularPath:
            return CreateTournamentConstraint(type, sequence);
        case CutType::TailTournament:
            return CreateTailTournamentConstraint(type, sequence);
        case CutType::UndirectedPath:
            return CreateUndirectedInfeasiblePathConstraint(type, sequence);
        case CutType::UndirectedTailPath:
            return CreateUndirectedInfeasibleTailPathConstraint(type, sequence);
        case CutType::InfeasibleTailPath:
            return CreateInfeasibleTailPathConstraint(type, sequence);
        default:
            throw std::runtime_error("Cut type not implemented!");
    }
}

Cut LazyConstraintsGenerator::CreateSubtourEliminationConstraint(CutType type,
                                                                 const Collections::IdVector& sequence,
                                                                 int minNumberVehicles)
{
    /*
    bool printLazyConstraintsToConsole = false;

    if (printLazyConstraintsToConsole)
    {
        std::vector<int> sequenceCopy;
        for (int node: sequence)
        {
            sequenceCopy.push_back(node);
        }
        std::ranges::sort(sequenceCopy);

        std::cout << "Lazy constraint SEC: ";
        for (int node: sequenceCopy)
        {
            std::cout << node << " | ";
        }
        std::cout << "minV: " << minNumberVehicles << "\n";
    }
    */

    auto cut = Cut(type);
    for (size_t i = 0; i < sequence.size() - 1; ++i)
    {
        const auto nodeI = sequence[i];
        for (size_t j = i + 1; j < sequence.size(); ++j)
        {
            const auto nodeJ = sequence[j];
            const auto valueIJ = (*mXValues)[nodeI][nodeJ];
            const auto valueJI = (*mXValues)[nodeJ][nodeI];
            cut.AddArc(-1.0, nodeI, nodeJ, valueIJ);
            cut.AddArc(-1.0, nodeJ, nodeI, valueJI);
        }
    }

    cut.RHS = -((int)sequence.size() - minNumberVehicles);

    return cut;
}

Cut LazyConstraintsGenerator::CreateTwoPathTailConstraint(CutType type, const Collections::IdVector& sequence)
{
    auto cut = Cut(type);

    // arcs in set S
    auto inSet = boost::dynamic_bitset<>(mInstance->Nodes.size());
    for (size_t i = 0; i < sequence.size() - 1; ++i)
    {
        const auto nodeI = sequence[i];
        for (size_t j = i + 1; j < sequence.size(); ++j)
        {
            const auto nodeJ = sequence[j];
            const auto valueIJ = (*mXValues)[nodeI][nodeJ];
            const auto valueJI = (*mXValues)[nodeJ][nodeI];
            cut.AddArc(-1.0, nodeI, nodeJ, valueIJ);
            cut.AddArc(-1.0, nodeJ, nodeI, valueJI);
        }

        inSet.set(nodeI);
    }
    inSet.set(sequence.back());

    // arcs between C\S (set difference of set of customers and S) and S
    for (size_t i = 1, n = mInstance->Nodes.size(); i < n; ++i)
    {
        if (inSet[i])
        {
            continue;
        }

        for (auto nodeJ: sequence)
        {
            const double valueij = (*mXValues)[i][nodeJ];
            const double valueji = (*mXValues)[nodeJ][i];

            cut.AddArc(1.0, i, nodeJ, valueij);
            cut.AddArc(1.0, nodeJ, i, valueji);
        }
    }

    cut.RHS = -((int)sequence.size() - 2);

    return cut;
}

Cut LazyConstraintsGenerator::CreateTournamentConstraint(CutType type, const Collections::IdVector& sequence)
{
    auto cut = Cut(type);
    for (size_t i = 0; i < sequence.size() - 1; ++i)
    {
        const auto nodeI = sequence[i];
        for (size_t j = i + 1; j < sequence.size(); ++j)
        {
            const auto nodeJ = sequence[j];
            const auto value = (*mXValues)[nodeI][nodeJ];
            cut.AddArc(-1.0, nodeI, nodeJ, value);
        }
    }

    cut.RHS = -((int)sequence.size() - 2);

    return cut;
}

Cut LazyConstraintsGenerator::CreateTailTournamentConstraint(CutType type, const Collections::IdVector& sequence)
{
    /*
    std::vector<int> sequenceCopy;
    for (int node : sequence)
    {
        sequenceCopy.push_back(node);
    }
    std::ranges::sort(sequenceCopy);
    std::cout << "Lazy constraint TTC: ";
    for (int node : sequenceCopy)
    {
        std::cout << node << " | ";
    }
    std::cout << "minV: " << 1 << "\n";
    */

    auto cut = Cut(type);

    /* Variant 1
     * Consider arcs to depot
     */

    for (size_t i = 0; i < sequence.size() - 1; ++i)
    {
        const auto nodeI = sequence[i];
        for (size_t j = i + 1; j < sequence.size(); ++j)
        {
            const auto nodeJ = sequence[j];
            const auto value = (*mXValues)[nodeI][nodeJ];
            cut.AddArc(-1.0, nodeI, nodeJ, value);
        }
    }

    for (size_t i = 0; i < sequence.size(); ++i)
    {
        const auto nodeI = sequence[i];
        const auto value = (*mXValues)[nodeI][0];
        cut.AddArc(-0.5, nodeI, 0, value);
    }

    cut.RHS = -((int)sequence.size() - 1);

    /* Variant 2
     * Consider arcs to customer nodes outside of S
     * inferior in preliminary tests
     */
    /*
    boost::dynamic_bitset<> inSet(mInstance->Nodes.size());
    for (int i = 0; i < sequence.size() - 1; ++i)
    {
        const int nodeI = sequence[i];
        for (int j = i + 1; j < sequence.size(); ++j)
        {
            const int nodeJ = sequence[j];
            const double value = (*mXValues)[nodeI][nodeJ];
            cut.AddArc(-1.0, nodeI, nodeJ, value);
        }

        inSet.set(nodeI);
    }

    int lastNode = sequence.back();
    inSet.set(lastNode);
    for (size_t i = 1, n = mInstance->Nodes.size(); i < n; ++i)
    {
        if (inSet[i])
            continue;

        const double value = (*mXValues)[lastNode][i];
        cut.AddArc(1.0, lastNode, i, value);
    }

    cut.RHS = -((int)sequence.size() - 2);
    */

    return cut;
}

Cut LazyConstraintsGenerator::CreateUndirectedInfeasiblePathConstraint(CutType type,
                                                                       const Collections::IdVector& sequence)
{
    auto cut = Cut(type);

    for (size_t i = 0; i < sequence.size() - 1; ++i)
    {
        const auto nodeI = sequence[i];
        const auto nodeJ = sequence[i + 1];
        const auto valueIJ = (*mXValues)[nodeI][nodeJ];
        const auto valueJI = (*mXValues)[nodeJ][nodeI];
        cut.AddArc(-1.0, nodeI, nodeJ, valueIJ);
        cut.AddArc(-1.0, nodeJ, nodeI, valueJI);
    }

    cut.RHS = -((int)sequence.size() - 2);

    return cut;
}

Cut LazyConstraintsGenerator::CreateUndirectedInfeasibleTailPathConstraint(CutType type,
                                                                           const Collections::IdVector& sequence)
{
    auto cut = Cut(type);

    /* Variant 1
     * Consider arcs to depot
     */

    for (size_t i = 0; i < sequence.size() - 1; ++i)
    {
        const auto nodeI = sequence[i];
        const auto nodeJ = sequence[i + 1];

        cut.AddArc(-1.0, nodeI, nodeJ, 1);
        cut.AddArc(-1.0, nodeJ, nodeI, 0);
    }

    for (const auto node: sequence)
    {
        cut.AddArc(-0.5, node, 0, 0);
    }

    cut.RHS = -((int)sequence.size() - 1);

    /* Variant 2
    * Consider arcs to customer nodes outside of S
    * inferior in preliminary tests

    boost::dynamic_bitset<> inSet(mInstance->Nodes.size());
    for (int i = 0; i < sequence.size() - 1; ++i)
    {
        int nodeI = sequence[i];
        int nodeJ = sequence[i + 1];

        cut.AddArc(-1.0, nodeI, nodeJ, 1);
        cut.AddArc(-1.0, nodeJ, nodeI, 0);

        inSet.set(nodeI);
    }

    int firstNode = sequence.front();
    int lastNode = sequence.back();
    inSet.set(lastNode);

    for (size_t i = 1, n = mInstance->Nodes.size(); i < n; ++i)
    {
        if (inSet[i])
            continue;

        const double valueLast = (*mXValues)[lastNode][i];
        cut.AddArc(1.0, lastNode, i, valueLast);
        const double valueFirst = (*mXValues)[firstNode][i];
        cut.AddArc(1.0, firstNode, i, valueFirst);
    }

    cut.RHS = -((int)sequence.size() - 2);
    */

    return cut;
}

Cut LazyConstraintsGenerator::CreateInfeasibleTailPathConstraint(CutType type, const Collections::IdVector& sequence)
{
    auto cut = Cut(type);

    // Consider arc to depot
    for (size_t i = 0; i < sequence.size() - 1; ++i)
    {
        const auto nodeI = sequence[i];
        const auto nodeJ = sequence[i + 1];

        cut.AddArc(-1.0, nodeI, nodeJ, 1);
    }

    cut.AddArc(-1.0, sequence.back(), 0, 1);

    cut.RHS = -((int)sequence.size() - 1);
    cut.CalcViolation();

    return cut;
}

}
}
}