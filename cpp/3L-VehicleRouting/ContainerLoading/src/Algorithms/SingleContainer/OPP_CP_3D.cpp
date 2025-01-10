#include "Algorithms/SingleContainer/OPP_CP_3D.h"

#include <algorithm>
#include <fstream>
#include <iostream>
#include <ostream>
#include <ranges>
#include <stdexcept>
#include <string>

namespace ContainerLoading
{
using namespace Model;

namespace Algorithms
{
void ContainerLoadingCP::BuildModel()
{
    CreateVariables();

    AddConstraints();

    ////AddObjective();
}

LoadingStatus ContainerLoadingCP::Solve()
{
    BuildModel();

    operations_research::sat::SatParameters parameters;
    SetParameters(parameters);

    operations_research::sat::Model model = operations_research::sat::Model();
    model.Add(operations_research::sat::NewSatParameters(parameters));

    operations_research::sat::CpModelProto protoModel = mModelCP.Build();
    ////auto validationResponse = operations_research::sat::ValidateCpModel(protoModel);
    ////LOG(INFO) << validationResponse;

    mResponse = operations_research::sat::SolveCpModel(protoModel, &model);

    ////LOG(INFO) << operations_research::sat::CpSolverResponseStats(mResponse);

    ////PrintSolution(Items, mResponse);
    switch (mResponse.status())
    {
        case operations_research::sat::OPTIMAL:
            [[fallthrough]];
        case operations_research::sat::FEASIBLE:
            return LoadingStatus::FeasOpt;
        case operations_research::sat::INFEASIBLE:
            return LoadingStatus::Infeasible;
        case operations_research::sat::UNKNOWN:
            return LoadingStatus::Unknown;
        default:
            return LoadingStatus::Invalid;
    }
}

void ContainerLoadingCP::WriteProtoModel(const operations_research::sat::CpModelProto& protoModel) const
{
    std::string protoModelString = protoModel.DebugString();
    std::ofstream file("protoModel_basicDomain.txt");
    file << protoModelString;
    file.close();
}

void ContainerLoadingCP::SetParameters(operations_research::sat::SatParameters& parameters) const
{
    parameters.set_num_search_workers(mParams.Threads);
    parameters.set_log_search_progress(mParams.LogFlag);
    ////parameters.set_search_branching(parameters.PORTFOLIO_SEARCH);
    parameters.set_max_time_in_seconds(mMaxRuntime);
    // Setting seed value is without effect for parallel mode
    // https://github.com/google/or-tools/issues/2793
    ////parameters.set_random_seed(mParams.Seed);

    ////parameters.set_cp_model_presolve(false);

    // Example how to set decision strategies, but not helpful here.
    /*
    cp_model.AddDecisionStrategy(mStartPositionsY,
        operations_research::sat::DecisionStrategyProto::VariableSelectionStrategy::DecisionStrategyProto_VariableSelectionStrategy_CHOOSE_LOWEST_MIN,
        operations_research::sat::DecisionStrategyProto::DomainReductionStrategy::DecisionStrategyProto_DomainReductionStrategy_SELECT_MIN_VALUE);
    mModelCP.AddDecisionStrategy(mStartPositionsX,
        operations_research::sat::DecisionStrategyProto::VariableSelectionStrategy::DecisionStrategyProto_VariableSelectionStrategy_CHOOSE_LOWEST_MIN,
        operations_research::sat::DecisionStrategyProto::DomainReductionStrategy::DecisionStrategyProto_DomainReductionStrategy_SELECT_MIN_VALUE);
    cp_model.AddDecisionStrategy(mStartPositionsZ,
        operations_research::sat::DecisionStrategyProto::VariableSelectionStrategy::DecisionStrategyProto_VariableSelectionStrategy_CHOOSE_LOWEST_MIN,
        operations_research::sat::DecisionStrategyProto::DomainReductionStrategy::DecisionStrategyProto_DomainReductionStrategy_SELECT_MIN_VALUE);
        */

    // Example how to set solution observer
    /*
    auto observer = operations_research::sat::NewFeasibleSolutionObserver([&](const
    operations_research::sat::CpSolverResponse mResponse) { LOG(INFO) << "Length " <<
    operations_research::sat::SolutionIntegerValue(mResponse, mMaxLength); LOG(INFO) << "LB " <<
    mResponse.best_objective_bound();
        });
    model.Add(observer);
    */
}

void ContainerLoadingCP::PrintSolution()
{
    for (size_t i = 0; i < mItems.size(); ++i)
    {
        LOG(INFO) << "Item " << std::to_string(i) << ": ("
                  << operations_research::sat::SolutionIntegerValue(mResponse, mStartPositionsX[i]) << ","
                  << operations_research::sat::SolutionIntegerValue(mResponse, mStartPositionsY[i]) << ","
                  << operations_research::sat::SolutionIntegerValue(mResponse, mStartPositionsZ[i]) << ") | ("
                  << operations_research::sat::SolutionIntegerValue(mResponse, mLengths[i]) << ","
                  << operations_research::sat::SolutionIntegerValue(mResponse, mWidths[i]) << ","
                  << operations_research::sat::SolutionIntegerValue(mResponse, mHeights[i]) << ") | ("
                  << operations_research::sat::SolutionIntegerValue(mResponse, mEndPositionsX[i]) << ","
                  << operations_research::sat::SolutionIntegerValue(mResponse, mEndPositionsY[i]) << ","
                  << operations_research::sat::SolutionIntegerValue(mResponse, mEndPositionsZ[i]) << ")";
    }

    for (size_t i = 0; i < mItems.size(); ++i)
    {
        mRelativeDirections.emplace_back();
        for (size_t j = 0; j < mItems.size(); ++j)
        {
            mRelativeDirections[i].emplace_back();
            std::stringstream vars;
            vars << std::to_string(i) << "_" << std::to_string(j) << ": ";
            for (const auto& dimension: mDimensions)
            {
                vars << operations_research::sat::SolutionBooleanValue(
                    mResponse, mRelativeDirections[i][j][dimension.FirstDirection])
                     << " ";
                vars << operations_research::sat::SolutionBooleanValue(
                    mResponse, mRelativeDirections[i][j][dimension.SecondDirection])
                     << " ";
            }

            vars << " | ";
            vars << operations_research::sat::SolutionIntegerValue(mResponse, mOverlapAreasXY[i][j]);
            LOG(INFO) << vars.str();
        }
    }
}

void ContainerLoadingCP::ExtractPacking(std::vector<Cuboid>& items) const
{
    for (size_t i = 0; i < items.size(); ++i)
    {
        auto& item = items[i];

        item.Rotated = (Rotation)operations_research::sat::SolutionBooleanValue(mResponse, mOrientation[i][RotationZ]);

        item.X = operations_research::sat::SolutionIntegerValue(mResponse, mStartPositionsX[i]);
        item.Y = operations_research::sat::SolutionIntegerValue(mResponse, mStartPositionsY[i]);
        item.Z = operations_research::sat::SolutionIntegerValue(mResponse, mStartPositionsZ[i]);
    }
}

std::vector<int> ContainerLoadingCP::ExtractSequence() const
{
    std::vector<std::tuple<int, int>> assignments;
    assignments.reserve(mNumberCustomers);
    for (size_t i = 0; i < mNumberCustomers; ++i)
    {
        assignments.emplace_back(operations_research::sat::SolutionIntegerValue(mResponse, mCustomerPosition[i]), i);
    }

    std::ranges::sort(assignments);

    std::vector<int> sequence = std::vector<int>();
    sequence.reserve(assignments.size());
    for (const auto& [position, id]: assignments)
    {
        sequence.push_back(id);
    }

    return sequence;
}

void ContainerLoadingCP::CreateVariables()
{
    size_t numberOfItems = mItems.size();

    std::vector<Cuboid> itemCopy;
    itemCopy.reserve(mItems.size());
    for (const auto& item: mItems)
    {
        itemCopy.push_back(item);
    }

    auto placementPointsPerType = PlacementPointGenerator::GeneratePlacementPatterns(
        mContainer, itemCopy, mPlacementPatternTypeX, mPlacementPatternTypeY, mPlacementPatternTypeZ);

    mStartPositionsX.reserve(numberOfItems);
    mEndPositionsX.reserve(numberOfItems);
    mStartPositionsY.reserve(numberOfItems);
    mEndPositionsY.reserve(numberOfItems);
    mStartPositionsZ.reserve(numberOfItems);
    mEndPositionsZ.reserve(numberOfItems);

    for (size_t i = 0; i < numberOfItems; i++)
    {
        const Cuboid& item = mItems[i];
        int minLength = item.EnableHorizontalRotation ? std::min(item.Dx, item.Dy) : item.Dx;
        int minWidth = item.EnableHorizontalRotation ? std::min(item.Dx, item.Dy) : item.Dy;

        mStartPositionsX.emplace_back(
            mModelCP.NewIntVar(operations_research::Domain::FromValues(placementPointsPerType[item].X)));
        mEndPositionsX.emplace_back(mModelCP.NewIntVar({minLength, mContainer.Dx}));

        mStartPositionsY.emplace_back(
            mModelCP.NewIntVar(operations_research::Domain::FromValues(placementPointsPerType[item].Y)));
        mEndPositionsY.emplace_back(mModelCP.NewIntVar({minWidth, mContainer.Dy}));

        mStartPositionsZ.emplace_back(
            mModelCP.NewIntVar(operations_research::Domain::FromValues(placementPointsPerType[item].Z)));
        mEndPositionsZ.emplace_back(mModelCP.NewIntVar({item.Dz, mContainer.Dz}));
    }

    mLengths.reserve(numberOfItems);
    mWidths.reserve(numberOfItems);
    mHeights.reserve(numberOfItems);

    for (size_t i = 0; i < numberOfItems; i++)
    {
        const Cuboid& item = mItems[i];

        int minLength = item.EnableHorizontalRotation ? std::min(item.Dx, item.Dy) : item.Dx;
        int minWidth = item.EnableHorizontalRotation ? std::min(item.Dx, item.Dy) : item.Dy;

        int maxLength = item.EnableHorizontalRotation ? std::max(item.Dx, item.Dy) : item.Dx;
        int maxWidth = item.EnableHorizontalRotation ? std::max(item.Dx, item.Dy) : item.Dy;

        mLengths.emplace_back(mModelCP.NewIntVar(
            operations_research::Domain::FromIntervals({{minLength, minLength}, {maxLength, maxLength}})));

        mWidths.emplace_back(mModelCP.NewIntVar(
            operations_research::Domain::FromIntervals({{minWidth, minWidth}, {maxWidth, maxWidth}})));

        mHeights.emplace_back(mModelCP.NewIntVar({item.Dz, item.Dz}));
    }

    mPlacedOnFloor.reserve(numberOfItems);
    for (size_t i = 0; i < numberOfItems; i++)
    {
        mPlacedOnFloor.emplace_back(mModelCP.NewBoolVar());
    }

    mIntervalsX.reserve(numberOfItems);
    mIntervalsY.reserve(numberOfItems);
    mIntervalsZ.reserve(numberOfItems);

    for (size_t i = 0; i < numberOfItems; i++)
    {
        mIntervalsX.emplace_back(mModelCP.NewIntervalVar(mStartPositionsX[i], mLengths[i], mEndPositionsX[i]));
        mIntervalsY.emplace_back(mModelCP.NewIntervalVar(mStartPositionsY[i], mWidths[i], mEndPositionsY[i]));
        mIntervalsZ.emplace_back(mModelCP.NewIntervalVar(mStartPositionsZ[i], mHeights[i], mEndPositionsZ[i]));
    }

    mOrientation.reserve(numberOfItems);
    for (size_t i = 0; i < numberOfItems; i++)
    {
        const Cuboid& itemI = mItems[i];
        mOrientation.emplace_back();
        mOrientation[i].reserve(mItemOrientations.size());

        for (size_t o = 0; o < mItemOrientations.size(); ++o)
        {
            mOrientation[i].emplace_back(mModelCP.NewBoolVar());
        }

        if (!itemI.EnableHorizontalRotation)
        {
            // TODO: consider not creating the variable instead of fixing it to zero.
            mModelCP.FixVariable(mOrientation[i][RotationZ], false);
        }
    }

    mRelativeDirections.reserve(numberOfItems);
    mSupportXY.reserve(numberOfItems);
    for (size_t i = 0; i < numberOfItems; i++)
    {
        mRelativeDirections.emplace_back();
        mRelativeDirections[i].reserve(numberOfItems);

        if (mEnableFragility || mEnableSupport)
        {
            mSupportXY.emplace_back();
            mSupportXY[i].reserve(numberOfItems);
        }

        for (size_t j = 0; j < numberOfItems; ++j)
        {
            mRelativeDirections[i].emplace_back();
            mRelativeDirections.reserve(mDimensions.size());
            for (size_t d = 0; d < mDimensions.size(); ++d)
            {
                mRelativeDirections[i][j].emplace_back(mModelCP.NewBoolVar());
                mRelativeDirections[i][j].emplace_back(mModelCP.NewBoolVar());
            }

            if (mEnableFragility || mEnableSupport)
            {
                mSupportXY[i].emplace_back(mModelCP.NewBoolVar());
            }
        }
    }

    mOverlapAreasXY.reserve(numberOfItems);
    mItemsOverlapsXY.reserve(numberOfItems);
    for (size_t i = 0; i < numberOfItems - 1; i++)
    {
        const Cuboid& itemI = mItems[i];

        mItemsOverlapsXY.emplace_back();
        mItemsOverlapsXY.reserve(numberOfItems - i);

        if (mEnableSupport)
        {
            mOverlapAreasXY.emplace_back();
            mOverlapAreasXY.reserve(numberOfItems - i);
        }

        for (size_t j = i + 1; j < numberOfItems; j++)
        {
            const Cuboid& itemJ = mItems[j];
            int maxIntersection = std::max(itemI.Dx * itemI.Dy, itemJ.Dx * itemJ.Dy);

            mItemsOverlapsXY[i].emplace_back(mModelCP.NewBoolVar());

            if (mEnableSupport)
            {
                mOverlapAreasXY[i].emplace_back(mModelCP.NewIntVar({0, maxIntersection}));
            }
        }
    }

    if (mEnableLifo && !mFixedSequence)
    {
        mCustomerPosition.reserve(mNumberCustomers);
        for (size_t i = 0; i < mNumberCustomers; ++i)
        {
            mCustomerPosition.emplace_back(mModelCP.NewIntVar({1, static_cast<int>(mNumberCustomers)}));
        }

        mSuccessionMatrix.reserve(mNumberCustomers);
        for (size_t i = 0; i < mNumberCustomers - 1; i++)
        {
            mSuccessionMatrix.emplace_back();
            mSuccessionMatrix.reserve(mNumberCustomers - i);
            for (size_t j = i + 1; j < mNumberCustomers; j++)
            {
                mSuccessionMatrix[i].emplace_back(mModelCP.NewBoolVar());
            }
        }
    }

    mMaxLength = mModelCP.NewIntVar({0, mContainer.Dx});
}

std::tuple<ORIntVars1D, ORIntVars1D> ContainerLoadingCP::GetIntVars(DimensionType dimension) const
{
    switch (dimension)
    {
        case AxisX:
            return std::make_tuple(mStartPositionsX, mEndPositionsX);
        case AxisY:
            return std::make_tuple(mStartPositionsY, mEndPositionsY);
        case AxisZ:
            return std::make_tuple(mStartPositionsZ, mEndPositionsZ);
        default:
            throw std::runtime_error("DimensionType not implemented.");
    }
}

void ContainerLoadingCP::AddConstraints()
{
    CreateNoOverlap();

    CreateItemOrientations();

    if (mEnableFragility || mEnableSupport)
    {
        CreateXYIntersectionBool();
        CreateSupportItem();
    }

    if (mEnableFragility)
    {
        CreateFragility();
    }

    CreateOnFloorConstraints();

    if (mEnableSupport)
    {
        CreateXYIntersectionArea();
        CreateSupportArea();
    }

    if (mEnableLifo)
    {
        if (mFixedSequence)
        {
            CreateLifoSequence();
        }
        else
        {
            CreatePositioningConstraints();
            CreateLifoNoSequence();
        }
    }
}

/// Relative directions of items. Necessary for non overlapping items.
void ContainerLoadingCP::CreateNoOverlap()
{
    size_t numberOfItems = mItems.size();
    for (size_t i = 0; i < numberOfItems; ++i)
    {
        for (size_t j = 0; j < numberOfItems; ++j)
        {
            if (i == j)
            {
                continue;
            }

            for (size_t d = 0; d < mDimensions.size(); ++d)
            {
                const Dimension& dimension = mDimensions[d];
                const auto [startPosition, endPosition] = GetIntVars(dimension.Type);

                mModelCP.AddLessOrEqual(endPosition[j], startPosition[i])
                    .OnlyEnforceIf(mRelativeDirections[i][j][dimension.FirstDirection]);
                mModelCP.AddLessThan(startPosition[i], endPosition[j])
                    .OnlyEnforceIf(mRelativeDirections[i][j][dimension.FirstDirection].Not());

                mModelCP.AddLessOrEqual(endPosition[i], startPosition[j])
                    .OnlyEnforceIf(mRelativeDirections[i][j][dimension.SecondDirection]);
                mModelCP.AddLessThan(startPosition[j], endPosition[i])
                    .OnlyEnforceIf(mRelativeDirections[i][j][dimension.SecondDirection].Not());

                mModelCP.AddEquality(mRelativeDirections[i][j][dimension.FirstDirection],
                                     mRelativeDirections[j][i][dimension.SecondDirection]);
            }

            // No overlap constraints
            mModelCP.AddAtLeastOne(mRelativeDirections[i][j]);
        }
    }
}

/// Set dimensions of items based on orientation
void ContainerLoadingCP::CreateItemOrientations()
{
    size_t numberOfItems = mItems.size();
    for (size_t i = 0; i < numberOfItems; ++i)
    {
        const Cuboid& item = mItems[i];
        for (size_t o = 0; o < mItemOrientations.size(); ++o)
        {
            // Dimensions of item depending on orientation
            auto [itemLength, itemWidth, itemHeight] = item.DetermineDimensions(mItemOrientations[o]);

            mModelCP.AddEquality(mLengths[i], itemLength).OnlyEnforceIf(mOrientation[i][o]);
            mModelCP.AddEquality(mWidths[i], itemWidth).OnlyEnforceIf(mOrientation[i][o]);
            mModelCP.AddEquality(mHeights[i], itemHeight).OnlyEnforceIf(mOrientation[i][o]);
        }

        mModelCP.AddExactlyOne(mOrientation[i]);
    }
}

/// Fragility of items
void ContainerLoadingCP::CreateFragility()
{
    size_t numberOfItems = mItems.size();

    // Variant 2 - fragile items can be stacked on fragile items.
    // Fragile items can be stacked onto other fragile or non-fragile items, whereas non-fragile items must not touch
    // fragile items from above.
    for (size_t i = 0; i < numberOfItems; ++i)
    {
        for (size_t j = 0; j < numberOfItems; ++j)
        {
            if (mItems[j].Fragility == Fragility::Fragile && mItems[i].Fragility == Fragility::None)
            {
                if (i != j)
                {
                    // Item j cannot support item i, if j is fragile and i non fragile.
                    mModelCP.FixVariable(mSupportXY[i][j], false);
                }
            }
        }
    }
}

/// Determines supported area of items
void ContainerLoadingCP::CreateSupportArea()
{
    for (size_t i = 0; i < mItems.size(); ++i)
    {
        ORLinExpr supportedAreaExpr;
        int areaI = mItems[i].Dy * mItems[i].Dx;
        for (size_t j = 0; j < mItems.size(); ++j)
        {
            if (i == j)
            {
                continue;
            }

            if (!mEnableFragility || mItems[j].Fragility == Fragility::None
                || (mItems[j].Fragility == Fragility::Fragile && mItems[i].Fragility == Fragility::Fragile))
            {
                int areaJ = mItems[j].Dy * mItems[j].Dx;
                int minArea = std::min(areaI, areaJ);
                operations_research::sat::IntVar usableArea = mModelCP.NewIntVar({0, minArea});
                if (i < j)
                {
                    auto position = j - i - 1;
                    mModelCP.AddMultiplicationEquality(usableArea, {mOverlapAreasXY[i][position], mSupportXY[i][j]});
                }
                else
                {
                    auto position = i - j - 1;
                    mModelCP.AddMultiplicationEquality(usableArea, {mOverlapAreasXY[j][position], mSupportXY[i][j]});
                }

                mModelCP.AddEquality(usableArea, 0).OnlyEnforceIf(mSupportXY[i][j].Not());
                supportedAreaExpr += usableArea;
            }
        }

        operations_research::sat::IntVar supportedArea = mModelCP.NewIntVar({0, areaI});
        mModelCP.AddEquality(supportedArea, supportedAreaExpr).OnlyEnforceIf(mPlacedOnFloor[i].Not());

        mModelCP
            .AddGreaterOrEqual(supportedArea, static_cast<int>(std::ceil(mSupportArea * mItems[i].Dx * mItems[i].Dy)))
            .OnlyEnforceIf(mPlacedOnFloor[i].Not());
    }
}

/// Set bool variable, if items i and j intersect in XY
void ContainerLoadingCP::CreateXYIntersectionBool()
{
    size_t numberOfItems = mItems.size();

    for (size_t i = 0; i < numberOfItems - 1; ++i)
    {
        for (size_t j = i + 1; j < numberOfItems; ++j)
        {
            auto positionJ = j - i - 1;

            mModelCP.AddAtLeastOne({mItemsOverlapsXY[i][positionJ],
                                    mRelativeDirections[i][j][Left],
                                    mRelativeDirections[i][j][Right],
                                    mRelativeDirections[i][j][Behind],
                                    mRelativeDirections[i][j][InFront]});

            mModelCP.AddImplication(mRelativeDirections[i][j][Left], mItemsOverlapsXY[i][positionJ].Not());
            mModelCP.AddImplication(mRelativeDirections[i][j][Right], mItemsOverlapsXY[i][positionJ].Not());
            mModelCP.AddImplication(mRelativeDirections[i][j][Behind], mItemsOverlapsXY[i][positionJ].Not());
            mModelCP.AddImplication(mRelativeDirections[i][j][InFront], mItemsOverlapsXY[i][positionJ].Not());
        }
    }
}

/// Determine intersection area of all items
void ContainerLoadingCP::CreateXYIntersectionArea()
{
    size_t numberOfItems = mItems.size();

    for (size_t i = 0; i < numberOfItems - 1; ++i)
    {
        for (size_t j = i + 1; j < numberOfItems; ++j)
        {
            if (mItems[i].Dz + mItems[j].Dz <= mContainer.Dz)
            {
                // Variant 2
                auto positionJ = j - i - 1;

                // Overlap in x
                operations_research::sat::IntVar diffXij = mModelCP.NewIntVar({0, mContainer.Dx});
                mModelCP
                    .AddEquality(diffXij,
                                 ORLinExpr::LinearExpr::WeightedSum({mEndPositionsX[i], mStartPositionsX[j]}, {1, -1}))
                    .OnlyEnforceIf({mItemsOverlapsXY[i][positionJ]});

                operations_research::sat::IntVar diffXji = mModelCP.NewIntVar({0, mContainer.Dx});
                mModelCP
                    .AddEquality(diffXji,
                                 ORLinExpr::LinearExpr::WeightedSum({mEndPositionsX[j], mStartPositionsX[i]}, {1, -1}))
                    .OnlyEnforceIf({mItemsOverlapsXY[i][positionJ]});

                operations_research::sat::IntVar xOverlap = mModelCP.NewIntVar({0, mContainer.Dx});
                mModelCP.AddMinEquality(xOverlap, {diffXij, diffXji, mLengths[i], mLengths[j]});
                mModelCP.AddEquality(xOverlap, 0).OnlyEnforceIf(mItemsOverlapsXY[i][positionJ].Not());

                // Overlap in y
                operations_research::sat::IntVar diffYij = mModelCP.NewIntVar({0, mContainer.Dy});
                mModelCP
                    .AddEquality(diffYij,
                                 ORLinExpr::LinearExpr::WeightedSum({mEndPositionsY[i], mStartPositionsY[j]}, {1, -1}))
                    .OnlyEnforceIf({mItemsOverlapsXY[i][positionJ]});

                operations_research::sat::IntVar diffYji = mModelCP.NewIntVar({0, mContainer.Dy});
                mModelCP
                    .AddEquality(diffYji,
                                 ORLinExpr::LinearExpr::WeightedSum({mEndPositionsY[j], mStartPositionsY[i]}, {1, -1}))
                    .OnlyEnforceIf({mItemsOverlapsXY[i][positionJ]});

                operations_research::sat::IntVar yOverlap = mModelCP.NewIntVar({0, mContainer.Dy});
                mModelCP.AddMinEquality(yOverlap, {diffYij, diffYji, mWidths[i], mWidths[j]});
                mModelCP.AddEquality(yOverlap, 0).OnlyEnforceIf(mItemsOverlapsXY[i][positionJ].Not());

                // Area
                mModelCP.AddMultiplicationEquality(mOverlapAreasXY[i][positionJ], {xOverlap, yOverlap});
            }
        }
    }
}

/// Set bool variable, if item j supports item i -> adjacent in Z AND items intersect in XY
void ContainerLoadingCP::CreateSupportItem()
{
    for (size_t i = 0; i < mItems.size(); ++i)
    {
        mModelCP.FixVariable(mSupportXY[i][i], false);
        for (size_t j = 0; j < mItems.size(); ++j)
        {
            if (i == j)
            {
                continue;
            }

            operations_research::sat::BoolVar isVerticallyAdjacent = mModelCP.NewBoolVar();
            mModelCP.AddEquality(mEndPositionsZ[j], mStartPositionsZ[i]).OnlyEnforceIf(isVerticallyAdjacent);
            mModelCP.AddNotEqual(mEndPositionsZ[j], mStartPositionsZ[i]).OnlyEnforceIf(isVerticallyAdjacent.Not());

            mModelCP.AddImplication(isVerticallyAdjacent.Not(), mSupportXY[i][j].Not());

            if (i < j)
            {
                auto position = j - i - 1;

                mModelCP.AddAtLeastOne(
                    {mSupportXY[i][j], isVerticallyAdjacent.Not(), mItemsOverlapsXY[i][position].Not()});
                mModelCP.AddImplication(mItemsOverlapsXY[i][position].Not(), mSupportXY[i][j].Not());
            }
            else
            {
                auto position = i - j - 1;

                mModelCP.AddAtLeastOne(
                    {mSupportXY[i][j], isVerticallyAdjacent.Not(), mItemsOverlapsXY[j][position].Not()});
                mModelCP.AddImplication(mItemsOverlapsXY[j][position].Not(), mSupportXY[i][j].Not());
            }
        }
    }
}

/// LIFO unloading with given customer sequence
void ContainerLoadingCP::CreateLifoSequence()
{
    size_t numberOfItems = mItems.size();

    for (size_t i = 0; i < numberOfItems; ++i)
    {
        for (size_t j = 0; j < numberOfItems; ++j)
        {
            if (i != j && mItems[i].GroupId < mItems[j].GroupId)
            {
                // Item i must be placed behind or below item j if
                // - item i is unloaded after item j (smaller group id) and
                // - item i is not placed left or right of item j -> in way to rear end of container

                mModelCP.AddAtLeastOne({mRelativeDirections[i][j][Behind], mRelativeDirections[i][j][Below]})
                    .OnlyEnforceIf({mRelativeDirections[i][j][Left].Not(), mRelativeDirections[i][j][Right].Not()});
            }
        }
    }
}

/// LIFO unloading without given customer sequence
void ContainerLoadingCP::CreateLifoNoSequence()
{
    for (size_t i = 0; i < mItems.size(); ++i)
    {
        auto customerI = mItems[i].GroupId;
        for (size_t j = 0; j < mItems.size(); ++j)
        {
            auto customerJ = mItems[j].GroupId;

            if (i != j && customerI != customerJ)
            {
                // Item i must be placed behind or below item j if
                // - item i is unloaded after item j (customer i succeeds customer j) and
                // - item i is not placed left or right of item j -> in way to rear end of container.

                if (customerI < customerJ)
                {
                    auto position = customerJ - customerI - 1;
                    mModelCP.AddAtLeastOne({mRelativeDirections[i][j][Behind], mRelativeDirections[i][j][Below]})
                        .OnlyEnforceIf({mRelativeDirections[i][j][Left].Not(),
                                        mRelativeDirections[i][j][Right].Not(),
                                        mSuccessionMatrix[customerI][position]});
                }
                else
                {
                    auto position = customerI - customerJ - 1;
                    mModelCP.AddAtLeastOne({mRelativeDirections[i][j][Behind], mRelativeDirections[i][j][Below]})
                        .OnlyEnforceIf({mRelativeDirections[i][j][Left].Not(),
                                        mRelativeDirections[i][j][Right].Not(),
                                        mSuccessionMatrix[customerJ][position].Not()});
                }
            }
        }
    }
}

/// Positioning of customers in route if customer sequence is not given; needed for LIFO
void ContainerLoadingCP::CreatePositioningConstraints()
{
    mModelCP.AddAllDifferent(mCustomerPosition);

    for (size_t i = 0; i < mNumberCustomers - 1; i++)
    {
        for (size_t j = i + 1; j < mNumberCustomers; j++)
        {
            auto positionInVector = j - i - 1;

            // If customer i is visited after customer j => position of i is greater than position of j
            // Sequence of customers in ascending order
            mModelCP.AddGreaterThan(mCustomerPosition[i], mCustomerPosition[j])
                .OnlyEnforceIf(mSuccessionMatrix[i][positionInVector]);
            mModelCP.AddLessThan(mCustomerPosition[i], mCustomerPosition[j])
                .OnlyEnforceIf(mSuccessionMatrix[i][positionInVector].Not());
        }
    }
}

/// Determine which items are placed on the floor
void ContainerLoadingCP::CreateOnFloorConstraints()
{
    for (size_t i = 0; i < mItems.size(); ++i)
    {
        mModelCP.AddEquality(mStartPositionsZ[i], 0).OnlyEnforceIf(mPlacedOnFloor[i]);
        mModelCP.AddGreaterThan(mStartPositionsZ[i], 0).OnlyEnforceIf(mPlacedOnFloor[i].Not());
    }
}

void ContainerLoadingCP::AddObjective()
{
    mModelCP.AddMaxEquality(mMaxLength, mEndPositionsX);

    mModelCP.Minimize(mMaxLength);
}

}
}