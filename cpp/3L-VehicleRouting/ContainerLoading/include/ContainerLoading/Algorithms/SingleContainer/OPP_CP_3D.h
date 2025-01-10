#pragma once

#include "CommonBasics/Helper/CPServices.h"

#include "Model/Container.h"

#include "Algorithms/CPSolverParameters.h"
#include "Algorithms/LoadingStatus.h"
#include "Algorithms/PlacementPoints.h"

namespace ContainerLoading
{
using namespace Model;
namespace Algorithms
{
class ContainerLoadingCP
{
  public:
    [[nodiscard]] std::tuple<ORIntVars1D, ORIntVars1D> GetIntVars(DimensionType dimension) const;

    void WriteProtoModel(const operations_research::sat::CpModelProto& protoModel) const;
    void PrintSolution();
    void ExtractPacking(std::vector<Cuboid>& items) const;
    [[nodiscard]] std::vector<int> ExtractSequence() const;

    [[nodiscard]] LoadingStatus Solve();
    [[nodiscard]] double GetRuntime() const { return mResponse.wall_time(); };

    ContainerLoadingCP(const CPSolverParams& params,
                       const Container& container,
                       const std::vector<Cuboid>& items,
                       const size_t numberCustomers,
                       const LoadingFlag loadingMask,
                       const double supportArea,
                       const double maxRuntime)
    : mParams(params),
      mContainer(container),
      mItems(items),
      mNumberCustomers(numberCustomers),
      mEnableFragility(IsSet(loadingMask, LoadingFlag::Fragility)),
      mEnableLifo(IsSet(loadingMask, LoadingFlag::Lifo)),
      mFixedSequence(IsSet(loadingMask, LoadingFlag::Sequence)),
      mEnableSupport(IsSet(loadingMask, LoadingFlag::Support)),
      mSupportArea(supportArea),
      mMaxRuntime(maxRuntime)
    {
        auto [placementPatternTypeX, placementPatternTypeY, placementPatternTypeZ] =
            PlacementPointGenerator::SelectMinimalFeasiblePatternType(loadingMask);
        mPlacementPatternTypeX = placementPatternTypeX;
        mPlacementPatternTypeY = placementPatternTypeY;
        mPlacementPatternTypeZ = placementPatternTypeZ;
    }

  private:
    const CPSolverParams& mParams;
    const Container& mContainer;
    const std::vector<Cuboid>& mItems;
    size_t mNumberCustomers;

    const bool mEnableFragility;
    const bool mEnableLifo;
    const bool mFixedSequence;
    const bool mEnableSupport;

    const double mSupportArea;

    const int mMaxReachability = 30;

    const double mMaxRuntime;

    PlacementPattern mPlacementPatternTypeX = PlacementPattern::None;
    PlacementPattern mPlacementPatternTypeY = PlacementPattern::None;
    PlacementPattern mPlacementPatternTypeZ = PlacementPattern::None;

    operations_research::sat::CpSolverResponse mResponse;

    std::vector<Dimension> mDimensions = {{AxisY, Right, Left}, {AxisX, InFront, Behind}, {AxisZ, Above, Below}};
    std::vector<Orientation> mItemOrientations = std::vector{NoRotation, RotationZ};

    operations_research::sat::CpModelBuilder mModelCP;

    ORIntVars1D mStartPositionsX;
    ORIntVars1D mEndPositionsX;
    ORIntVars1D mStartPositionsY;
    ORIntVars1D mEndPositionsY;
    ORIntVars1D mStartPositionsZ;
    ORIntVars1D mEndPositionsZ;

    ORIntervalVars mIntervalsX;
    ORIntervalVars mIntervalsY;
    ORIntervalVars mIntervalsZ;

    ORBoolVars3D mRelativeDirections; // mRelativeDirections[i][j][direction], bool, 1 if item i is placed relatively to
                                      // item j in direction

    ORBoolVars2D mItemsOverlapsXY; // mItemsOverlapsXY[i][j], bool, items i and j intersect in xy-plane
    ORBoolVars2D mSupportXY; // mSupportXY[i][j], bool, 1, if item i is supported by item j xy-plane ? -> items
                             // intersect AND item j is directly below item i
    ORIntVars2D
        mOverlapAreasXY; // mOverlapAreasXY[i][j], integer, size of intersection area in xy-plane of items i and j

    ORIntVars1D mWidths;
    ORIntVars1D mLengths;
    ORIntVars1D mHeights;

    ORBoolVars1D mPlacedOnFloor;
    ORBoolVars2D mOrientation;

    ORIntVars1D mCustomerPosition; // mCustomerPosition[i], integer, position of customer i in route, smaller values
                                   // visited earlier
    ORBoolVars2D mSuccessionMatrix; // suceeds[i][j], bool, 1, if customer i succeeds customer j in route

    operations_research::sat::IntVar mMaxLength;

    void BuildModel();
    void AddConstraints();
    void CreateNoOverlap();
    void CreateItemOrientations();
    void CreateFragility();
    void CreateSupportItem();
    void CreateSupportArea();
    void CreateXYIntersectionBool();
    void CreateXYIntersectionArea();
    void CreateLifoSequence();
    void CreateLifoNoSequence();
    void CreatePositioningConstraints();
    void CreateOnFloorConstraints();

    void AddObjective();
    void CreateVariables();

    void SetParameters(operations_research::sat::SatParameters& parameters) const;
};

}
}