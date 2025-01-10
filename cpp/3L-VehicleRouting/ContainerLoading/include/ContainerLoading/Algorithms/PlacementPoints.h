#pragma once

#include "Model/Container.h"

#include "LoadingStatus.h"

#include <boost/dynamic_bitset/dynamic_bitset.hpp>

#include <unordered_map>
#include <vector>

namespace ContainerLoading
{
using namespace Model;
namespace Algorithms
{
using int64 = int64_t;

using PlacementPoints1D = std::vector<bool>;
using PlacementPoints2D = std::vector<std::vector<bool>>;
using PlacementPoints3D = std::vector<std::vector<std::vector<bool>>>;

enum class PlacementPattern
{
    None,
    UnitDiscretization,
    NormalPatterns,
    ReducedRasterPoints,
    RegularNormalPatterns,
    MeetInTheMiddle
};

struct ItemPlacementPoints
{
    PlacementPoints3D Coordinates;
};

struct ItemPlacementPatterns
{
    std::vector<int64> X;
    std::vector<int64> Y;
    std::vector<int64> Z;
};

struct ItemPlacementPatternsBitset
{
    boost::dynamic_bitset<> X;
    boost::dynamic_bitset<> Y;
    boost::dynamic_bitset<> Z;
};

class PlacementPointGenerator
{
  public:
    enum class MeetInTheMiddleMinimizationTarget
    {
        /// Minimize |M_is|
        IndividualPlacementPoints,

        /// Minimize |M_s|
        PlacementPointUnion
    };

    static std::tuple<PlacementPattern, PlacementPattern, PlacementPattern>
        SelectMinimalFeasiblePatternType(LoadingFlag loadingMask);

    /// Keep std::vector versions for reference.
    /*
    static std::vector<int64> DetermineNormalPatternsX(
        const Container& container,
        const std::vector<Cuboid>& items);

    static std::vector<int64> DetermineNormalPatternsY(
        const Container& container,
        const std::vector<Cuboid>& items);

    static std::vector<int64> DetermineNormalPatternsZ(
        const Container& container,
        const std::vector<Cuboid>& items);
    */

    /// Regular normal patterns according to Côté, J. F., & Iori, M. (2018). The meet-in-the-middle principle for
    /// cutting and packing problems. INFORMS Journal on Computing, 30(4), 646-661.
    static boost::dynamic_bitset<>
        DetermineRegularNormalPatternsX(int containerDx, int actualContainerDx, const std::vector<Cuboid*>& items);

    static boost::dynamic_bitset<>
        DetermineRegularNormalPatternsY(int containerDy, int actualContainerDy, const std::vector<Cuboid*>& items);

    static boost::dynamic_bitset<>
        DetermineRegularNormalPatternsZ(int containerDz, int actualContainerDz, const std::vector<Cuboid*>& items);

    static std::tuple<std::vector<int64>, std::vector<int64>, std::vector<int64>>
        DetermineMeetInTheMiddlePatterns(const Container& container, std::vector<Cuboid>& items);

    static std::vector<ItemPlacementPatternsBitset>
        DetermineMinimalMeetInTheMiddlePatterns(const Container& container,
                                                std::vector<Cuboid>& items,
                                                MeetInTheMiddleMinimizationTarget minimizationTarget);
    static std::vector<boost::dynamic_bitset<>>
        DetermineMinimalMeetInTheMiddlePatterns(const Container& container,
                                                std::vector<Cuboid>& items,
                                                MeetInTheMiddleMinimizationTarget minimizationTarget,
                                                Axis axis);

    static std::vector<boost::dynamic_bitset<>>
        DetermineUnitDiscretizationPoints(const Container& container, std::vector<Cuboid>& items, Axis axis);
    static std::vector<boost::dynamic_bitset<>>
        GenerateRegularNormalPatterns(const Container& container, std::vector<Cuboid>& items, Axis axis);

    static std::tuple<boost::dynamic_bitset<>, boost::dynamic_bitset<>, boost::dynamic_bitset<>>
        DetermineItemSpecificMeetInTheMiddlePatterns(const Container& container,
                                                     const std::vector<Cuboid*>& items,
                                                     const Cuboid& itemI);

    static boost::dynamic_bitset<> DetermineRegularNormalPatterns(const Container& container,
                                                                  const std::vector<Cuboid*>& items,
                                                                  const Cuboid& itemI,
                                                                  std::vector<int>& meetInTheMiddlePointsLeftX,
                                                                  std::vector<int>& meetInTheMiddlePointsRightX,
                                                                  MeetInTheMiddleMinimizationTarget minimizationTarget,
                                                                  Axis axis);

    static boost::dynamic_bitset<> DetermineMeetInTheMiddlePatterns(const Container& container,
                                                                    const std::vector<Cuboid*>& items,
                                                                    const Cuboid& itemI,
                                                                    int threshold,
                                                                    Axis axis);

    /// Preprocessing steps according to Côté, J. F., & Iori, M. (2018). The meet-in-the-middle principle for cutting
    /// and packing problems. INFORMS Journal on Computing, 30(4), 646-661.
    static std::vector<boost::dynamic_bitset<>> DetermineReducedMeetInTheMiddlePatterns(std::vector<Cuboid>& items,
                                                                                        const Container& container,
                                                                                        int threshold,
                                                                                        Axis axis,
                                                                                        bool enablePreprocessingStep1,
                                                                                        bool enablePreprocessingStep2);

    static void DetermineSingleItemReducedLeftRightPatterns(int threshold,
                                                            int separationThreshold,
                                                            boost::dynamic_bitset<>& placementPointsLeft,
                                                            const Container& container,
                                                            Axis axis,
                                                            int minSelectedItemDimension,
                                                            const std::vector<Cuboid*>& doublyFilteredItems,
                                                            boost::dynamic_bitset<>& placementPointsRightPrime,
                                                            const std::vector<Cuboid*>& filteredItems);

    static void DetermineEnlargedItemDimensionsLeft(
        std::vector<Cuboid>& items,
        Axis axis,
        std::vector<std::vector<int>>& itemSpecificModifiedItemDimensions,
        const std::vector<boost::dynamic_bitset<>>& itemSpecificPlacementPointsLeft,
        const Container& container,
        const std::vector<boost::dynamic_bitset<>>& preliminaryItemSpecificMeetInTheMiddleSets);
    static void DetermineEnlargedItemDimensionsRight(
        std::vector<Cuboid>& items,
        Axis axis,
        std::vector<std::vector<int>>& itemSpecificModifiedItemDimensions,
        std::vector<boost::dynamic_bitset<>>& itemSpecificPlacementPointsRightPrime,
        const std::vector<boost::dynamic_bitset<>>& preliminaryItemSpecificMeetInTheMiddleSets);

    static void RemoveRedundantPatterns(std::vector<Cuboid>& items,
                                        const std::vector<std::vector<int>>& itemSpecificModifiedItemDimensions,
                                        std::vector<boost::dynamic_bitset<>>& itemSpecificPlacementPointsLeft);

    static std::vector<boost::dynamic_bitset<>>
        GenerateMeetInTheMiddlePatterns(std::vector<Cuboid>& items,
                                        const Container& container,
                                        const std::vector<boost::dynamic_bitset<>>& regularNormalPatterns,
                                        int threshold,
                                        Axis axis);

    static std::vector<int64> ConvertPlacementBitsetToVector(const boost::dynamic_bitset<>& itemSpecificPatternBitset);
    static ItemPlacementPatterns
        ConvertPlacementBitsetToVector(const boost::dynamic_bitset<>& itemSpecificPatternBitsetX,
                                       const boost::dynamic_bitset<>& itemSpecificPatternBitsetY,
                                       const boost::dynamic_bitset<>& itemSpecificPatternBitsetZ);
    static ItemPlacementPatterns
        ConvertPlacementBitsetToVector(const ItemPlacementPatternsBitset& itemSpecificPatternBitset);

    ////static std::vector<int64> DetermineStartPointsFlatSpan(const std::vector<int64>& patterns); // Keep for
    ///reference.
    static std::vector<int64>
        DetermineEndPoints(const std::vector<int64>& patterns, int dim, int rotDim, bool enableRotation);

    static std::vector<boost::dynamic_bitset<>> GeneratePlacementPatterns(const Container& container,
                                                                          std::vector<Cuboid>& items,
                                                                          PlacementPattern placementPatternType,
                                                                          Axis axis);

    static std::vector<std::vector<int64>> GeneratePlacementPatternsBaseType(const Container& container,
                                                                             std::vector<Cuboid>& items,
                                                                             PlacementPattern placementPatternType,
                                                                             Axis axis);

    static std::unordered_map<Cuboid, ItemPlacementPatterns, HomogeneityHash, HomogeneityHash>
        GeneratePlacementPatterns(const Container& container,
                                  std::vector<Cuboid>& items,
                                  PlacementPattern placementPatternTypeX,
                                  PlacementPattern placementPatternTypeY,
                                  PlacementPattern placementPatternTypeZ);

    static std::tuple<std::unordered_map<Cuboid, ItemPlacementPatterns, HomogeneityHash, HomogeneityHash>,
                      ItemPlacementPatterns>
        GeneratePlacementPatternsWithUnion(const Container& container,
                                           std::vector<Cuboid>& items,
                                           PlacementPattern placementPatternTypeX,
                                           PlacementPattern placementPatternTypeY,
                                           PlacementPattern placementPatternTypeZ);

    static void CountPlacementPoints(const std::vector<boost::dynamic_bitset<>>& itemSpecificPatternBitsetX,
                                     const std::vector<boost::dynamic_bitset<>>& itemSpecificPatternBitsetY,
                                     const std::vector<boost::dynamic_bitset<>>& itemSpecificPatternBitsetZ);
};

}
}