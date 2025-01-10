#include "Algorithms/PlacementPoints.h"

#include <iostream>
namespace ContainerLoading
{
using namespace Model;

namespace Algorithms
{
std::tuple<std::vector<int64>, std::vector<int64>, std::vector<int64>>
    PlacementPointGenerator::DetermineMeetInTheMiddlePatterns(const Container& container, std::vector<Cuboid>& items)
{
    boost::dynamic_bitset<> placeableCoordinatesX(container.Dx + 1);
    boost::dynamic_bitset<> placeableCoordinatesY(container.Dy + 1);
    boost::dynamic_bitset<> placeableCoordinatesZ(container.Dz + 1);

    for (size_t i = 0; i < items.size(); ++i)
    {
        const Cuboid& itemI = items[i];

        std::vector<Cuboid*> filteredItems;
        filteredItems.reserve(items.size() - 1);

        for (size_t j = 0; j < items.size(); j++)
        {
            if (i == j)
            {
                continue;
            }

            Cuboid& itemJ = items[j];

            filteredItems.emplace_back(&itemJ);
        }

        auto [placeableItemCoordinatesX, placeableItemCoordinatesY, placeableItemCoordinatesZ] =
            DetermineItemSpecificMeetInTheMiddlePatterns(container, filteredItems, itemI);

        placeableCoordinatesX |= placeableItemCoordinatesX;
        placeableCoordinatesY |= placeableItemCoordinatesY;
        placeableCoordinatesZ |= placeableItemCoordinatesZ;
    }

    std::vector<int64> placementPointsX;
    std::vector<int64> placementPointsY;
    std::vector<int64> placementPointsZ;

    placementPointsX.reserve(placeableCoordinatesX.count());
    placementPointsY.reserve(placeableCoordinatesY.count());
    placementPointsZ.reserve(placeableCoordinatesZ.count());

    for (size_t p = 0; p < placeableCoordinatesX.size(); p++)
    {
        if (placeableCoordinatesX[p])
        {
            placementPointsX.push_back(p);
        }
    }

    for (size_t p = 0; p < placeableCoordinatesY.size(); p++)
    {
        if (placeableCoordinatesY[p])
        {
            placementPointsY.push_back(p);
        }
    }

    for (size_t p = 0; p < placeableCoordinatesZ.size(); p++)
    {
        if (placeableCoordinatesZ[p])
        {
            placementPointsZ.push_back(p);
        }
    }

    return std::make_tuple(std::move(placementPointsX), std::move(placementPointsY), std::move(placementPointsZ));
}

std::tuple<boost::dynamic_bitset<>, boost::dynamic_bitset<>, boost::dynamic_bitset<>>
    PlacementPointGenerator::DetermineItemSpecificMeetInTheMiddlePatterns(const Container& container,
                                                                          const std::vector<Cuboid*>& items,
                                                                          const Cuboid& itemI)
{
    boost::dynamic_bitset<> meetInTheMiddlePointsX(container.Dx + 1);
    boost::dynamic_bitset<> meetInTheMiddlePointsY(container.Dy + 1);
    boost::dynamic_bitset<> meetInTheMiddlePointsZ(container.Dz + 1);

    // Meet-in-the-middle patterns should only be generated for one value of t. When t = binDimension,
    // meet-in-the-middle patterns are equal to regular normal patterns, i.e., MiM_it = \mathcal{B}_i.
    meetInTheMiddlePointsX |= DetermineMeetInTheMiddlePatterns(container, items, itemI, container.Dx, Axis::X);
    meetInTheMiddlePointsY |= DetermineMeetInTheMiddlePatterns(container, items, itemI, container.Dy, Axis::Y);
    meetInTheMiddlePointsZ |= DetermineMeetInTheMiddlePatterns(container, items, itemI, container.Dz, Axis::Z);

    return std::make_tuple(
        std::move(meetInTheMiddlePointsX), std::move(meetInTheMiddlePointsY), std::move(meetInTheMiddlePointsZ));
}

std::vector<ItemPlacementPatternsBitset> PlacementPointGenerator::DetermineMinimalMeetInTheMiddlePatterns(
    const Container& container,
    std::vector<Cuboid>& items,
    MeetInTheMiddleMinimizationTarget minimizationTarget)
{
    std::vector<boost::dynamic_bitset<>> minimalMeetInTheMiddlePatternsX =
        DetermineMinimalMeetInTheMiddlePatterns(container, items, minimizationTarget, Axis::X);
    std::vector<boost::dynamic_bitset<>> minimalMeetInTheMiddlePatternsY =
        DetermineMinimalMeetInTheMiddlePatterns(container, items, minimizationTarget, Axis::Y);
    std::vector<boost::dynamic_bitset<>> minimalMeetInTheMiddlePatternsZ =
        DetermineMinimalMeetInTheMiddlePatterns(container, items, minimizationTarget, Axis::Z);

    std::vector<ItemPlacementPatternsBitset> placementPatterns;
    placementPatterns.reserve(items.size());
    for (size_t i = 0; i < items.size(); i++)
    {
        ItemPlacementPatternsBitset itemPattern;

        boost::dynamic_bitset<>& patternsX = minimalMeetInTheMiddlePatternsX[i];
        boost::dynamic_bitset<>& patternsY = minimalMeetInTheMiddlePatternsY[i];
        boost::dynamic_bitset<>& patternsZ = minimalMeetInTheMiddlePatternsZ[i];

        itemPattern.X = std::move(patternsX);
        itemPattern.Y = std::move(patternsY);
        itemPattern.Z = std::move(patternsZ);

        placementPatterns.emplace_back(std::move(itemPattern));
    }

    return placementPatterns;
}

std::vector<boost::dynamic_bitset<>>
    PlacementPointGenerator::DetermineUnitDiscretizationPoints(const Container& container,
                                                               std::vector<Cuboid>& items,
                                                               Axis axis)
{
    std::vector<boost::dynamic_bitset<>> itemSpecificUnitDiscretizationPoints;
    itemSpecificUnitDiscretizationPoints.reserve(items.size());

    int containerDimension = container.Dimension(axis);

    for (size_t i = 0; i < items.size(); ++i)
    {
        const Cuboid& itemI = items[i];
        int minimumItemDimension = itemI.MinimumRotatableDimension(axis);

        boost::dynamic_bitset<> unitDiscretizationPoints(containerDimension + 1);
        unitDiscretizationPoints.set();

        assert(containerDimension - minimumItemDimension >= 0);
        for (size_t p = containerDimension + 1 - minimumItemDimension; p < unitDiscretizationPoints.size(); p++)
        {
            unitDiscretizationPoints.reset(p);
        }

        itemSpecificUnitDiscretizationPoints.push_back(std::move(unitDiscretizationPoints));
    }

    return itemSpecificUnitDiscretizationPoints;
}

std::vector<boost::dynamic_bitset<>> PlacementPointGenerator::DetermineMinimalMeetInTheMiddlePatterns(
    const Container& container,
    std::vector<Cuboid>& items,
    MeetInTheMiddleMinimizationTarget minimizationTarget,
    Axis axis)
{
    std::vector<boost::dynamic_bitset<>> regularNormalPatterns;
    regularNormalPatterns.reserve(items.size());

    std::vector<int> meetInTheMiddlePointsLeft(container.Dimension(axis) + 1, 0);
    std::vector<int> meetInTheMiddlePointsRight(container.Dimension(axis) + 1, 0);

    for (size_t i = 0; i < items.size(); ++i)
    {
        const Cuboid& itemI = items[i];

        std::vector<Cuboid*> filteredItems;
        filteredItems.reserve(items.size() - 1);

        for (size_t j = 0; j < items.size(); j++)
        {
            if (i == j)
            {
                continue;
            }

            Cuboid& itemJ = items[j];

            filteredItems.emplace_back(&itemJ);
        }

        boost::dynamic_bitset<> regularNormalPattern = DetermineRegularNormalPatterns(container,
                                                                                      filteredItems,
                                                                                      itemI,
                                                                                      meetInTheMiddlePointsLeft,
                                                                                      meetInTheMiddlePointsRight,
                                                                                      minimizationTarget,
                                                                                      axis);

        regularNormalPatterns.emplace_back(std::move(regularNormalPattern));
    }

    // Determine cumulative placement points.
    for (size_t p = 1; p <= container.Dimension(axis); p++)
    {
        meetInTheMiddlePointsLeft[p] += meetInTheMiddlePointsLeft[p - 1];
        meetInTheMiddlePointsRight[container.Dimension(axis) - p] +=
            meetInTheMiddlePointsRight[container.Dimension(axis) - (p - 1)];
    }

    // Determine tMin as threshold for minimal placement points.
    int tMin = 1;
    int min = meetInTheMiddlePointsLeft[0] + meetInTheMiddlePointsRight[1];

    for (size_t p = 2; p <= container.Dimension(axis); p++)
    {
        if (meetInTheMiddlePointsLeft[p - 1] + meetInTheMiddlePointsRight[p] < min)
        {
            min = meetInTheMiddlePointsLeft[p - 1] + meetInTheMiddlePointsRight[p];
            tMin = p;
        }
    }

    // Build minimal meet-in-the-middle set according to tMin.
    std::vector<boost::dynamic_bitset<>> itemSpecificMeetInTheMiddlePoints;

    bool enablePreprocessingStep1 = true;
    bool enablePreprocessingStep2 = true;
    if (enablePreprocessingStep1 || enablePreprocessingStep2)
    {
        // Is it guaranteed that the reduced set is also minimal for tMin (tMin is determined without
        // preprocessing)? Probably not.
        // TODO.Design: preprocessing could also be integrated into GenerateMinimalMeetInTheMiddlePatternsX(). Could it
        // be integrated even earlier to guarantee to find tMin?
        itemSpecificMeetInTheMiddlePoints = DetermineReducedMeetInTheMiddlePatterns(
            items, container, tMin, axis, enablePreprocessingStep1, enablePreprocessingStep2);
    }
    else
    {
        itemSpecificMeetInTheMiddlePoints =
            GenerateMeetInTheMiddlePatterns(items, container, regularNormalPatterns, tMin, axis);
    }

    return itemSpecificMeetInTheMiddlePoints;
}

/// Preprocessing steps according to Côté, J. F., & Iori, M. (2018). The meet-in-the-middle principle for cutting and
/// packing problems. INFORMS Journal on Computing, 30(4), 646-661.
std::vector<boost::dynamic_bitset<>>
    PlacementPointGenerator::DetermineReducedMeetInTheMiddlePatterns(std::vector<Cuboid>& items,
                                                                     const Container& container,
                                                                     int threshold,
                                                                     Axis axis,
                                                                     bool enablePreprocessingStep1,
                                                                     bool enablePreprocessingStep2)
{
    // Preprocessing step 1 using Proposition 5.
    // Determine minimum item.
    int minimalItemIndex = 0;
    int minOverallDimension = items.front().MinimumRotatableDimension(axis);

    for (size_t k = 1; k < items.size(); k++)
    {
        const Cuboid& itemK = items[k];

        int itemDimension = itemK.MinimumRotatableDimension(axis);
        if (itemDimension < minOverallDimension)
        {
            minOverallDimension = itemDimension;
            minimalItemIndex = k;
        }
    }

    int separationThreshold = std::ceil((container.Dimension(axis) - minOverallDimension) / 2.0);

    std::vector<boost::dynamic_bitset<>> itemSpecificPlacementPointsLeft;
    std::vector<boost::dynamic_bitset<>> itemSpecificPlacementPointsRightPrime;

    itemSpecificPlacementPointsLeft.reserve(items.size());
    itemSpecificPlacementPointsRightPrime.reserve(items.size());

    // Determine left and right patterns. If Proposition 5 is enabled, with additionally reduced items sets. If it is
    // disabled with the standard item set resp. the standard meet-in-the-middle procedure, cp. Algorithm 2 in the
    // paper.
    for (size_t i = 0; i < items.size(); i++)
    {
        const Cuboid& item = items[i];
        int minSelectedItemDimension = item.MinimumRotatableDimension(axis);

        boost::dynamic_bitset<> placementPointsLeft;
        boost::dynamic_bitset<> placementPointsRightPrime;

        std::vector<Cuboid*> filteredItems;
        std::vector<Cuboid*> doublyFilteredItems;
        filteredItems.reserve(items.size() - 1);
        doublyFilteredItems.reserve(items.size() - 1);

        for (size_t k = 0; k < items.size(); k++)
        {
            if (k == i)
            {
                // Do not filter k == minimalItemIndex if Proposition 5 is disabled, s.t. doublyFilteredItems is only
                // reduced by i, as in the normal procedure.
                continue;
            }

            filteredItems.emplace_back(&items[k]);

            if (k == minimalItemIndex)
            {
                continue;
            }

            doublyFilteredItems.emplace_back(&items[k]);
        }

        if (!enablePreprocessingStep1)
        {
            DetermineSingleItemReducedLeftRightPatterns(threshold,
                                                        separationThreshold,
                                                        placementPointsLeft,
                                                        container,
                                                        axis,
                                                        minSelectedItemDimension,
                                                        filteredItems,
                                                        placementPointsRightPrime,
                                                        filteredItems);
        }
        else
        {
            DetermineSingleItemReducedLeftRightPatterns(threshold,
                                                        separationThreshold,
                                                        placementPointsLeft,
                                                        container,
                                                        axis,
                                                        minSelectedItemDimension,
                                                        doublyFilteredItems,
                                                        placementPointsRightPrime,
                                                        filteredItems);
        }

        itemSpecificPlacementPointsLeft.emplace_back(std::move(placementPointsLeft));
        itemSpecificPlacementPointsRightPrime.emplace_back(std::move(placementPointsRightPrime));
    }

    std::vector<boost::dynamic_bitset<>> itemSpecificPlacementPointsRight;
    itemSpecificPlacementPointsRight.reserve(items.size());
    std::vector<boost::dynamic_bitset<>> itemSpecificMeetInTheMiddleSets;
    itemSpecificMeetInTheMiddleSets.reserve(items.size());

    for (size_t i = 0; i < items.size(); i++)
    {
        const Cuboid& itemI = items[i];
        const int minItemDimensionI = itemI.MinimumRotatableDimension(axis);

        const boost::dynamic_bitset<>& placementPointsRightPrime = itemSpecificPlacementPointsRightPrime[i];
        boost::dynamic_bitset<> placementPointsRight(container.Dimension(axis) + 1);

        // Placement points placementPointsRightPrime (R'_it) are generated as a left-side pattern. But, we want to
        // obtain the corresponding right-side pattern (placementPointsRight, R_it). So, R'_it must be mirrored to
        // obtain R_it.
        for (size_t p = 0; p < placementPointsRightPrime.size(); p++)
        {
            if (placementPointsRightPrime[p])
            {
                placementPointsRight.set((size_t)container.Dimension(axis) - (size_t)minItemDimensionI - p);
            }
        }

        itemSpecificPlacementPointsRight.emplace_back(std::move(placementPointsRight));

        // Then build the union of placementPointsLeft (L_it) and R_it to obtain the set of meet-in-the-middle points
        // (M_it).
        itemSpecificMeetInTheMiddleSets.emplace_back(
            std::move(itemSpecificPlacementPointsLeft[i] | itemSpecificPlacementPointsRight[i]));
    }

    if (!enablePreprocessingStep2)
    {
        return itemSpecificMeetInTheMiddleSets;
    }

    // Preprocessing step 2 using Proposition 6 and Proposition 7.
    // Initialize modifiable item dimensions.
    std::vector<std::vector<int>> itemSpecificModifiedItemDimensions;
    itemSpecificModifiedItemDimensions.reserve(items.size() + 1);
    for (size_t i = 0; i < items.size(); i++)
    {
        const Cuboid& itemI = items[i];
        int minItemDimensionI = itemI.MinimumRotatableDimension(axis);

        itemSpecificModifiedItemDimensions.emplace_back(
            std::vector<int>(container.Dimension(axis) + 1, minItemDimensionI));
    }

    // Proposition 6 for left.
    DetermineEnlargedItemDimensionsLeft(items,
                                        axis,
                                        itemSpecificModifiedItemDimensions,
                                        itemSpecificPlacementPointsLeft,
                                        container,
                                        itemSpecificMeetInTheMiddleSets);

    // Proposition 6 for right.
    DetermineEnlargedItemDimensionsRight(items,
                                         axis,
                                         itemSpecificModifiedItemDimensions,
                                         itemSpecificPlacementPointsRight,
                                         itemSpecificMeetInTheMiddleSets);

    // Merge left and right patterns to meet in the middle sets
    // Necessary since right placement points may be changed in DetermineEnlargedItemDimensionsRight()
    for (size_t i = 0; i < items.size(); i++)
    {
        itemSpecificMeetInTheMiddleSets[i] = itemSpecificPlacementPointsLeft[i] | itemSpecificPlacementPointsRight[i];
    }

    // Proposition 7
    RemoveRedundantPatterns(items, itemSpecificModifiedItemDimensions, itemSpecificMeetInTheMiddleSets);

    return itemSpecificMeetInTheMiddleSets;
}

void PlacementPointGenerator::DetermineSingleItemReducedLeftRightPatterns(
    int threshold,
    int separationThreshold,
    boost::dynamic_bitset<>& placementPointsLeft,
    const Container& container,
    Axis axis,
    int minSelectedItemDimension,
    const std::vector<Cuboid*>& doublyFilteredItems,
    boost::dynamic_bitset<>& placementPointsRightPrime,
    const std::vector<Cuboid*>& filteredItems)
{
    std::vector<Cuboid*> filteredItemsA;
    std::vector<Cuboid*> filteredItemsB;

    if (threshold <= separationThreshold)
    {
        // Remove item k from left.
        filteredItemsA = filteredItems;
        filteredItemsB = doublyFilteredItems;
    }
    else
    {
        // t >= separationThreshold + 1
        // Remove item k from right.
        filteredItemsA = doublyFilteredItems;
        filteredItemsB = filteredItems;

        // Example 1.
        // Two items with dimensions = {0: 2, 1: 5}. Assume t = container.Dx = 20, so t > separationThreshold = 9. A
        // feasible solution must exist (for any t). Because t = container.Dx, M_i == \mathcal{B}_i. When itemI = 0:
        // minimalItemIndex = 0, items = {1}, filteredItems = {1},
        //                  L_0 = {0, 5}, R'_0 = {} because container.Dx - minSelectedItemDimension - t <= 0.
        // When itemI = 1:  minimalItemIndex = 0, items = {0}, filteredItems = {},
        //                  L_1 = {0, 2}, R'_1 = {}.
        // A solution exists that satisfies the MiM principle, e.g. {0: [0, 2], 1: [2, 7]}.
        //
        // TODO.Doc: Example with right aligned items that satisfies MiM principle.
    }

    switch (axis)
    {
        case Axis::X:
            placementPointsLeft = DetermineRegularNormalPatternsX(
                std::min(threshold - 1, container.Dimension(axis) - minSelectedItemDimension),
                container.Dimension(axis),
                filteredItemsB);
            placementPointsRightPrime =
                DetermineRegularNormalPatternsX(container.Dimension(axis) - minSelectedItemDimension - threshold,
                                                container.Dimension(axis),
                                                filteredItemsA);
            break;
        case Axis::Y:
            placementPointsLeft = DetermineRegularNormalPatternsY(
                std::min(threshold - 1, container.Dimension(axis) - minSelectedItemDimension),
                container.Dimension(axis),
                filteredItemsB);
            placementPointsRightPrime =
                DetermineRegularNormalPatternsY(container.Dimension(axis) - minSelectedItemDimension - threshold,
                                                container.Dimension(axis),
                                                filteredItemsA);
            break;
        case Axis::Z:
            placementPointsLeft = DetermineRegularNormalPatternsZ(
                std::min(threshold - 1, container.Dimension(axis) - minSelectedItemDimension),
                container.Dimension(axis),
                filteredItemsB);
            placementPointsRightPrime =
                DetermineRegularNormalPatternsZ(container.Dimension(axis) - minSelectedItemDimension - threshold,
                                                container.Dimension(axis),
                                                filteredItemsA);
            break;
        default:
            throw std::runtime_error("Undefined axis.");
    }
}

void PlacementPointGenerator::DetermineEnlargedItemDimensionsLeft(
    std::vector<Cuboid>& items,
    Axis axis,
    std::vector<std::vector<int>>& itemSpecificModifiedItemDimensions,
    const std::vector<boost::dynamic_bitset<>>& itemSpecificPlacementPointsLeft,
    const Container& container,
    const std::vector<boost::dynamic_bitset<>>& preliminaryItemSpecificMeetInTheMiddleSets)
{
    for (size_t k = 0; k < items.size(); k++)
    {
        const Cuboid& itemK = items[k];
        int minSelectedItemDimension = itemK.MinimumRotatableDimension(axis);

        std::vector<int>& modifiedItemDimensionsK = itemSpecificModifiedItemDimensions[k];
        const boost::dynamic_bitset<>& placementPointsLeft = itemSpecificPlacementPointsLeft[k];
        if (placementPointsLeft.count() == 0)
            continue;

        for (size_t p = 0, n = placementPointsLeft.size() - minSelectedItemDimension; p < n; p++)
        {
            if (!placementPointsLeft[p])
            {
                continue;
            }

            int sMin = container.Dimension(axis);

            for (size_t i = 0; i < items.size(); i++)
            {
                if (i == k)
                {
                    continue;
                }

                const Cuboid& itemI = items[i];
                int minSelectedItemDimensionI = itemI.MinimumRotatableDimension(axis);
                const boost::dynamic_bitset<>& meetInTheMiddlePointsI = preliminaryItemSpecificMeetInTheMiddleSets[i];

                for (size_t s = 0, m = meetInTheMiddlePointsI.size() - minSelectedItemDimensionI; s < m; s++)
                {
                    if (!meetInTheMiddlePointsI[s])
                    {
                        continue;
                    }

                    if (s >= p + modifiedItemDimensionsK[p])
                    {
                        sMin = std::min(sMin, (int)s);
                        break;
                    }
                }
            }

            int q = std::min(container.Dimension(axis), sMin);

            if (q > p)
            {
                assert(q - p >= minSelectedItemDimension);
                modifiedItemDimensionsK[p] = q - p;
            }
        }
    }
}

void PlacementPointGenerator::DetermineEnlargedItemDimensionsRight(
    std::vector<Cuboid>& items,
    Axis axis,
    std::vector<std::vector<int>>& itemSpecificModifiedItemDimensions,
    std::vector<boost::dynamic_bitset<>>& itemSpecificPlacementPointsRight,
    const std::vector<boost::dynamic_bitset<>>& preliminaryItemSpecificMeetInTheMiddleSets)
{
    for (size_t k = 0; k < items.size(); k++)
    {
        const Cuboid& itemK = items[k];
        const int minSelectedItemDimension = itemK.MinimumRotatableDimension(axis);

        std::vector<int>& modifiedItemDimensionsK = itemSpecificModifiedItemDimensions[k];
        boost::dynamic_bitset<>& placementPointsRight = itemSpecificPlacementPointsRight[k];
        if (placementPointsRight.count() == 0)
            continue;

        for (size_t p = 0; p < placementPointsRight.size() - minSelectedItemDimension; p++)
        {
            if (!placementPointsRight[p])
            {
                continue;
            }

            int sMax = 0;

            for (size_t i = 0; i < items.size(); i++)
            {
                if (i == k)
                {
                    continue;
                }
                const Cuboid& itemI = items[i];
                int minSelectedItemDimensionI = itemI.MinimumRotatableDimension(axis);
                const boost::dynamic_bitset<>& meetInTheMiddlePointsI = preliminaryItemSpecificMeetInTheMiddleSets[i];
                std::vector<int>& modifiedItemDimensionsI = itemSpecificModifiedItemDimensions[i];

                for (size_t s = 0; s < meetInTheMiddlePointsI.size() - minSelectedItemDimensionI; s++)
                {
                    if (!meetInTheMiddlePointsI[s])
                    {
                        continue;
                    }

                    if (s + modifiedItemDimensionsI[s] <= p)
                    {
                        sMax = std::max(sMax, (int)s + modifiedItemDimensionsI[s]);
                    }
                    else
                    {
                        break;
                    }
                }
            }

            int q = std::max(0, sMax);

            if (q < p)
            {
                placementPointsRight.set(q);
                placementPointsRight.reset(p);

                modifiedItemDimensionsK[q] = p + minSelectedItemDimension - q;
            }
        }
    }
}

void PlacementPointGenerator::RemoveRedundantPatterns(
    std::vector<Cuboid>& items,
    const std::vector<std::vector<int>>& itemSpecificModifiedItemDimensions,
    std::vector<boost::dynamic_bitset<>>& itemSpecificPlacementPoints)
{
    for (size_t k = 0; k < items.size(); k++)
    {
        const Cuboid& itemK = items[k];

        const std::vector<int>& modifiedItemDimensionsK = itemSpecificModifiedItemDimensions[k];
        boost::dynamic_bitset<>& placementPoints = itemSpecificPlacementPoints[k];

        for (size_t p = 0; p < placementPoints.size() - 1; p++)
        {
            if (!placementPoints[p])
            {
                continue;
            }

            // p < s
            for (size_t s = p + 1; s < placementPoints.size(); s++)
            {
                if (!placementPoints[s])
                {
                    continue;
                }

                int modifiedWidthP = modifiedItemDimensionsK[p];
                int modifiedWidthS = modifiedItemDimensionsK[s];
                if (s + modifiedWidthS <= p + modifiedWidthP)
                {
                    placementPoints.reset(p);
                }
            }
        }
    }
}

std::vector<boost::dynamic_bitset<>> PlacementPointGenerator::GenerateMeetInTheMiddlePatterns(
    std::vector<Cuboid>& items,
    const Container& container,
    const std::vector<boost::dynamic_bitset<>>& regularNormalPatterns,
    int threshold,
    Axis axis)
{
    std::vector<boost::dynamic_bitset<>> itemSpecificMeetInTheMiddlePoints;
    itemSpecificMeetInTheMiddlePoints.reserve(items.size());

    for (size_t i = 0; i < items.size(); ++i)
    {
        boost::dynamic_bitset<> meetInTheMiddlePoints(container.Dimension(axis) + 1);

        // Item specific meet-in-the-middle patterns \mathcal{M}_i can be extracted from this loop.
        const Cuboid& itemI = items[i];
        const boost::dynamic_bitset<>& regularNormalPattern = regularNormalPatterns[i];

        for (size_t p = 0; p < regularNormalPattern.size(); p++)
        {
            if (!regularNormalPattern[p])
            {
                continue;
            }

            if (p < threshold)
            {
                meetInTheMiddlePoints.set(p);
            }

            int itemDimension = itemI.MinimumRotatableDimension(axis);

            if ((size_t)container.Dimension(axis) - (size_t)itemDimension - p >= threshold)
            {
                meetInTheMiddlePoints.set((size_t)container.Dimension(axis) - (size_t)itemDimension - p);
            }
        }

        itemSpecificMeetInTheMiddlePoints.emplace_back(std::move(meetInTheMiddlePoints));
    }

    return itemSpecificMeetInTheMiddlePoints;
}

boost::dynamic_bitset<>
    PlacementPointGenerator::DetermineRegularNormalPatterns(const Container& container,
                                                            const std::vector<Cuboid*>& items,
                                                            const Cuboid& itemI,
                                                            std::vector<int>& meetInTheMiddlePointsLeft,
                                                            std::vector<int>& meetInTheMiddlePointsRight,
                                                            MeetInTheMiddleMinimizationTarget minimizationTarget,
                                                            Axis axis)
{
    // Note: with rotation, item specific normal patterns can be reduced by creating item specific normal patterns for
    // each rotation separately. This is only useful, when item specific placement points are actually used in the model
    // formulation.
    int itemDimension = itemI.MinimumRotatableDimension(axis);

    boost::dynamic_bitset<> regularNormalPatterns;
    switch (axis)
    {
        case Axis::X:
            regularNormalPatterns = DetermineRegularNormalPatternsX(
                container.Dimension(axis) - itemDimension, container.Dimension(axis), items);
            break;
        case Axis::Y:
            regularNormalPatterns = DetermineRegularNormalPatternsY(
                container.Dimension(axis) - itemDimension, container.Dimension(axis), items);
            break;
        case Axis::Z:
            regularNormalPatterns = DetermineRegularNormalPatternsZ(
                container.Dimension(axis) - itemDimension, container.Dimension(axis), items);
            break;
        default:
            break;
    }

    for (size_t p = 0; p < regularNormalPatterns.size(); p++)
    {
        if (regularNormalPatterns[p])
        {
            switch (minimizationTarget)
            {
                case PlacementPointGenerator::MeetInTheMiddleMinimizationTarget::IndividualPlacementPoints:
                    meetInTheMiddlePointsLeft[p]++;
                    meetInTheMiddlePointsRight[(size_t)container.Dimension(axis) - (size_t)itemDimension - p]++;
                    break;
                case PlacementPointGenerator::MeetInTheMiddleMinimizationTarget::PlacementPointUnion:
                    meetInTheMiddlePointsLeft[p] = 1;
                    meetInTheMiddlePointsRight[(size_t)container.Dimension(axis) - (size_t)itemDimension - p] = 1;
                    break;
                default:
                    break;
            }
        }
    }

    return regularNormalPatterns;
}

boost::dynamic_bitset<> PlacementPointGenerator::DetermineMeetInTheMiddlePatterns(const Container& container,
                                                                                  const std::vector<Cuboid*>& items,
                                                                                  const Cuboid& itemI,
                                                                                  int threshold,
                                                                                  Axis axis)
{
    int minDimension = itemI.MinimumRotatableDimension(axis);

    boost::dynamic_bitset<> meetInTheMiddlePoints;
    boost::dynamic_bitset<> placementPointsRightPrime;
    switch (axis)
    {
        case Axis::X:
            meetInTheMiddlePoints = DetermineRegularNormalPatternsX(
                std::min(threshold - 1, container.Dimension(axis) - minDimension), container.Dimension(axis), items);
            placementPointsRightPrime = DetermineRegularNormalPatternsX(
                container.Dimension(axis) - minDimension - threshold, container.Dimension(axis), items);
            break;
        case Axis::Y:
            meetInTheMiddlePoints = DetermineRegularNormalPatternsY(
                std::min(threshold - 1, container.Dimension(axis) - minDimension), container.Dimension(axis), items);
            placementPointsRightPrime = DetermineRegularNormalPatternsY(
                container.Dimension(axis) - minDimension - threshold, container.Dimension(axis), items);
            break;
        case Axis::Z:
            meetInTheMiddlePoints = DetermineRegularNormalPatternsZ(
                std::min(threshold - 1, container.Dimension(axis) - minDimension), container.Dimension(axis), items);
            placementPointsRightPrime = DetermineRegularNormalPatternsZ(
                container.Dimension(axis) - minDimension - threshold, container.Dimension(axis), items);
            break;
        default:
            break;
    }

    for (size_t p = 0; p < placementPointsRightPrime.size(); p++)
    {
        if (placementPointsRightPrime[p])
        {
            meetInTheMiddlePoints.set((size_t)container.Dx - (size_t)minDimension - p);
        }
    }

    return meetInTheMiddlePoints;
}

boost::dynamic_bitset<> PlacementPointGenerator::DetermineRegularNormalPatternsX(int containerDx,
                                                                                 int actualContainerDx,
                                                                                 const std::vector<Cuboid*>& items)
{
    // + 1 can be neglected, because at coordinate actualContainerDx, no item with itemDx > 0 can ever be placed.
    // boost::dynamic_bitset, because of performance when building set intersections in the calling methods. Instead,
    // std::vector can also be used.
    boost::dynamic_bitset<> xT(actualContainerDx + 1);

    if (containerDx < 0)
    {
        return xT;
    }

    xT.set(0);

    if (containerDx == 0)
    {
        return xT;
    }

    for (size_t i = 0; i < items.size(); ++i)
    {
        const Cuboid& item = *items[i];

        int minLength = item.EnableHorizontalRotation ? std::min(item.Dx, item.Dy) : item.Dx;

        for (int p = containerDx - minLength; p > -1; --p)
        {
            if (xT[p])
            {
                // + 1 --> p + item.Dx <= containerDx instead of <, cf. Algorithm 1 in Côté, J. F., & Iori, M. (2018).
                // The meet-in-the-middle principle for cutting and packing problems. INFORMS Journal on Computing,
                // 30(4), 646-661.
                if (p + item.Dx <= containerDx)
                {
                    xT.set(p + item.Dx);
                }

                if (item.EnableHorizontalRotation && p + item.Dy <= containerDx)
                {
                    xT.set(p + item.Dy);
                }
            }
        }
    }

    return xT;
}

boost::dynamic_bitset<> PlacementPointGenerator::DetermineRegularNormalPatternsY(int containerDy,
                                                                                 int actualContainerDy,
                                                                                 const std::vector<Cuboid*>& items)
{
    boost::dynamic_bitset<> yT(actualContainerDy + 1, 0);

    if (containerDy < 0)
    {
        return yT;
    }

    yT.set(0);

    if (containerDy == 0)
    {
        return yT;
    }

    for (size_t i = 0; i < items.size(); ++i)
    {
        const Cuboid& item = *items[i];
        int minWidth = item.EnableHorizontalRotation ? std::min(item.Dx, item.Dy) : item.Dy;

        for (int p = containerDy - minWidth; p > -1; --p)
        {
            if (yT[p])
            {
                if (p + item.Dy <= containerDy)
                {
                    yT.set(p + item.Dy);
                }

                if (item.EnableHorizontalRotation && p + item.Dx <= containerDy)
                {
                    yT.set(p + item.Dx);
                }
            }
        }
    }

    return yT;
}

boost::dynamic_bitset<> PlacementPointGenerator::DetermineRegularNormalPatternsZ(int containerDz,
                                                                                 int actualContainerDz,
                                                                                 const std::vector<Cuboid*>& items)
{
    boost::dynamic_bitset<> zT(actualContainerDz + 1, 0);

    if (containerDz < 0)
    {
        return zT;
    }

    zT.set(0);

    if (containerDz == 0)
    {
        return zT;
    }

    for (size_t i = 0; i < items.size(); ++i)
    {
        const Cuboid& item = *items[i];

        for (int p = containerDz - item.Dz; p > -1; --p)
        {
            if (zT[p] && p + item.Dz <= containerDz)
            {
                zT.set(p + item.Dz);
            }
        }
    }

    return zT;
}

std::tuple<PlacementPattern, PlacementPattern, PlacementPattern>
    PlacementPointGenerator::SelectMinimalFeasiblePatternType(LoadingFlag loadingMask)
{
    // When the loading mask is passed from LoadingChecker, we need to use the mask and cannot use the PackingType that
    // is available there, because the loading mask might be different from the PackingType.
    switch (loadingMask)
    {
        // Only check against concatenated flags.
        case LoadingFlag::Complete:
        case LoadingFlag::NoFragility:
        case LoadingFlag::NoLifo:
            // UD in X and Y because of support. R-NP in Z is sufficient because items must not hover.
            return std::tuple(PlacementPattern::UnitDiscretization,
                              PlacementPattern::UnitDiscretization,
                              PlacementPattern::RegularNormalPatterns);
        case LoadingFlag::NoSupport:
        case LoadingFlag::FragilityOnly:
            // UD in Z because items can hover, which is sometimes necessary to not violate fragility or produce
            // feasibility with hovering. R-NP in X and Y is sufficient because there always exists a placement point
            // that does not overlap with another item, which retains feasibility w.r.t. fragility. MiM not feasible
            // with lifo constraints.
            return std::tuple(PlacementPattern::RegularNormalPatterns,
                              PlacementPattern::RegularNormalPatterns,
                              PlacementPattern::UnitDiscretization);
        case LoadingFlag::LifoNoSequence:
        case LoadingFlag::LifoSequence:
            // R-NP in Z is sufficient because items must not hover.
            return std::tuple(PlacementPattern::RegularNormalPatterns,
                              PlacementPattern::RegularNormalPatterns,
                              PlacementPattern::RegularNormalPatterns);
        case LoadingFlag::LoadingOnly:
            // Standard 3D-OPP where the only constraint is no overlap.
            // Regular normal patterns with domain reduction according to Soh (2010) would also be feasible.
            return std::tuple(PlacementPattern::MeetInTheMiddle,
                              PlacementPattern::MeetInTheMiddle,
                              PlacementPattern::MeetInTheMiddle);
        default:
            throw std::runtime_error("No pattern type for loading mask implemented.");
    }
}

/*
std::vector<int64> PlacementPointGenerator::DetermineNormalPatternsX(
    const Container& container,
    const std::vector<Cuboid>& items)
{
    std::vector<int> xT(container.Dx, 0);

    xT[0] = 1;

    int minLengthAllItems = container.Dx;

    for (size_t i = 0; i < items.size(); ++i)
    {
        const Cuboid& item = items[i];

        int minLength = item.EnableHorizontalRotation ? std::min(item.Dx, item.Dy) : item.Dx;

        minLengthAllItems = minLength < minLengthAllItems ? minLength : minLengthAllItems;

        for (int p = container.Dx - minLength; p > -1; --p)
        {
            if (xT[p] == 1)
            {
                if (p + item.Dx < container.Dx)
                {
                    xT[p + item.Dx] = 1;
                }

                if (item.EnableHorizontalRotation && p + item.Dy < container.Dx)
                {
                    xT[p + item.Dy] = 1;
                }
            }
        }
    }

    std::vector<int64> xNormalPatterns;
    xNormalPatterns.reserve(container.Dx);

    for (int64 p = 0; p <= container.Dx - minLengthAllItems; ++p)
    {
        if (xT[p] == 1)
        {
            xNormalPatterns.push_back(p);
        }
    }
    return xNormalPatterns;
}

std::vector<int64> PlacementPointGenerator::DetermineNormalPatternsY(
    const Container& container,
    const std::vector<Cuboid>& items)
{
    std::vector<int> yT(container.Dy, 0);

    yT[0] = 1;

    int minWidthAllItems = container.Dy;

    for (size_t i = 0; i < items.size(); ++i)
    {
        const Cuboid& item = items[i];
        int minWidth = item.EnableHorizontalRotation ? std::min(item.Dx, item.Dy) : item.Dy;

        minWidthAllItems = minWidth < minWidthAllItems ? minWidth : minWidthAllItems;

        for (int p = container.Dy - minWidth; p > -1; --p)
        {
            if (yT[p] == 1)
            {
                if (p + item.Dy < container.Dy)
                {
                    yT[p + item.Dy] = 1;
                }
                if (item.EnableHorizontalRotation && p + item.Dx < container.Dy)
                {
                    yT[p + item.Dx] = 1;
                }

            }
        }
    }

    std::vector<int64> yNormalPatterns;
    yNormalPatterns.reserve(container.Dy);

    for (int64 p = 0; p <= container.Dy - minWidthAllItems; ++p)
    {
        if (yT[p] == 1)
        {
            yNormalPatterns.push_back(p);
        }
    }

    return yNormalPatterns;
}

std::vector<int64> PlacementPointGenerator::DetermineNormalPatternsZ(
    const Container& container,
    const std::vector<Cuboid>& items)
{
    std::vector<int> zT(container.Dz, 0);

    zT[0] = 1;

    int minHeightAllItems = container.Dz;

    for (size_t i = 0; i < items.size(); ++i)
    {
        const Cuboid& item = items[i];

        minHeightAllItems = item.Dz < minHeightAllItems ? item.Dz : minHeightAllItems;

        ////if (item.EnableStacking)
        ////{
        for (int p = container.Dz - item.Dz; p > -1; --p)
        {
            if (zT[p] == 1 && p + item.Dz < container.Dz)
            {
                zT[p + item.Dz] = 1;
            }
        }
        ////}
    }

    std::vector<int64> zNormalPatterns;
    zNormalPatterns.reserve(container.Dz);

    for (int64 p = 0; p <= container.Dz - minHeightAllItems; ++p)
    {
        if (zT[p] == 1)
        {
            zNormalPatterns.push_back(p);
        }
    }

    return zNormalPatterns;
}

std::tuple<std::vector<int64>, std::vector<int64>, std::vector<int64>>
PlacementPointGenerator::DetermineNormalPatterns3D(const Container& container, const std::vector<Cuboid>& items)
{
    auto normalPatternsX = PlacementPointGenerator::DetermineNormalPatternsX(container, items);
    auto normalPatternsY = PlacementPointGenerator::DetermineNormalPatternsY(container, items);
    auto normalPatternsZ = PlacementPointGenerator::DetermineNormalPatternsZ(container, items);

    return std::make_tuple(std::move(normalPatternsX), std::move(normalPatternsY), std::move(normalPatternsZ));
}

std::tuple<std::vector<int64>, std::vector<int64>> PlacementPointGenerator::DetermineNormalPatternsXY(const Container&
container, const std::vector<Cuboid>& items)
{
    auto normalPatternsX = PlacementPointGenerator::DetermineNormalPatternsX(container, items);
    auto normalPatternsY = PlacementPointGenerator::DetermineNormalPatternsY(container, items);

    return std::make_tuple(std::move(normalPatternsX), std::move(normalPatternsY));
}
*/

std::vector<int64>
    PlacementPointGenerator::ConvertPlacementBitsetToVector(const boost::dynamic_bitset<>& itemSpecificPatternBitset)
{
    std::vector<int64> itemSpecificPattern;
    itemSpecificPattern.reserve(itemSpecificPatternBitset.count());

    for (size_t p = 0; p < itemSpecificPatternBitset.size(); p++)
    {
        if (itemSpecificPatternBitset[p])
        {
            itemSpecificPattern.push_back(p);
        }
    }

    return itemSpecificPattern;
}

ItemPlacementPatterns
    PlacementPointGenerator::ConvertPlacementBitsetToVector(const boost::dynamic_bitset<>& itemSpecificPatternBitsetX,
                                                            const boost::dynamic_bitset<>& itemSpecificPatternBitsetY,
                                                            const boost::dynamic_bitset<>& itemSpecificPatternBitsetZ)
{
    ItemPlacementPatterns itemSpecificPattern;
    itemSpecificPattern.X.reserve(itemSpecificPatternBitsetX.count());
    itemSpecificPattern.Y.reserve(itemSpecificPatternBitsetY.count());
    itemSpecificPattern.Z.reserve(itemSpecificPatternBitsetZ.count());

    for (size_t p = 0; p < itemSpecificPatternBitsetX.size(); p++)
    {
        if (itemSpecificPatternBitsetX[p])
        {
            itemSpecificPattern.X.push_back(p);
        }
    }

    for (size_t p = 0; p < itemSpecificPatternBitsetY.size(); p++)
    {
        if (itemSpecificPatternBitsetY[p])
        {
            itemSpecificPattern.Y.push_back(p);
        }
    }

    for (size_t p = 0; p < itemSpecificPatternBitsetZ.size(); p++)
    {
        if (itemSpecificPatternBitsetZ[p])
        {
            itemSpecificPattern.Z.push_back(p);
        }
    }

    return itemSpecificPattern;
}

ItemPlacementPatterns PlacementPointGenerator::ConvertPlacementBitsetToVector(
    const ItemPlacementPatternsBitset& itemSpecificPatternBitset)
{
    return ConvertPlacementBitsetToVector(
        itemSpecificPatternBitset.X, itemSpecificPatternBitset.Y, itemSpecificPatternBitset.Z);
}

/*
std::vector<int64> PlacementPointGenerator::DetermineStartPointsFlatSpan(const std::vector<int64>& patterns)
{
    std::vector<int64> startPoints;
    startPoints.reserve(patterns.size() * 2);

    int64 startInterval = 0;

    for (size_t p = 1; p < patterns.size(); ++p)
    {
        int prevPoint = patterns[p - 1];
        int currentPoint = patterns[p];

        if (currentPoint == prevPoint + 1)
            continue;

        startPoints.push_back(startInterval);
        startPoints.push_back(patterns[p - 1]);

        startInterval = currentPoint;
    }

    // Contains the same starting points multiple times. Is need for intervals in CP model, compare
FromFlatSpanOfIntervals. startPoints.push_back(startInterval); startPoints.push_back(patterns.back());

    return startPoints;
}
*/

std::vector<int64> PlacementPointGenerator::DetermineEndPoints(const std::vector<int64>& patterns,
                                                               int dim,
                                                               int rotDim,
                                                               bool enableRotation)
{
    std::vector<int64> startPoints;
    startPoints.reserve(patterns.size());
    for (size_t p = 0; p < patterns.size(); ++p)
    {
        int64 startPoint = patterns[p];
        startPoints.push_back(startPoint + dim);

        if (!enableRotation)
            continue;

        if (dim == rotDim)
            continue;

        startPoints.push_back(startPoint + rotDim);
    }

    return startPoints;
}

std::vector<boost::dynamic_bitset<>> PlacementPointGenerator::GenerateRegularNormalPatterns(const Container& container,
                                                                                            std::vector<Cuboid>& items,
                                                                                            Axis axis)
{
    std::vector<boost::dynamic_bitset<>> itemSpecificRegularNormalPatterns;
    itemSpecificRegularNormalPatterns.reserve(items.size());

    for (size_t i = 0; i < items.size(); i++)
    {
        const Cuboid itemI = items[i];
        int itemDimension = itemI.MinimumRotatableDimension(axis);

        std::vector<Cuboid*> filteredItems;
        filteredItems.reserve(items.size() - 1);
        for (size_t j = 0; j < items.size(); j++)
        {
            if (i == j)
            {
                continue;
            }

            Cuboid& itemJ = items[j];
            filteredItems.emplace_back(&itemJ);
        }

        boost::dynamic_bitset<> regularNormalPatterns;
        switch (axis)
        {
            case Axis::X:
                regularNormalPatterns = DetermineRegularNormalPatternsX(
                    container.Dimension(axis) - itemDimension, container.Dimension(axis), filteredItems);
                break;
            case Axis::Y:
                regularNormalPatterns = DetermineRegularNormalPatternsY(
                    container.Dimension(axis) - itemDimension, container.Dimension(axis), filteredItems);
                break;
            case Axis::Z:
                regularNormalPatterns = DetermineRegularNormalPatternsZ(
                    container.Dimension(axis) - itemDimension, container.Dimension(axis), filteredItems);
                break;
            default:
                break;
        }

        itemSpecificRegularNormalPatterns.push_back(std::move(regularNormalPatterns));
    }

    return itemSpecificRegularNormalPatterns;
}

std::vector<boost::dynamic_bitset<>>
    PlacementPointGenerator::GeneratePlacementPatterns(const Container& container,
                                                       std::vector<Cuboid>& items,
                                                       PlacementPattern placementPatternType,
                                                       Axis axis)
{
    std::vector<boost::dynamic_bitset<>> itemSpecificPlacementPatterns;
    itemSpecificPlacementPatterns.reserve(items.size());

    switch (placementPatternType)
    {
        case PlacementPattern::UnitDiscretization:
            itemSpecificPlacementPatterns = DetermineUnitDiscretizationPoints(container, items, axis);
            break;
        case PlacementPattern::RegularNormalPatterns:
            itemSpecificPlacementPatterns = GenerateRegularNormalPatterns(container, items, axis);
            break;
        case PlacementPattern::MeetInTheMiddle:
            // TODO.Design: Parametrize minimization target.
            itemSpecificPlacementPatterns = DetermineMinimalMeetInTheMiddlePatterns(
                container, items, MeetInTheMiddleMinimizationTarget::PlacementPointUnion, axis);
            break;
        case PlacementPattern::None:
        case PlacementPattern::NormalPatterns:
        case PlacementPattern::ReducedRasterPoints:
        default:
            throw std::runtime_error("Placement pattern generation not implemented.");
    }

    return itemSpecificPlacementPatterns;
}

std::vector<std::vector<int64>>
    PlacementPointGenerator::GeneratePlacementPatternsBaseType(const Container& container,
                                                               std::vector<Cuboid>& items,
                                                               PlacementPattern placementPatternType,
                                                               Axis axis)
{
    std::vector<boost::dynamic_bitset<>> itemSpecificPlacementPatterns;
    itemSpecificPlacementPatterns.reserve(items.size());

    switch (placementPatternType)
    {
        case PlacementPattern::UnitDiscretization:
            itemSpecificPlacementPatterns = DetermineUnitDiscretizationPoints(container, items, axis);
            break;
        case PlacementPattern::RegularNormalPatterns:
            itemSpecificPlacementPatterns = GenerateRegularNormalPatterns(container, items, axis);
            break;
        case PlacementPattern::MeetInTheMiddle:
            // TODO.Design: Parametrize minimization target.
            itemSpecificPlacementPatterns = DetermineMinimalMeetInTheMiddlePatterns(
                container, items, MeetInTheMiddleMinimizationTarget::PlacementPointUnion, axis);
            break;
        case PlacementPattern::None:
        case PlacementPattern::NormalPatterns:
        case PlacementPattern::ReducedRasterPoints:
        default:
            throw std::runtime_error("Placement pattern generation not implemented.");
    }

    std::vector<std::vector<int64>> itemSpecificPlacementPatternsVector;
    itemSpecificPlacementPatternsVector.reserve(items.size());
    for (size_t i = 0; i < items.size(); i++)
    {
        auto itemSpecificPlacementPattern =
            PlacementPointGenerator::ConvertPlacementBitsetToVector(itemSpecificPlacementPatterns[i]);
        itemSpecificPlacementPatternsVector.push_back(std::move(itemSpecificPlacementPattern));
    }

    return itemSpecificPlacementPatternsVector;
}

std::unordered_map<Cuboid, ItemPlacementPatterns, HomogeneityHash, HomogeneityHash>
    PlacementPointGenerator::GeneratePlacementPatterns(const Container& container,
                                                       std::vector<Cuboid>& items,
                                                       PlacementPattern placementPatternTypeX,
                                                       PlacementPattern placementPatternTypeY,
                                                       PlacementPattern placementPatternTypeZ)
{
    std::vector<boost::dynamic_bitset<>> placementPatternX =
        GeneratePlacementPatterns(container, items, placementPatternTypeX, Axis::X);
    std::vector<boost::dynamic_bitset<>> placementPatternY =
        GeneratePlacementPatterns(container, items, placementPatternTypeY, Axis::Y);
    std::vector<boost::dynamic_bitset<>> placementPatternZ =
        GeneratePlacementPatterns(container, items, placementPatternTypeZ, Axis::Z);

    ////CountPlacementPoints(placementPatternX, placementPatternY, placementPatternZ);

    std::unordered_map<Cuboid, ItemPlacementPatterns, HomogeneityHash, HomogeneityHash>
        itemTypeSpecificPlacementPatterns;
    itemTypeSpecificPlacementPatterns.reserve(items.size()); // Overestimation when there are homogeneous items.

    for (size_t i = 0; i < items.size(); i++)
    {
        const Cuboid& item = items[i];

        if (itemTypeSpecificPlacementPatterns.contains(item))
        {
            continue;
        }

        ItemPlacementPatterns itemSpecificPattern = PlacementPointGenerator::ConvertPlacementBitsetToVector(
            placementPatternX[i], placementPatternY[i], placementPatternZ[i]);

        itemTypeSpecificPlacementPatterns.emplace(item, std::move(itemSpecificPattern));
    }

    return itemTypeSpecificPlacementPatterns;
}

std::tuple<std::unordered_map<Cuboid, ItemPlacementPatterns, HomogeneityHash, HomogeneityHash>, ItemPlacementPatterns>
    PlacementPointGenerator::GeneratePlacementPatternsWithUnion(const Container& container,
                                                                std::vector<Cuboid>& items,
                                                                PlacementPattern placementPatternTypeX,
                                                                PlacementPattern placementPatternTypeY,
                                                                PlacementPattern placementPatternTypeZ)
{
    std::vector<boost::dynamic_bitset<>> placementPatternX =
        GeneratePlacementPatterns(container, items, placementPatternTypeX, Axis::X);
    std::vector<boost::dynamic_bitset<>> placementPatternY =
        GeneratePlacementPatterns(container, items, placementPatternTypeY, Axis::Y);
    std::vector<boost::dynamic_bitset<>> placementPatternZ =
        GeneratePlacementPatterns(container, items, placementPatternTypeZ, Axis::Z);

    std::unordered_map<Cuboid, ItemPlacementPatterns, HomogeneityHash, HomogeneityHash>
        itemTypeSpecificPlacementPatterns;
    itemTypeSpecificPlacementPatterns.reserve(items.size()); // Overestimation when there are homogeneous items.

    ItemPlacementPatternsBitset placementPointUnionBitset;
    placementPointUnionBitset.X = placementPatternX.front();
    placementPointUnionBitset.Y = placementPatternY.front();
    placementPointUnionBitset.Z = placementPatternZ.front();

    size_t itemsSpecificPlacementPointCountX = 0;
    size_t itemsSpecificPlacementPointCountY = 0;
    size_t itemsSpecificPlacementPointCountZ = 0;

    for (size_t i = 0; i < items.size(); i++)
    {
        const Cuboid& item = items[i];

        itemsSpecificPlacementPointCountX += placementPatternX[i].count();
        itemsSpecificPlacementPointCountY += placementPatternY[i].count();
        itemsSpecificPlacementPointCountZ += placementPatternZ[i].count();

        if (itemTypeSpecificPlacementPatterns.contains(item))
        {
            continue;
        }

        placementPointUnionBitset.X |= placementPatternX[i];
        placementPointUnionBitset.Y |= placementPatternY[i];
        placementPointUnionBitset.Z |= placementPatternZ[i];

        ItemPlacementPatterns itemSpecificPattern = PlacementPointGenerator::ConvertPlacementBitsetToVector(
            placementPatternX[i], placementPatternY[i], placementPatternZ[i]);

        itemTypeSpecificPlacementPatterns.emplace(item, std::move(itemSpecificPattern));
    }

    size_t placementPointUnionCountX = placementPointUnionBitset.X.count();
    size_t placementPointUnionCountY = placementPointUnionBitset.Y.count();
    size_t placementPointUnionCountZ = placementPointUnionBitset.Z.count();

    ItemPlacementPatterns placementPointsUnion =
        PlacementPointGenerator::ConvertPlacementBitsetToVector(placementPointUnionBitset);

    ////std::cout << "Placement points (X, Y, Z): ";
    ////std::cout << "|X_is| = (" << itemsSpecificPlacementPointCountX << ", " << itemsSpecificPlacementPointCountY <<
    ///", " << itemsSpecificPlacementPointCountZ << "), "; /std::cout << "|X_s| = (" << placementPointUnionCountX << ",
    ///" << placementPointUnionCountY << ", " << placementPointUnionCountZ << ")"; /std::cout << "\n";

    return std::tuple(itemTypeSpecificPlacementPatterns, placementPointsUnion);
}

void PlacementPointGenerator::CountPlacementPoints(
    const std::vector<boost::dynamic_bitset<>>& itemSpecificPatternBitsetX,
    const std::vector<boost::dynamic_bitset<>>& itemSpecificPatternBitsetY,
    const std::vector<boost::dynamic_bitset<>>& itemSpecificPatternBitsetZ)
{
    size_t itemsSpecificPlacementPointCountX = 0;
    size_t itemsSpecificPlacementPointCountY = 0;
    size_t itemsSpecificPlacementPointCountZ = 0;

    boost::dynamic_bitset<> placementPointUnionBitsetX = itemSpecificPatternBitsetX.front(); // copy
    boost::dynamic_bitset<> placementPointUnionBitsetY = itemSpecificPatternBitsetY.front(); // copy
    boost::dynamic_bitset<> placementPointUnionBitsetZ = itemSpecificPatternBitsetZ.front(); // copy

    // itemSpecificPatternBitsetX.size() == itemSpecificPatternBitsetY.size() == itemSpecificPatternBitsetZ.size()
    for (size_t i = 0; i < itemSpecificPatternBitsetX.size(); i++)
    {
        placementPointUnionBitsetX |= itemSpecificPatternBitsetX[i];
        placementPointUnionBitsetY |= itemSpecificPatternBitsetY[i];
        placementPointUnionBitsetZ |= itemSpecificPatternBitsetZ[i];

        itemsSpecificPlacementPointCountX += itemSpecificPatternBitsetX[i].count();
        itemsSpecificPlacementPointCountY += itemSpecificPatternBitsetY[i].count();
        itemsSpecificPlacementPointCountZ += itemSpecificPatternBitsetZ[i].count();
    }

    size_t placementPointUnionCountX = placementPointUnionBitsetX.count();
    size_t placementPointUnionCountY = placementPointUnionBitsetY.count();
    size_t placementPointUnionCountZ = placementPointUnionBitsetZ.count();

    std::cout << "Placement points (X, Y, Z): ";
    std::cout << "|X_is| = (" << itemsSpecificPlacementPointCountX << ", " << itemsSpecificPlacementPointCountY << ", "
              << itemsSpecificPlacementPointCountZ << "), ";
    std::cout << "|X_s| = (" << placementPointUnionCountX << ", " << placementPointUnionCountY << ", "
              << placementPointUnionCountZ << ")";
    std::cout << "\n";
}

}
}