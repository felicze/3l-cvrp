#include "Algorithms/LoadingInterfaceServices.h"

namespace VehicleRouting
{
namespace Algorithms
{
std::vector<Cuboid> InterfaceConversions::SelectItems(const Collections::IdVector& nodeIds, const std::vector<Node>& nodes, bool reversedDirection)
{
    std::vector<Cuboid> selectedItems;
    if (!reversedDirection)
    {
        for (size_t i = 0; i < nodeIds.size(); ++i)
        {
            const auto& items = nodes[nodeIds[i]].Items;

            for (const auto& item: items)
            {
                selectedItems.emplace_back(item);
                selectedItems.back().GroupId = nodeIds.size() - 1 - i;
            }
        }
    }
    else
    {
        for (size_t i = 0; i < nodeIds.size(); ++i)
        {
            const auto& items = nodes[nodeIds[i]].Items;

            for (const auto& item: items)
            {
                selectedItems.emplace_back(item);
                selectedItems.back().GroupId = i;
            }
        }
    }

    return selectedItems;
}

std::vector<Group> InterfaceConversions::NodesToGroup(const std::vector<Node>& nodes)
{
    std::vector<Group> groups;
    groups.reserve(nodes.size());
    for (const Node& node: nodes)
    {
        groups.emplace_back(node.InternId,
                            node.ExternId,
                            node.PositionX,
                            node.PositionY,
                            node.TotalWeight,
                            node.TotalVolume,
                            node.TotalArea,
                            node.Items);
    }

    return groups;
}

}
}