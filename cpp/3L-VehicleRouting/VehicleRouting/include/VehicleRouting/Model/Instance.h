#pragma once

#include "CommonBasics/Helper/ModelServices.h"
#include "Node.h"
#include "Vehicle.h"

#include <map>
#include <ranges>
#include <string>
#include <utility>

namespace VehicleRouting
{
namespace Model
{
class Instance
{
  public:
    std::string Name;
    std::vector<Vehicle> Vehicles;
    std::vector<Node> Nodes;
    Collections::IdVector CustomerIds;
    size_t DepotIndex = 0;
    size_t LowerBoundVehicles = 0;

    Instance(std::string name, const std::vector<Vehicle>& vehicles, const std::vector<Node>& nodes)
    : Name(std::move(name)), Vehicles(vehicles), Nodes(nodes)
    {
        DetermineDistanceMatrixEuclidean();

        for (const auto& node: Nodes | std::ranges::views::drop(1))
        {
            CustomerIds.push_back(node.InternId);
        }
    }

    void DetermineDistanceMatrixEuclidean();
    [[nodiscard]] double Distance(size_t tail, size_t head) const { return mDistances.at(tail).at(head); }
    [[nodiscard]] std::span<const Node> GetCustomers() const { return {Nodes.data() + 1, Nodes.size() - 1}; }
    [[nodiscard]] size_t GetDepotId() const { return Nodes[DepotIndex].InternId; }

  private:
    std::map<size_t, std::map<size_t, double>> mDistances;
};

}
}