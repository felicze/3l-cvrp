#pragma once

#include "CommonBasics/Helper/ModelServices.h"
#include "ContainerLoading/Model/Container.h"
#include "ContainerLoading/Model/ContainerLoadingInstance.h"

#include <utility>
#include <vector>

namespace VehicleRouting
{
namespace Model
{
using namespace ContainerLoading::Model;

class Node : public Group
{
  public:
    Node() = default;
    // InternId corresponds to index in vector Nodes in Instance
    Node(size_t internId,
         size_t externId,
         double xPos,
         double yPos,
         double totalWeight,
         double totalVolume,
         double totalArea,
         std::vector<Cuboid> items)
    : Group(internId, externId, xPos, yPos, totalWeight, totalVolume, totalArea, std::move(items))
    {
    }
};

struct Arc
{
    double Coefficient;
    size_t Tail;
    size_t Head;

    Arc(const double coef, const size_t i, const size_t j) : Coefficient(coef), Tail(i), Head(j){};
};

class Route
{
  public:
    size_t Id;
    Collections::IdVector Sequence; // InternId of Nodes
    double TotalVolume = 0.0;
    double TotalWeight = 0.0;

    Route(size_t id, Collections::IdVector& sequence) : Id(id), Sequence(sequence){};
};

}
}