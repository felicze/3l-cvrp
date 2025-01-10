#pragma once

#include "Container.h"
#include <vector>

namespace ContainerLoading
{
namespace Model
{

class Group
{
  public:
    size_t InternId;
    size_t ExternId;
    double PositionX;
    double PositionY;
    double TotalWeight;
    double TotalVolume;
    double TotalArea;

    std::vector<Cuboid> Items;

    Group() = default;
    Group(size_t internId,
          size_t externId,
          double xPos,
          double yPos,
          double totalWeight,
          double totalVolume,
          double totalArea,
          std::vector<Cuboid> items)
    : InternId(internId),
      ExternId(externId),
      PositionX(xPos),
      PositionY(yPos),
      TotalWeight(totalWeight),
      TotalVolume(totalVolume),
      TotalArea(totalArea),
      Items(items)
    {
    }
};

}
}