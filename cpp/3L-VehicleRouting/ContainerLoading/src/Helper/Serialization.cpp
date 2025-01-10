#include "Helper/Serialization.h"

namespace ContainerLoading
{
namespace Model
{

void from_json(const json& j, Cuboid& item)
{
    j.at("X").get_to(item.X);
    j.at("Y").get_to(item.Y);
    j.at("Z").get_to(item.Z);
    j.at("Dx").get_to(item.Dx);
    j.at("Dy").get_to(item.Dy);
    j.at("Dz").get_to(item.Dz);
    j.at("Weight").get_to(item.Weight);
    j.at("EnableHorizontalRotation").get_to(item.EnableHorizontalRotation);
    j.at("Fragility").get_to(item.Fragility);
    j.at("Rotated").get_to(item.Rotated);
}

void to_json(json& j, const Cuboid& item)
{
    j = json{
        {"X", item.X},
        {"Y", item.Y},
        {"Z", item.Z},
        {"Dx", item.Dx},
        {"Dy", item.Dy},
        {"Dz", item.Dz},
        {"Weight", item.Weight},
        {"EnableHorizontalRotation", item.EnableHorizontalRotation},
        {"Fragility", item.Fragility},
        {"Rotated", item.Rotated},
    };
}

void from_json(const json& j, Container& container)
{
    j.at("Dx").get_to(container.Dx);
    j.at("Dy").get_to(container.Dy);
    j.at("Dz").get_to(container.Dz);
    j.at("WeightLimit").get_to(container.WeightLimit);
}

void to_json(json& j, const Container& container)
{
    j = json{
        {"Dx", container.Dx},
        {"Dy", container.Dy},
        {"Dz", container.Dz},
        {"WeightLimit", container.WeightLimit},
    };
}

}
}