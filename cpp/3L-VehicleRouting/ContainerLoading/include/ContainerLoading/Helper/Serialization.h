#pragma once

#include "nlohmann/json.hpp"
using json = nlohmann::json;

#include "Model/Container.h"

namespace ContainerLoading
{
namespace Model
{

enum class Rotation;
enum class Fragility;

NLOHMANN_JSON_SERIALIZE_ENUM(Rotation, {{Rotation::None, "None"}, {Rotation::Yaw, "Yaw"}});

NLOHMANN_JSON_SERIALIZE_ENUM(Fragility, {{Fragility::None, "None"}, {Fragility::Fragile, "Fragile"}});

class Cuboid;
class Container;

void from_json(const json& j, Cuboid& item);
void to_json(json& j, const Cuboid& item);

void from_json(const json& j, Container& container);
void to_json(json& j, const Container& container);
}
}