#include "Model/Container.h"

#include <cassert>
#include <cmath>

namespace ContainerLoading
{
namespace Model
{
void ICoordinate::SetPosition(int x, int y, int z)
{
    X = x;
    Y = y;
    Z = z;
}

void IDimension::SetOrientation(int dx, int dy, int dz, Model::Rotation rotation)
{
    Dx = dx;
    Dy = dy;
    Dz = dz;

    Rotated = rotation;
}

std::tuple<int, int, int> Cuboid::DetermineDimensions(Orientation orientation) const
{
    switch (orientation)
    {
        case NoRotation:
            return std::make_tuple(Dx, Dy, Dz);
        case RotationZ:
            return std::make_tuple(Dy, Dx, Dz);
        default:
            throw std::runtime_error("Orientation not implemented.");
    }
}

int Cuboid::MinimumRotatableDimension(Axis axis) const
{
    switch (axis)
    {
        case Axis::X:
            return EnableHorizontalRotation ? std::min(Dx, Dy) : Dx;
        case Axis::Y:
            return EnableHorizontalRotation ? std::min(Dx, Dy) : Dy;
        case Axis::Z:
            return Dz;
        default:
            throw std::runtime_error("Invalid axis.");
    }
}

int Container::Dimension(Axis axis) const
{
    switch (axis)
    {
        case Axis::X:
            return Dx;
        case Axis::Y:
            return Dy;
        case Axis::Z:
            return Dz;
        default:
            throw std::runtime_error("Invalid axis.");
    }
}

bool ICuboid::Contains(const ICoordinate& other) const
{
    return other.X >= X && other.X < X + Dx && other.Y >= Y && other.Y < Y + Dy && other.Z >= Z && other.Z < Z + Dz;
}

}
}