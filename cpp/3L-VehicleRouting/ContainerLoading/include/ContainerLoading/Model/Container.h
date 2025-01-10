#pragma once

#include <cstddef>
#include <limits>
#include <stdexcept>
#include <tuple>

namespace ContainerLoading
{
namespace Model
{
enum DimensionType
{
    AxisX = 0,
    AxisY,
    AxisZ
};

enum RelativeDirection
{
    Right = 0,
    Left,
    InFront,
    Behind,
    Above,
    Below
};

enum Orientation
{
    NoRotation = 0,
    RotationZ,
    RotationY,
    RotationYZ, // (xy)
    RotationXZ, // (zy)
    RotationX
};

struct Dimension
{
    DimensionType Type;
    RelativeDirection FirstDirection;
    RelativeDirection SecondDirection;
};

enum class Axis
{
    X,
    Y,
    Z
};

/// Rotations by 90 degrees.
enum class Rotation
{
    None = 0,

    /// Rotate horizontally around z-axis
    Yaw
};

struct ICoordinate
{
  public:
    int X = 0;
    int Y = 0;
    int Z = 0;

    void SetPosition(int x, int y, int z);

  protected:
    ICoordinate() = default;
    ICoordinate(int x, int y, int z) : X(x), Y(y), Z(z) {}
};

struct IDimension
{
  public:
    int Dx = 0;
    int Dy = 0;
    int Dz = 0;

    double Area = 0.0;
    double Volume = 0.0;

    Model::Rotation Rotated = Model::Rotation::None;

    void SetOrientation(int dx, int dy, int dz, Model::Rotation rotation = Model::Rotation::None);

  protected:
    IDimension() = default;

    // Caution, overflowing ony 32 bit systems because frequently volume > int::max()
    IDimension(int dx, int dy, int dz)
    : Dx(dx), Dy(dy), Dz(dz), Area((double)dx * (double)dy), Volume((double)dx * (double)dy * (double)dz)
    {
    }
    IDimension(int dx, int dy, int dz, Model::Rotation rotation)
    : Dx(dx),
      Dy(dy),
      Dz(dz),
      Area((double)dx * (double)dy),
      Volume((double)dx * (double)dy * (double)dz),
      Rotated(rotation)
    {
    }
};

struct ICuboid : public ICoordinate, public IDimension
{
  public:
    [[nodiscard]] bool Contains(const ICoordinate& other) const;

    [[nodiscard]] virtual int MinimumRotatableDimension(Axis axis) const = 0;

  protected:
    ICuboid() = default;
    ICuboid(int x, int y, int z, int dx, int dy, int dz) : ICoordinate(x, y, z), IDimension(dx, dy, dz) {}
    ICuboid(int x, int y, int z, int dx, int dy, int dz, Model::Rotation rotation)
    : ICoordinate(x, y, z), IDimension(dx, dy, dz, rotation)
    {
    }
};

enum class Fragility
{
    None = 0,

    /// Fragile items can be stacked onto other fragile or non-fragile item. But non-fragile items must not touch
    /// fragile items from above.
    Fragile,
};

class Cuboid : public ICuboid
{
  public:
    /// Temporary
    size_t InternId = 0;
    size_t ExternId = 0;
    size_t GroupId = 0;

    double Weight = 0.0;

    bool EnableHorizontalRotation = true;
    bool RequireFloorPlacement = false;

    Model::Fragility Fragility = Model::Fragility::None;

    Cuboid() = default;
    Cuboid(const Cuboid& cuboid) = default;

    Cuboid(int x, int y, int z, int dx, int dy, int dz) : ICuboid(x, y, z, dx, dy, dz) {}

    Cuboid(int x,
           int y,
           int z,
           int dx,
           int dy,
           int dz,
           Model::Rotation rotation,
           Model::Fragility fragility = Fragility::None)
    : ICuboid(x, y, z, dx, dy, dz, rotation), Fragility(fragility)
    {
    }

    Cuboid(int dx, int dy, int dz) : ICuboid(0, 0, 0, dx, dy, dz) {}
    Cuboid(size_t internId,
           size_t externId,
           int dx,
           int dy,
           int dz,
           bool enableHorizontalRotation,
           Model::Fragility fragility,
           size_t groupId,
           double weight)
    : ICuboid(0, 0, 0, dx, dy, dz),
      InternId(internId),
      ExternId(externId),
      GroupId(groupId),
      Weight(weight),
      EnableHorizontalRotation(enableHorizontalRotation),
      Fragility(fragility)
    {
    }

    virtual ~Cuboid() = default;

    [[nodiscard]] std::tuple<int, int, int> DetermineDimensions(Orientation orientation) const;

    [[nodiscard]] int MinimumRotatableDimension(Axis axis) const override;
};

class Container : public ICuboid
{
  public:
    double WeightLimit = std::numeric_limits<double>::max();

    Container() = default;
    Container(const Container&) = default;
    Container(int x, int y, int z, int dx, int dy, int dz, double weightLimit)
    : ICuboid(x, y, z, dx, dy, dz), WeightLimit(weightLimit)
    {
    }
    Container(int dx, int dy, int dz, double weightLimit) : ICuboid(0, 0, 0, dx, dy, dz), WeightLimit(weightLimit) {}

    virtual ~Container() = default;

    [[nodiscard]] int MinimumRotatableDimension(Axis axis [[maybe_unused]]) const override
    {
        throw std::runtime_error("Rotations not implemented for containers.");
    };
    [[nodiscard]] int Dimension(Axis axis) const;
};

/// https://stackoverflow.com/a/9729747/5587903
struct HomogeneityHash
{
    std::size_t operator()(const Cuboid& cuboid) const
    {
        int dx = cuboid.Rotated == Rotation::Yaw ? cuboid.Dy : cuboid.Dx;
        int dy = cuboid.Rotated == Rotation::Yaw ? cuboid.Dx : cuboid.Dy;

        // Compute individual hash values for first, second and third
        // http://stackoverflow.com/a/1646913/126995
        size_t hash = 17;

        hash = hash * 31 + std::hash<int>()(dx);
        hash = hash * 31 + std::hash<int>()(dy);
        hash = hash * 31 + std::hash<int>()(cuboid.Dz);
        hash = hash * 31 + std::hash<size_t>()(cuboid.GroupId);
        hash = hash * 31 + std::hash<int>()((int)cuboid.Weight);
        hash = hash * 31 + std::hash<Fragility>()(cuboid.Fragility);
        hash = hash * 31 + std::hash<bool>()(cuboid.EnableHorizontalRotation);
        hash = hash * 31 + std::hash<bool>()(cuboid.RequireFloorPlacement);

        return hash;
    }

    bool operator()(const Cuboid& lhs, const Cuboid& rhs) const
    {
        int lhsDx = lhs.Rotated == Rotation::Yaw ? lhs.Dy : lhs.Dx;
        int lhsDy = lhs.Rotated == Rotation::Yaw ? lhs.Dx : lhs.Dy;

        int rhsDx = rhs.Rotated == Rotation::Yaw ? rhs.Dy : rhs.Dx;
        int rhsDy = rhs.Rotated == Rotation::Yaw ? rhs.Dx : rhs.Dy;

        // clang-format off
        return lhsDx == rhsDx 
                && lhsDy == rhsDy 
                && lhs.Dz == rhs.Dz 
                && lhs.GroupId == rhs.GroupId
                && (int)lhs.Weight == (int)rhs.Weight 
                && lhs.Fragility == rhs.Fragility
                && lhs.EnableHorizontalRotation == rhs.EnableHorizontalRotation
                && lhs.RequireFloorPlacement == rhs.RequireFloorPlacement;
        // clang-format on
    }
};

}
}