#pragma once

namespace ContainerLoading
{
namespace Algorithms
{
enum class LoadingStatus
{
    Invalid = 0,
    FeasOpt,
    Infeasible,
    Unknown
};

enum class PackingType
{
    None = 0, // For safety.
    Complete, // all selected loading constraints, i.e. all or subset of no overlap, fragility, support, lifo
    CompleteNoSequence, // all selected loading constraints, i.e. all or subset of no overlap, fragility, support, lifo
                        // without given sequence
    NoLifo, // all selected loading constraints, but LIFO definitely disabled
    NoSupport, // all selected loading constraints, but support definitely disabled
    NoFragility, // all selected loading constraints, but fragility definitely disabled
    NoSupportNoSequence, // all selected loading constraints, lifo without given sequence, but support definitely
                         // disabled
    LifoNoSequence, // Only lifo without given sequence, but support and fragility definitely disabled
    NoSupportNoLifo, // all selected loading constraints, but support and lifo definitely disabled
    NoSupportNoFragility, // all selected loading constraints, but support and fragility definitely disable
    LoadingOnly // only non overlapping of items
};

enum class LoadingFlag
{
    // Basic flags
    NoneSet = 0,
    NoOverlap = 1,
    Fragility = 1 << 1,
    Support = 1 << 2,
    Lifo = 1 << 3,
    Sequence = 1 << 4,
    // Concatenated flags
    Complete = NoOverlap | Fragility | Support | Lifo | Sequence,
    NoFragility = NoOverlap | Support | Lifo | Sequence,
    NoLifo = NoOverlap | Fragility | Support,
    NoSupport = NoOverlap | Fragility | Lifo | Sequence,
    LifoNoSequence = NoOverlap | Lifo,
    LifoSequence = NoOverlap | Lifo | Sequence,
    FragilityOnly = NoOverlap | Fragility,
    LoadingOnly = 1
};

constexpr inline LoadingFlag operator|(LoadingFlag a, LoadingFlag b)
{
    return static_cast<LoadingFlag>(static_cast<int>(a) | static_cast<int>(b));
}

constexpr inline LoadingFlag operator&(LoadingFlag a, LoadingFlag b)
{
    return static_cast<LoadingFlag>(static_cast<int>(a) & static_cast<int>(b));
}

/// The second argument (flag) must only be one of the basic flags in LoadingFlag.
constexpr inline bool IsSet(const LoadingFlag& mask, const LoadingFlag& flag)
{
    return (mask & flag) != LoadingFlag::NoneSet;
}

}
}