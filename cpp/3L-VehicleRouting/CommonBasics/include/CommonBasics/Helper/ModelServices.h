#pragma once

#include <boost/functional/hash.hpp>
#include <unordered_set>
#include <vector>

namespace Collections
{
template <typename TContainer> struct container_hash
{
    std::size_t operator()(TContainer const& c) const { return boost::hash_range(c.begin(), c.end()); }
};

using IdVector = std::vector<size_t>;
using SequenceSet = std::unordered_set<IdVector, container_hash<IdVector>>;
using SequenceVector = std::vector<IdVector>;

}
