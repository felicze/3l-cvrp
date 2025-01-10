#pragma once

#include "CommonBasics/Helper/ModelServices.h"

#include <map>
#include <utility>
#include <vector>

#include <boost/graph/adjacency_list.hpp>

namespace VehicleRouting::Algorithms::Cuts
{
class CVRPSEPGraph
{
  public:
    std::vector<int> EdgeHead; // must be int for CVRPSEP
    std::vector<int> EdgeTail; // must be int for CVRPSEP
    std::vector<double> RelaxedValueEdge;
    size_t CounterEdges = 0;

    CVRPSEPGraph() = default;

    explicit CVRPSEPGraph(size_t numberNodes)
    {
        EdgeHead = std::vector<int>(numberNodes * numberNodes, 0);
        EdgeTail = std::vector<int>(numberNodes * numberNodes, 0);
        RelaxedValueEdge = std::vector<double>(numberNodes * numberNodes, 0.0);
    };

    void Build(const std::vector<std::vector<double>>& valueX, double epsForIntegrality);
};

namespace GraphFunctions
{
// An unnamed namespace is only accessible from within the current namespace
// You can compare it with private data members of a class
//
// The attributes are used! But in external files.
// We indicate that they are unused to avoid several hundreds of warnings..

extern size_t numberOfDepots;
extern size_t numberOfCustomers;
extern size_t numberOfNodes;
// Validate x Values
inline double epsilonValue;
// add small positive value to edge weight to generate short cycles with dijkstra
inline double edgeUsagePenalty = 0.001;

struct VertexProperties
{
    int Index = -1;
    int OriginIndex = -1;
    int Bipartite = -1; // 0 and 1 for part of bipartite graph
};

struct EdgeProperties
{
    double Weight = 0.0;
};

using VertexDescriptor_t =
    boost::adjacency_list_traits<boost::vecS, boost::vecS, boost::undirectedS>::vertex_descriptor;
using Graph = boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS, VertexProperties, EdgeProperties>;
using Digraph = boost::adjacency_list<boost::vecS, boost::vecS, boost::directedS>;

void SetValues(size_t nDepots, size_t nCustomers, size_t nNodes, double epsIntegrality);

void CreateListDigraph(Digraph& graph, const std::vector<std::vector<double>>& x);

void RemoveEdges(const VertexDescriptor_t& u, const VertexDescriptor_t& v, Graph& graph);

bool IsSimple(Collections::IdVector cycle);

Collections::IdVector GetConnectedComponents(Graph& graph);

std::map<size_t, std::pair<size_t, size_t>> CreateEdgeMap(const std::vector<std::vector<double>>& x, Graph& graph);

size_t CreateBipartiteGraph(Graph& bipartiteGraph, Graph& graph);

bool CreateIncompatibleGraph(const std::vector<std::vector<double>>& x,
                             Graph& graph,
                             std::map<size_t, std::pair<size_t, size_t>>& map_in);

bool IsIncidentEdge(const VertexDescriptor_t& u, const VertexDescriptor_t& v, Graph& bipartiteGraph);

bool IsIncompatible(const std::pair<size_t, size_t>& infoV, const std::pair<size_t, size_t>& infoU);

// The following functions return a vector containing
// weakly connected components. You can call the functions
// with a ListGraph, a ListDiGraph, or a with given LP relaxation

std::vector<Collections::IdVector> GetConnectedComponents(const Digraph& g);
std::vector<Collections::IdVector> GetConnectedComponents(const std::vector<std::vector<double>>& x);
}

}
