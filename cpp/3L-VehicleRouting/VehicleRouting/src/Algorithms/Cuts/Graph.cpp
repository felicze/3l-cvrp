#include "Algorithms/Cuts/Graph.h"

#include <boost/graph/connected_components.hpp>
#include <cstddef>

namespace VehicleRouting::Algorithms::Cuts
{
namespace GraphFunctions
{
size_t numberOfDepots = 0;
size_t numberOfCustomers = 0;
size_t numberOfNodes = 0;

}

void GraphFunctions::SetValues(size_t nDepots, size_t nCustomers, size_t nNodes, double epsIntegrality)
{
    numberOfDepots = nDepots;
    numberOfCustomers = nCustomers;
    numberOfNodes = nNodes;
    epsilonValue = epsIntegrality;
}

void GraphFunctions::CreateListDigraph(Digraph& graph, const std::vector<std::vector<double>>& x)
{
    for (size_t i = 0, last = numberOfNodes; i < last; ++i)
    {
        boost::add_vertex(graph);
    }

    const boost::property_map<Digraph, boost::vertex_index_t>::type indexMap = boost::get(boost::vertex_index, graph);
    const std::pair<Digraph::vertex_iterator, Digraph::vertex_iterator> vertexIteratorRange = boost::vertices(graph);

    for (Digraph::vertex_iterator sourceVertexIterator = vertexIteratorRange.first;
         sourceVertexIterator != vertexIteratorRange.second;
         ++sourceVertexIterator)
    {
        const size_t i = boost::get(indexMap, *sourceVertexIterator);

        if (i < numberOfDepots)
        {
            continue;
        }

        for (Digraph::vertex_iterator targetVertexIterator = sourceVertexIterator + 1;
             targetVertexIterator != vertexIteratorRange.second;
             ++targetVertexIterator)
        {
            const size_t j = boost::get(indexMap, *targetVertexIterator);
            if (j < numberOfDepots)
            {
                continue;
            }

            if (j < i)
            {
                continue;
            }

            const double edgeIsUsedThreshold = 0.0001;
            if (x[i][j] + x[j][i] > edgeIsUsedThreshold)
            {
                const Digraph::vertex_descriptor sourceVertex = boost::vertex(i, graph);
                const Digraph::vertex_descriptor targetVertex = boost::vertex(j, graph);

                boost::add_edge(sourceVertex, targetVertex, graph);
                boost::add_edge(targetVertex, sourceVertex, graph);
            }
        }
    }
}

void GraphFunctions::RemoveEdges(const VertexDescriptor_t& u, const VertexDescriptor_t& v, Graph& graph)
{
    Graph::out_edge_iterator edgeIt;
    Graph::out_edge_iterator edgeEnd;
    for (std::tie(edgeIt, edgeEnd) = boost::out_edges(u, graph); edgeIt != edgeEnd; ++edgeIt)
    {
        if (v == boost::target(*edgeIt, graph) || v == boost::source(*edgeIt, graph))
        {
            boost::remove_edge(*edgeIt, graph);
        }
    }
}

bool GraphFunctions::IsSimple(Collections::IdVector cycle)
{
    std::sort(std::begin(cycle), std::end(cycle) - 1);

    return std::ranges::adjacent_find(std::as_const(cycle)) == std::cend(cycle);
}

Collections::IdVector GraphFunctions::GetConnectedComponents(Graph& graph)
{
    auto convertedGraph = Digraph(boost::num_vertices(graph));

    boost::graph_traits<Graph>::edge_iterator edgeIt;
    boost::graph_traits<Graph>::edge_iterator edgeEnd;
    for (boost::tie(edgeIt, edgeEnd) = boost::edges(graph); edgeIt != edgeEnd; ++edgeIt)
    {
        const auto uIdxGraph = graph[boost::source(*edgeIt, graph)].Index;
        const auto vIdxGraph = graph[boost::target(*edgeIt, graph)].Index;

        const Digraph::vertex_descriptor sourceVertex = boost::vertex(static_cast<size_t>(uIdxGraph), convertedGraph);
        const Digraph::vertex_descriptor targetVertex = boost::vertex(static_cast<size_t>(vIdxGraph), convertedGraph);

        boost::add_edge(sourceVertex, targetVertex, convertedGraph);
        boost::add_edge(targetVertex, sourceVertex, convertedGraph);
    }

    Collections::IdVector components(boost::num_vertices(convertedGraph));
    boost::connected_components(convertedGraph, components.data());
    return components;
}

std::map<size_t, std::pair<size_t, size_t>> GraphFunctions::CreateEdgeMap(const std::vector<std::vector<double>>& x,
                                                                          Graph& graph)
{
    std::map<size_t, std::pair<size_t, size_t>> map; // maps node to arc (int, int)

    size_t ctr = 0;
    for (size_t i = 0; i != numberOfNodes; ++i)
    {
        for (size_t j = 0; j != numberOfNodes; ++j)
        {
            if (x[i][j] > epsilonValue)
            {
                boost::add_vertex(
                    VertexProperties{.Index = static_cast<int>(ctr), .OriginIndex = static_cast<int>(ctr)}, graph);
                map[ctr++] = std::make_pair(i, j);
            }
        }
    }
    return map;
}

size_t GraphFunctions::CreateBipartiteGraph(Graph& bipartiteGraph, Graph& graph)
{
    int nNodes = 0;
    int nBipartiteNodes = 0;
    boost::graph_traits<Graph>::vertex_iterator i;
    boost::graph_traits<Graph>::vertex_iterator iEnd;
    boost::graph_traits<Graph>::vertex_iterator j;
    boost::graph_traits<Graph>::vertex_iterator jEnd;
    Graph::out_edge_iterator edgeIt;
    Graph::out_edge_iterator edgeEnd;

    for (boost::tie(i, iEnd) = boost::vertices(graph); i != iEnd; ++i)
    {
        boost::add_vertex(VertexProperties{.Index = nBipartiteNodes, .OriginIndex = nNodes, .Bipartite = 0},
                          bipartiteGraph);
        ++nNodes;
        ++nBipartiteNodes;
    }

    for (boost::tie(i, iEnd) = boost::vertices(graph); i != iEnd; ++i)
    {
        boost::add_vertex(
            VertexProperties{.Index = nBipartiteNodes, .OriginIndex = nBipartiteNodes - nNodes, .Bipartite = 1},
            bipartiteGraph);
        ++nBipartiteNodes;
    }

    for (boost::tie(i, iEnd) = boost::vertices(bipartiteGraph); i != iEnd; ++i)
    {
        const auto fromIdx = bipartiteGraph[*i].OriginIndex;

        for (boost::tie(j, jEnd) = boost::vertices(bipartiteGraph); j != jEnd; ++j)
        {
            const auto toIdx = bipartiteGraph[*j].OriginIndex;

            if (toIdx <= fromIdx)
            {
                continue;
            }

            if (bipartiteGraph[*i].Bipartite == bipartiteGraph[*j].Bipartite)
            {
                continue;
            }

            for (std::tie(edgeIt, edgeEnd) = boost::out_edges(static_cast<size_t>(fromIdx), graph); edgeIt != edgeEnd;
                 ++edgeIt)
            {
                if (const auto edgeHeadIdx = graph[boost::target(*edgeIt, graph)].Index; toIdx == edgeHeadIdx)
                {
                    const Graph::vertex_descriptor sourceVertex =
                        boost::vertex(static_cast<size_t>(bipartiteGraph[*i].Index), bipartiteGraph);
                    const Graph::vertex_descriptor targetVertex =
                        boost::vertex(static_cast<size_t>(bipartiteGraph[*j].Index), bipartiteGraph);

                    boost::add_edge(sourceVertex, targetVertex, EdgeProperties{graph[*edgeIt].Weight}, bipartiteGraph);
                }
            }
        }
    }

    return static_cast<size_t>(nNodes);
}

bool GraphFunctions::CreateIncompatibleGraph(const std::vector<std::vector<double>>& x,
                                             Graph& graph,
                                             std::map<size_t, std::pair<size_t, size_t>>& map_in)
{
    bool emptyGraph = true;

    boost::graph_traits<Graph>::vertex_iterator i;
    boost::graph_traits<Graph>::vertex_iterator iEnd;
    boost::graph_traits<Graph>::vertex_iterator j;
    boost::graph_traits<Graph>::vertex_iterator jEnd;

    for (boost::tie(i, iEnd) = boost::vertices(graph); i != iEnd; ++i)
    {
        const auto u = static_cast<size_t>(graph[*i].OriginIndex);
        std::pair<size_t, size_t>& infoEdgeU = map_in[u];

        for (boost::tie(j, jEnd) = boost::vertices(graph); j != jEnd; ++j)
        {
            const auto v = static_cast<size_t>(graph[*j].OriginIndex);

            if (v <= u)
            {
                continue;
            }

            if (std::pair<size_t, size_t>& infoEdgeV = map_in[v];
                IsIncompatible(infoEdgeU, infoEdgeV) or IsIncompatible(infoEdgeV, infoEdgeU))
            {
                const Graph::vertex_descriptor sourceVertex = boost::vertex(u, graph);
                const Graph::vertex_descriptor targetVertex = boost::vertex(v, graph);

                const double value =
                    1 + edgeUsagePenalty - x[infoEdgeU.first][infoEdgeU.second] - x[infoEdgeV.first][infoEdgeV.second];

                boost::add_edge(sourceVertex, targetVertex, EdgeProperties{value}, graph);

                emptyGraph = false;
            }
        }
    }

    return emptyGraph;
}

bool GraphFunctions::IsIncidentEdge(const VertexDescriptor_t& u, const VertexDescriptor_t& v, Graph& bipartiteGraph)
{
    const auto vIdx = static_cast<size_t>(bipartiteGraph[v].OriginIndex);

    Graph::out_edge_iterator edgeIt;
    Graph::out_edge_iterator edgeEnd;
    for (std::tie(edgeIt, edgeEnd) = boost::out_edges(u, bipartiteGraph); edgeIt != edgeEnd; ++edgeIt)
    {
        if (const auto edgeHeadIdx =
                static_cast<size_t>(bipartiteGraph[boost::target(*edgeIt, bipartiteGraph)].OriginIndex);
            vIdx == edgeHeadIdx)
        {
            return true;
        }
    }

    return false;
}

bool GraphFunctions::IsIncompatible(const std::pair<size_t, size_t>& infoV, const std::pair<size_t, size_t>& infoU)
{
    // Here: Possible edge incompatibilities between u and v:
    //	v from customer to customer: a) u same source / target  b) 2-subtour
    //	v from depot to customer: u same target
    //	v from customer to depot: u same source

    // case (i):
    if (infoV.first >= numberOfDepots and infoV.second >= numberOfDepots)
    {
        if (infoV.first == infoU.first)
        {
            return true;
        }

        if (infoV.second == infoU.second)
        {
            return true;
        }

        if (infoV.second == infoU.first and infoV.first == infoU.second)
        {
            return true;
        }
    }

    // case (ii-a)
    else if (infoV.first < numberOfDepots and infoV.second >= numberOfDepots)
    {
        if (infoU.second == infoV.second)
        {
            return true;
        }
        if (infoU.first == infoV.second and infoU.second < numberOfDepots and infoU.second != infoV.first)
        {
            return true;
        }
    }

    // case (ii-b)
    else if (infoV.first >= numberOfDepots and infoV.second < numberOfDepots)
    {
        if (infoU.first == infoV.first)
        {
            return true;
        }

        if (infoU.second == infoV.first and infoU.first < numberOfDepots and infoU.first != infoV.second)
        {
            return true;
        }
    }

    return false;
}

std::vector<Collections::IdVector> GraphFunctions::GetConnectedComponents(const Digraph& g)
{
    Collections::IdVector components(boost::num_vertices(g));
    const size_t numCC = boost::connected_components(g, components.data());

    std::vector<Collections::IdVector> connectedComponents;
    const std::pair<Digraph::vertex_iterator, Digraph::vertex_iterator> vertexIteratorRange = boost::vertices(g);

    for (size_t idxCC = 0; idxCC < numCC; idxCC++)
    {
        Collections::IdVector connectedComponent;

        for (Digraph::vertex_iterator vertexIt = vertexIteratorRange.first; vertexIt != vertexIteratorRange.second;
             ++vertexIt)
        {
            const size_t u = *vertexIt;
            if (components[u] == idxCC)
            {
                connectedComponent.push_back(u);
            }
        }

        if (connectedComponent.empty())
        {
            break;
        }

        connectedComponents.emplace_back(std::move(connectedComponent));
    }

    return connectedComponents;
}

std::vector<Collections::IdVector> GraphFunctions::GetConnectedComponents(const std::vector<std::vector<double>>& x)
{
    Digraph graph;
    CreateListDigraph(graph, x);

    return GetConnectedComponents(graph);
}

void CVRPSEPGraph::Build(const std::vector<std::vector<double>>& valueX, const double epsForIntegrality)
{
    CounterEdges = 1;
    const size_t nNodes = valueX.size();
    const size_t nCustomerNodes = nNodes - 1;

    for (size_t iNode = 0; iNode < nNodes - 1; ++iNode)
    {
        const size_t nodeI = iNode == 0 ? nCustomerNodes + 1 : iNode;
        for (size_t jNode = iNode + 1; jNode < nNodes; ++jNode)
        {
            const double value = valueX[iNode][jNode] + valueX[jNode][iNode];
            const size_t nodeJ = jNode == 0 ? nCustomerNodes + 1 : jNode;
            if (value > epsForIntegrality)
            {
                EdgeHead[CounterEdges] = static_cast<int>(nodeI);
                EdgeTail[CounterEdges] = static_cast<int>(nodeJ);
                RelaxedValueEdge[CounterEdges] = value;
                CounterEdges++;
            }
        }
    }
}

}
