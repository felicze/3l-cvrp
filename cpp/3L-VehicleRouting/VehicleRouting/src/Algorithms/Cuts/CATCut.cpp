#include "Algorithms/Cuts/CATCut.h"

#include <boost/graph/connected_components.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>

namespace VehicleRouting::Algorithms::Cuts
{
std::vector<Cut> CatCut::FindCuts(const std::vector<std::vector<double>>& x)
{
    std::vector<Cut> cuts;

    GraphFunctions::Graph graph;

    std::map<size_t, std::pair<size_t, size_t>> map = CreateEdgeMap(x, graph);

    // Return cuts in case of an empty graph
    if (CreateIncompatibleGraph(x, graph, map))
    {
        return cuts;
    }

    // edge-weighted graph is constructed
    // - graph : the actual edge-weighted graph
    // - valEdge: the values of the edges in graph
    // For every edge, compute minimum weight cycle in edge-weighted graph:
    // create bipartite graph from 'graph' to find cycles of odd length
    GraphFunctions::Graph bipartiteGraph;

    size_t numberOfNodes = CreateBipartiteGraph(bipartiteGraph, graph);

    Collections::IdVector components = GetConnectedComponents(bipartiteGraph);

    boost::graph_traits<GraphFunctions::Graph>::vertex_iterator uIt;
    boost::graph_traits<GraphFunctions::Graph>::vertex_iterator uEnd;
    boost::graph_traits<GraphFunctions::Graph>::vertex_iterator vIt;
    boost::graph_traits<GraphFunctions::Graph>::vertex_iterator vEnd;

    for (boost::tie(uIt, uEnd) = boost::vertices(bipartiteGraph); uIt != uEnd; ++uIt)
    {
        GraphFunctions::VertexDescriptor_t u = *uIt;
        GraphFunctions::VertexDescriptor_t uBipartite = u + numberOfNodes;

        if (bipartiteGraph[u].Bipartite == 1)
        {
            continue;
        }

        for (boost::tie(vIt, vEnd) = boost::vertices(bipartiteGraph); vIt != vEnd; ++vIt)
        {
            GraphFunctions::VertexDescriptor_t v = *vIt;
            GraphFunctions::VertexDescriptor_t vBipartite = v + numberOfNodes;
            if (bipartiteGraph[v].Bipartite == 1)
            {
                continue;
            }

            if (bipartiteGraph[u].OriginIndex >= bipartiteGraph[v].OriginIndex)
            {
                continue;
            }

            if (components[bipartiteGraph[u].Index] != components[bipartiteGraph[v].Index])
            {
                continue;
            }

            if (!IsIncidentEdge(u, vBipartite, bipartiteGraph))
            {
                continue;
            }

            // https://stackoverflow.com/questions/31145082/bgl-dijkstra-shortest-paths-with-bundled-properties
            std::vector<double> distances(boost::num_vertices(bipartiteGraph));
            std::vector<GraphFunctions::VertexDescriptor_t> predecessors(boost::num_vertices(bipartiteGraph));

            boost::dijkstra_shortest_paths(
                bipartiteGraph,
                u,
                boost::distance_map(boost::make_iterator_property_map(distances.begin(),
                                                                      boost::get(boost::vertex_index, bipartiteGraph)))
                    .predecessor_map(boost::make_iterator_property_map(predecessors.begin(),
                                                                       boost::get(boost::vertex_index, bipartiteGraph)))
                    .weight_map(boost::get(&GraphFunctions::EdgeProperties::Weight, bipartiteGraph)));

            double distance = distances[v];

            GraphFunctions::Graph::out_edge_iterator edgeIt;
            GraphFunctions::Graph::out_edge_iterator edgeEnd;
            for (std::tie(edgeIt, edgeEnd) = boost::out_edges(u, bipartiteGraph); edgeIt != edgeEnd; ++edgeIt)
            {
                if (GraphFunctions::VertexDescriptor_t t = boost::target(*edgeIt, bipartiteGraph); vBipartite == t)
                {
                    distance += bipartiteGraph[*edgeIt].Weight;
                    break;
                }
            }

            if (double lb = (1.0 - distance) / 2; lb < -0.5)
            {
                continue;
            }

            Collections::IdVector cycle;
            cycle.push_back(v);

            GraphFunctions::VertexDescriptor_t previousVertex = predecessors[v];
            cycle.push_back(bipartiteGraph[previousVertex].OriginIndex);

            while (true)
            {
                previousVertex = predecessors[previousVertex];
                cycle.push_back(bipartiteGraph[previousVertex].OriginIndex);

                if (previousVertex == u)
                {
                    break;
                }
            }

            cycle.push_back(v);

            // Check if cycle is chordless and simple
            if (!GraphFunctions::IsSimple(cycle))
            {
                continue;
            }

            // Check if chordless. Loop over all incident edges of all nodes in cycle, and check whether there are edges
            // to nodes in the cycle not belonging to the cycle

            bool chordless = true;

            // Cycle with 3 nodes is always chordless
            if (cycle.size() > 4)
            {
                for (auto el: cycle)
                {
                    for (std::tie(edgeIt, edgeEnd) =
                             boost::out_edges(boost::vertex(el, bipartiteGraph), bipartiteGraph);
                         edgeIt != edgeEnd;
                         ++edgeIt)
                    {
                        GraphFunctions::VertexDescriptor_t el2 = boost::target(*edgeIt, bipartiteGraph);

                        if (std::ranges::find(std::as_const(cycle), el2) == std::end(cycle))
                        {
                            continue;
                        }

                        chordless = false;
                        for (size_t i = 0; i != cycle.size() - 1; ++i)
                        {
                            if ((el == cycle[i] && el2 == cycle[i + 1]) || (el2 == cycle[i] && el == cycle[i + 1]))
                            {
                                chordless = true;
                                break;
                            }
                        }
                    }
                }
            }

            if (!chordless)
            {
                continue;
            }

            // Check real cut value:
            auto cut = Cut(CutType::CAT);
            Collections::IdVector source(numberOfNodes, 0);
            Collections::IdVector sink(numberOfNodes, 0);

            for (size_t i = 0; i != cycle.size() - 1; ++i)
            {
                const auto node = cycle[i];

                const auto& [edgeTail, edgeHead] = map[node];

                cut.AddArc(-1.0, edgeTail, edgeHead, x[edgeTail][edgeHead]);

                source[edgeTail]++;
                sink[edgeHead]++;
            }

            // Add edges from source to sink

            for (size_t idx = 0; idx != source.size(); ++idx)
            {
                if (source[idx] != 2)
                {
                    continue;
                }

                for (size_t jdx = 0; jdx != sink.size(); ++jdx)
                {
                    if (sink[jdx] != 2)
                    {
                        continue;
                    }

                    if (idx == jdx)
                    {
                        continue;
                    }

                    bool adj = false;

                    for (size_t i = 0, len = cycle.size() - 1; i < len; ++i)
                    {
                        if (map[cycle[i]].first == idx and map[cycle[i]].second == jdx)
                        {
                            adj = true;
                            break;
                        }
                    }

                    if (!adj)
                    {
                        cut.AddArc(-1.0, idx, jdx, x[idx][jdx]);
                    }
                }
            }

            cut.RHS = -(static_cast<double>(cycle.size()) - 2.0) / 2.0;
            cut.CalcViolation();

            if (cut.Violation <= InputParameters->UserCut.ViolationThreshold.at(Type))
            {
                continue;
            }

            cuts.emplace_back(std::move(cut));

            RemoveEdges(u, vBipartite, bipartiteGraph);
            RemoveEdges(uBipartite, v, bipartiteGraph);
            components = GetConnectedComponents(bipartiteGraph);
        }
    }

    return cuts;
}
}
