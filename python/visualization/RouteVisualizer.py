import streamlit as st
import networkx as nx
import numpy as np

import pydeck as pdk
import plotly.graph_objects as go

from ColorConversion import hex_color_to_shaded_list


def make_routing_figure(filtered_tours):
    return visualize_tours_networkx(filtered_tours)


def visualize_tours_pdk(tours):
    first_tour = tours.head(1)
    view_state = pdk.ViewState(
        latitude=first_tour["coordinates"][0][0][1],  # depotLat
        longitude=first_tour["coordinates"][0][0][0],  # depotLon
        zoom=9,
    )

    layer = pdk.Layer(
        type="PathLayer",  # https://pydeck.gl/gallery/trips_layer.html
        data=tours,
        pickable=True,
        get_color="color",
        width_scale=20,
        width_min_pixels=2,
        get_path="coordinates",
        get_width=5,
    )

    pydeck = pdk.Deck(
        map_style="mapbox://styles/mapbox/light-v9",
        layers=layer,
        initial_view_state=view_state,
        tooltip={"html": "Tour ID: {tourId} <br/> sites: {sites}"},
    )
    # pydeck does not support multiple tooltips https://discuss.streamlit.io/t/how-can-i-set-different-tooltips-for-multiple-layers-on-a-pydeck-streamlit/2588
    # for multiple tooltips use Altair instead https://discuss.streamlit.io/t/how-do-use-location-pin-symbol-in-map-and-how-to-connect-those-location-in-streamlit/2308/2
    # explains the notation latitude:Q https://altair-viz.github.io/user_guide/encoding.html#encoding-data-types
    # example from altair https://altair-viz.github.io/gallery/airports_count.html

    st.pydeck_chart(pydeck)


def visualize_tours_networkx(tours):
    graph = nx.DiGraph()
    edge_labels = {}
    first_tour = tours.head(1).iloc[0]
    depot_coords = [first_tour["coordinates"][0][0], first_tour["coordinates"][0][1]]

    graph.add_node(0, pos=(depot_coords[0], depot_coords[1]), color="white", position=0, routeId=-1)

    for tour in tours.itertuples(index=False):
        coordinates = getattr(tour, "coordinates")
        sites = getattr(tour, "sites")
        base_color = getattr(tour, "color")
        tour_id = getattr(tour, "tourId")

        colors = hex_color_to_shaded_list(base_color, len(sites))
        for idx, site_id in enumerate(sites):
            coords = coordinates[idx + 1]
            graph.add_node(site_id, pos=(coords[0], coords[1]), color=colors[idx], position=(idx + 1), routeId=tour_id)

        graph.add_edge(0, sites[0], color=base_color, tourId=tour_id)
        edge_labels[(0, sites[0])] = "route " + str(tour_id)
        for idx, site_id in enumerate(sites[:-1]):
            next_site_id = sites[idx + 1]
            graph.add_edge(site_id, next_site_id, color=base_color, arrowsize=20, arrowstyle="fancy")

        graph.add_edge(sites[-1], 0)

    edge_x = []
    edge_y = []
    line_width = []
    for edge in graph.edges():
        node_i = edge[0]
        node_j = edge[1]
        x0, y0 = graph.nodes[node_i]["pos"]
        x1, y1 = graph.nodes[node_j]["pos"]
        edge_x.append(x0)
        edge_x.append(x1)
        edge_x.append(None)
        edge_y.append(y0)
        edge_y.append(y1)
        edge_y.append(None)
        if node_i == 0:
            line_width.append(1.5)
        else:
            line_width.append(0.5)

    edge_trace = go.Scatter(x=edge_x, y=edge_y, line=dict(width=0.5, color="#888"), hoverinfo="none", mode="lines")

    node_x = []
    node_y = []
    colors = []
    node_ids = []
    positions = []
    route_ids = []
    for node in graph.nodes():
        x, y = graph.nodes[node]["pos"]
        node_x.append(x)
        node_y.append(y)
        node_ids.append(node)
        colors.append(graph.nodes[node]["color"])
        positions.append(graph.nodes[node]["position"])
        route_ids.append(graph.nodes[node]["routeId"])

    node_trace = go.Scatter(
        x=node_x,
        y=node_y,
        mode="markers+text",
        customdata=np.stack((positions, route_ids), axis=-1),
        hovertemplate="<b>Node: %{text}</b><br>"
        + "Route: %{customdata[1]}<br>"
        + "Coords: (%{x},%{y})<br>"
        + "Position: %{customdata[0]}",
        text=node_ids,
        marker=dict(size=20, color=colors),
    )

    fig = go.Figure(
        data=[edge_trace, node_trace],
        layout=go.Layout(
            showlegend=False,
            hovermode="closest",
            margin=dict(b=20, l=5, r=5, t=40),
            xaxis=dict(showgrid=False, zeroline=False, showticklabels=False),
            yaxis=dict(showgrid=False, zeroline=False, showticklabels=False),
        ),
    )

    fig.update_traces(textposition="middle center")

    return fig
