import numpy as np

import plotly.graph_objects as go
from mpl_toolkits.mplot3d.art3d import Poly3DCollection


def cuboid_data_2(o, size=(1, 1, 1)):
    x = [
        [[0, 1, 0], [0, 0, 0], [1, 0, 0], [1, 1, 0]],
        [[0, 0, 0], [0, 0, 1], [1, 0, 1], [1, 0, 0]],
        [[1, 0, 1], [1, 0, 0], [1, 1, 0], [1, 1, 1]],
        [[0, 0, 1], [0, 0, 0], [0, 1, 0], [0, 1, 1]],
        [[0, 1, 0], [0, 1, 1], [1, 1, 1], [1, 1, 0]],
        [[0, 1, 1], [0, 0, 1], [1, 0, 1], [1, 1, 1]],
    ]
    x = np.array(x).astype(float)
    for i in range(3):
        x[:, :, i] *= size[i]
    x += np.array(o)
    return x


def plot_cube_at_2(positions, sizes=None, colors=None, **kwargs):
    if not isinstance(colors, (list, np.ndarray)):
        colors = ["C0"] * len(positions)
    if not isinstance(sizes, (list, np.ndarray)):
        sizes = [(1, 1, 1)] * len(positions)
    g = []
    for p, s, c in zip(positions, sizes, colors):
        g.append(cuboid_data_2(p, size=s))
    return Poly3DCollection(np.concatenate(g), facecolors=np.repeat(colors, 6), **kwargs)


def create_cuboid_mesh(cuboid_dx, cuboid_dy, cuboid_dz, color):
    i = [7, 0, 0, 0, 4, 4, 6, 6, 4, 0, 3, 2]
    j = [3, 4, 1, 2, 5, 6, 5, 2, 0, 1, 6, 3]
    k = [0, 7, 2, 3, 6, 7, 1, 1, 5, 5, 7, 6]

    triangles = np.vstack((i, j, k)).T

    x = ([0.0, 0.0, cuboid_dx, cuboid_dx, 0.0, 0.0, cuboid_dx, cuboid_dx],)
    y = ([0.0, cuboid_dy, cuboid_dy, 0.0, 0.0, cuboid_dy, cuboid_dy, 0.0],)
    z = ([0.0, 0.0, 0.0, 0.0, cuboid_dz, cuboid_dz, cuboid_dz, cuboid_dz],)
    vertices = np.vstack((x, y, z)).T
    tri_points = vertices[triangles]

    # extract the lists of x, y, z coordinates of the triangle vertices and connect them by a line
    x_extended = []
    y_extended = []
    z_extended = []
    for t in tri_points:
        x_extended.extend([t[k % 3][0] for k in range(4)] + [None])
        y_extended.extend([t[k % 3][1] for k in range(4)] + [None])
        z_extended.extend([t[k % 3][2] for k in range(4)] + [None])

    # define the trace for triangle sides
    container_outline = go.Scatter3d(
        x=x_extended, y=y_extended, z=z_extended, mode="lines", name="", line=dict(color=color, width=1)
    )

    return container_outline


def add_item_mesh(item, meshes):
    x = item["X"]
    y = item["Y"]
    z = item["Z"]

    is_rotated = item["Rotated"] != "None"

    dx = item["Dx"] if not is_rotated else item["Dy"]
    dy = item["Dy"] if not is_rotated else item["Dx"]
    dz = item["Dz"]

    is_fragile = item["Fragility"] != "None"

    fragile = "Y" if is_fragile else "N"
    rotatable = "Y" if item["EnableHorizontalRotation"] is True else "N"
    rotated = "Y" if is_rotated else "N"

    color = item["Color"]
    fragility_opacity = 0.8 if is_fragile else 1.0

    site_id = item["SiteId"]

    item_mesh = go.Mesh3d(
        # 8 vertices of a cube
        x=[x, x, x + dx, x + dx, x, x, x + dx, x + dx],
        y=[y, y + dy, y + dy, y, y, y + dy, y + dy, y],
        z=[z, z, z, z, z + dz, z + dz, z + dz, z + dz],
        i=[7, 0, 0, 0, 4, 4, 6, 6, 4, 0, 3, 2],
        j=[3, 4, 1, 2, 5, 6, 5, 2, 0, 1, 6, 3],
        k=[0, 7, 2, 3, 6, 7, 1, 1, 5, 5, 7, 6],
        opacity=fragility_opacity,
        color=color,
        flatshading=True,
        hovertext=f"Fragile: {fragile} <br>Rotatable {rotatable} <br>Rotated {rotated} <br> Node: {site_id}",
    )

    meshes.append(item_mesh)


def add_stop_meshes(stops, meshes):
    for stop in stops:
        for item in stop["Items"]:
            add_item_mesh(item, meshes)


def add_container_mesh(container, meshes):
    container_dx = container["Dx"]
    container_dy = container["Dy"]
    container_dz = 0.0
    original_container_dz = container["Dz"]

    container_color = container["Color"]

    container_mesh = go.Mesh3d(
        # 8 vertices of a cube
        x=[0.0, 0.0, container_dx, container_dx, 0.0, 0.0, container_dx, container_dx],
        y=[0.0, container_dy, container_dy, 0.0, 0.0, container_dy, container_dy, 0.0],
        z=[0.0, 0.0, 0.0, 0.0, container_dz, container_dz, container_dz, container_dz],
        i=[7, 0, 0, 0, 4, 4, 6, 6, 4, 0, 3, 2],
        j=[3, 4, 1, 2, 5, 6, 5, 2, 0, 1, 6, 3],
        k=[0, 7, 2, 3, 6, 7, 1, 1, 5, 5, 7, 6],
        opacity=0.1,
        color=container_color,
        flatshading=True,
    )

    meshes.append(container_mesh)

    container_outline = create_cuboid_mesh(container_dx, container_dy, original_container_dz, container_color)

    meshes.append(container_outline)


def make_packing_figure(packing, selected_tour_ids):
    tour_id = packing["TourId"]

    if tour_id not in selected_tour_ids:
        return

    container = packing["Container"]

    meshes = []

    add_container_mesh(packing["Container"], meshes)

    add_stop_meshes(packing["Stops"], meshes)

    # https://computergraphics.stackexchange.com/questions/8960/what-are-the-i-j-k-components-of-a-3d-mesh-on-plot-ly-online
    # https://stackoverflow.com/questions/60371624/drawing-a-3d-box-in-a-3d-scatterplot-using-plotly
    # https://stackoverflow.com/questions/62761970/checking-intersection-of-generated-cuboids-spheres
    fig = go.Figure(data=meshes)
    fig.update_layout(
        scene=dict(
            xaxis=dict(
                nticks=0,
                range=[0, container["Dx"]],
            ),
            yaxis=dict(
                nticks=0,
                range=[0, container["Dy"]],
            ),
            zaxis=dict(
                nticks=0,
                range=[0, container["Dz"]],
            ),
        ),
        scene_xaxis_visible=False,
        scene_yaxis_visible=False,
        scene_zaxis_visible=False,
    )

    return fig
