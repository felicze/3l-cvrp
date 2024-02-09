import streamlit as st
import pandas as pd

import plotly.graph_objects as go
import plotly.express as px


def plot_solver_progress(lb_data, ub_data, time_offset=0.0):
    fig = go.Figure()
    fig.add_trace(
        go.Scatter(
            x=lb_data["Time"] + time_offset, y=lb_data["LB"], mode="lines", name="lower_bound", line={"shape": "hv"}
        )
    )

    fig.add_trace(
        go.Scatter(
            x=ub_data["Time"] + time_offset,
            y=ub_data["ObjVal"],
            mode="lines+markers",
            name="upper_bound",
            line={"shape": "hv"},
        )
    )

    fig.update_layout(
        xaxis_title="Time in seconds",
        yaxis_title="Objective function value",
        font=dict(
            size=12,
        ),
        hovermode="x unified",
    )

    st.plotly_chart(fig, use_container_width=False)


def plot_element_count_sankey(element_data, value, consider_memory_mgmt=True):
    nodes = [
        ["ID", "Label"],
        [0, "Routes"],
        [1, "Single customer routes"],
        [2, "Disconnected route"],
        [3, "Connected routes"],
        [4, "1D approx inf"],
        [5, "Route prechecked feas"],
        [6, "Combination prechecked inf"],
        [7, "Heuristic feas"],
        [8, "Exact check"],
        [9, "Feasible"],
        [10, "Unknown"],
        [11, "Two path"],
        [12, "Regular tournament path"],
        [13, "Tail tournament path"],
        [14, "Add constraint(s)"],
        [15, "Accept route"],
        [16, "Add new route(s)"],
        [17, "Skip route"],
        [18, "Tail path"],
        [19, "Heuristic inf"],
    ]

    cumulated_data_exact_check = 0
    cumulated_data_connected = 0
    data = element_data[value].copy()

    links = [
        ["Source", "Target", "Value"],
        # all routes
        [0, 1, data["SingleCustRoutes"]],
        [0, 2, data["DisconnectedRoutes"]],
        [0, 3, data["ConnectedRoutes"] + cumulated_data_connected + cumulated_data_exact_check],
        # Connected routes
        [3, 4, data["MinVehApproxInf"]],
        [3, 5, data["RoutePrechecked"]],
        [3, 6, data["CustCombiInfeasible"]],
        [3, 7, data["HeuristicFeas"]],
        [3, 19, data["HeuristicInf"]],
        [19, 8, data["HeuristicInf"]],
        [3, 8, cumulated_data_exact_check],
        # Exact check routes
        [8, 9, data["ExactLimitFeas"] + data["ExactFeas"]],
        # [8, 10, data["ExactLimitUnk"]],
        [8, 11, data["TwoPath"]],
        [8, 12, data["RegTournamentPath"]],
        [8, 13, data["TailTournamentPath"]],
        [8, 18, data["InfTailPath"] if "InfTailPath" in data else 0],
        # End links
        [1, 15, data["SingleCustRoutes"]],
        [2, 14, data["DisconnectedRoutes"]],
        [4, 14, data["MinVehApproxInf"]],
        [5, 15, data["RoutePrechecked"]],
        [6, 14, data["CustCombiInfeasible"]],
        # [10,17, data["ExactLimitUnk"]],
        [11, 14, data["TwoPath"]],
        [12, 14, data["RegTournamentPath"]],
        [13, 14, data["TailTournamentPath"]],
        [18, 14, data["InfTailPath"] if "InfTailPath" in data else 0],
    ]

    target_feasible_check = 16 if consider_memory_mgmt else 15
    links.append([7, target_feasible_check, data["HeuristicFeas"]])
    links.append([9, target_feasible_check, data["ExactLimitFeas"] + data["ExactFeas"]])

    # ensure connectivity in case of no heuristic, no Memory mgmt etc.
    remaining_rest = (
        data["ConnectedRoutes"]
        - data["HeuristicInf"]
        - data["MinVehApproxInf"]
        - data["RoutePrechecked"]
        - data["CustCombiInfeasible"]
        - data["HeuristicFeas"]
    )
    if remaining_rest > 0:
        links.append([3, 8, remaining_rest])

    nodes_headers = nodes.pop(0)
    links_headers = links.pop(0)
    df_nodes = pd.DataFrame(nodes, columns=nodes_headers)
    df_links = pd.DataFrame(links, columns=links_headers)

    fig = go.Figure(
        go.Sankey(
            node={
                "label": df_nodes["Label"].dropna(axis=0, how="any"),
                "pad": 25,
                "thickness": 15,
            },
            link={
                "source": df_links["Source"].dropna(axis=0, how="any"),
                "target": df_links["Target"].dropna(axis=0, how="any"),
                "value": df_links["Value"].dropna(axis=0, how="any"),
            },
        )
    )

    fig.update_layout(title_text=value, margin=dict(l=5, r=5, b=5, t=20), font=dict(size=8))
    st.plotly_chart(fig, use_container_width=True)


def plot_element_bar(element_data):
    count_data = element_data["Count"]
    time_data = element_data["Time"] / 1e6
    bars = [
        ["Operation", "Count", "Time"],
        ["SingleVehicle", count_data["SingleVehicle"], time_data["SingleVehicle"]],
        [
            "Precheck route",
            count_data["RoutePrechecked"] + count_data["RoutePrecheckedNot"],
            time_data["RoutePrechecked"] + time_data["RoutePrecheckedNot"],
        ],
        [
            "Check cust combi",
            count_data["CustCombiInfeasible"] + count_data["CustCombiInfNot"],
            time_data["CustCombiInfeasible"] + time_data["CustCombiInfNot"],
        ],
        [
            "Check heuristic",
            count_data["HeuristicFeas"] + count_data["HeuristicInf"],
            time_data["HeuristicFeas"] + time_data["HeuristicInf"],
        ],
        [
            "Exact check ",
            count_data["ExactLimitFeas"]
            + count_data["ExactLimitInf"]
            + count_data["ExactLimitUnk"]
            + count_data["ExactFeas"]
            + count_data["ExactInf"]
            + count_data["ExactInvalid"],
            time_data["ExactLimitFeas"]
            + time_data["ExactLimitInf"]
            + time_data["ExactLimitUnk"]
            + time_data["ExactFeas"]
            + time_data["ExactInf"]
            + time_data["ExactInvalid"],
        ],
        [
            "Two path lift",
            count_data["TwoPath"] + count_data["TwoPathNot"],
            time_data["TwoPath"] + time_data["TwoPathNot"],
        ],
        [
            "Regular tournament lift",
            count_data["RegTournamentPath"] + count_data["RegTournamentPathNot"],
            time_data["RegTournamentPath"] + time_data["RegTournamentPathNot"],
        ],
    ]
    if "InfTailPath" in count_data:
        bars.append(["Tail path", count_data["InfTailPath"], time_data["InfTailPath"]])
    else:
        bars.append(["Tail tournament path", count_data["TailTournamentPath"], time_data["TailTournamentPath"]])
    bars.append(["Reverse path check", count_data["RevSeq"], time_data["RevSeq"]])
    bars_headers = bars.pop(0)

    df_bars = pd.DataFrame(bars, columns=bars_headers)

    fig = go.Figure(
        data=[
            go.Bar(name="Count", x=df_bars["Operation"], y=df_bars["Count"], yaxis="y", offsetgroup=1),
            go.Bar(name="Time", x=df_bars["Operation"], y=df_bars["Time"], yaxis="y2", offsetgroup=2),
        ],
        layout={
            "yaxis": {"title": "Call count"},
            "yaxis2": {"title": "Time in seconds", "overlaying": "y", "side": "right"},
        },
    )
    fig.update_layout(barmode="group", xaxis_title="Operation")

    st.plotly_chart(fig, use_container_width=True)


def plot_lazy_constraints_bar(data):
    fig = px.bar(data, x=data.index, y="Count", labels={"Count": "# Lazy constraints", "index": "Lazy constraint type"})

    st.plotly_chart(fig, use_container_width=True)


def plot_cuts_bar(cut_data):
    fig = go.Figure(
        data=[
            go.Bar(name="Count", x=cut_data.index, y=cut_data["Count"], yaxis="y", offsetgroup=1),
            go.Bar(name="Time", x=cut_data.index, y=cut_data["Time"] / 1e6, yaxis="y2", offsetgroup=2),
        ],
        layout={
            "yaxis": {"title": "Cuts found"},
            "yaxis2": {"title": "Time in seconds", "overlaying": "y", "side": "right"},
        },
    )

    fig.update_layout(barmode="group", xaxis_title="Cut type")

    st.plotly_chart(fig)


def analyze_solver_statistics(solver_statistics_data):
    st.subheader("Solver progress")

    time_preprocessing = solver_statistics_data["Summary"]["Preprocessing"]
    plot_solver_progress(
        solver_statistics_data["LBProgress"], solver_statistics_data["UBProgress"], time_offset=time_preprocessing
    )

    st.subheader("Solver components")

    st.subheader("Integer solutions")
    col1, col2 = st.columns(2)
    with col1:
        plot_element_bar(solver_statistics_data["ElementData"])
    with col2:
        plot_lazy_constraints_bar(solver_statistics_data["LazyConstraintsCount"])

    plot_element_count_sankey(solver_statistics_data["ElementData"], "Count")

    if "CutData" in solver_statistics_data:
        st.subheader("Fractional solutions")
        plot_cuts_bar(solver_statistics_data["CutData"])
