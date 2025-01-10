# Vehicle routing problems with three-dimensional loading constraints

This repo is part of the manuscript *A branch-and-cut algorithm for vehicle routing problems with three-dimensional loading constraints*, which is currently under review.

The repo contains the instances, solutions, the source code of the algorithm, and a visualizer for the solutions and solver statistics.

## Abstract
This paper presents a new branch-and-cut algorithm based on infeasible path elimination for the three-dimensional loading capacitated vehicle routing problem (3L-CVRP) with different loading problem variants. We show that a previously infeasible route can become feasible by adding a new customer if support constraints are enabled in the loading subproblem and call this the incremental feasibility property. Consequently, different infeasible path definitions apply to different 3L-CVRP variants and we introduce several variant-depending lifting steps to strengthen infeasible path inequalities. The loading subproblem is solved exactly using a flexible constraint programming model to determine the feasibility or infeasibility of a route. An extreme point-based packing heuristic is implemented to reduce time-consuming calls to the exact loading algorithm. Furthermore, we integrate a start solution procedure and periodically combine memoized feasible routes in a set-partitioning-based heuristic to generate new upper bounds. A comprehensive computational study, employing well-known benchmark instances, showcases the significant performance improvements achieved through the algorithmic enhancements. Consequently, we not only prove the optimality of many best-known heuristic solutions for the first time but also introduce new optimal and best solutions for a large number of instances.

## Instances
Instances are the classical 3L-CVRP instances introduced in [Gendreau et al. (2006)](https://doi.org/10.1287/trsc.1050.0145). We use only instances with at most 50 customer nodes.

<pre>
.
└── /data/input/3l-cvrp/
    ├── parameters/ 
    ├── E016-03m.json
    ├── E016-05m.json
    ├── ...
    └── E051-05e.json
</pre>

In addition, we provide the parameter files used in the benchmark tests.

## Solutions

[Gendreau et al. (2006)](https://doi.org/10.1287/trsc.1050.0145) consider different constraints in the container loading subproblem and introduce the following five variants.

<table class="tg">
<thead>
  <tr>
    <th class="tg-0pky">Variant</th>
    <th class="tg-c3ow" colspan="5">Constraints   </th>
  </tr>
</thead>
<tbody>
  <tr>
    <td class="tg-0pky"></td>
    <td class="tg-0pky">no-overlap</td>
    <td class="tg-0pky">rotation</td>
    <td class="tg-0pky">support</td>
    <td class="tg-0pky">fragility</td>
    <td class="tg-0pky">lifo</td>
  </tr>
  <tr>
    <td class="tg-0pky">all-constraints</td>
    <td class="tg-c3ow">x</td>
    <td class="tg-c3ow">x</td>
    <td class="tg-c3ow">x</td>
    <td class="tg-c3ow">x</td>
    <td class="tg-c3ow">x</td>
  </tr>
  <tr>
    <td class="tg-0pky">no-fragility</td>
    <td class="tg-c3ow">x</td>
    <td class="tg-c3ow">x</td>
    <td class="tg-c3ow">x</td>
    <td class="tg-c3ow"></td>
    <td class="tg-c3ow">x</td>
  </tr>
  <tr>
    <td class="tg-0pky">no-lifo</td>
    <td class="tg-c3ow">x</td>
    <td class="tg-c3ow">x</td>
    <td class="tg-c3ow">x</td>
    <td class="tg-c3ow">x</td>
    <td class="tg-c3ow"></td>
  </tr>
  <tr>
    <td class="tg-0pky">no-support</td>
    <td class="tg-c3ow">x</td>
    <td class="tg-c3ow">x</td>
    <td class="tg-c3ow"></td>
    <td class="tg-c3ow">x</td>
    <td class="tg-c3ow">x</td>
  </tr>
  <tr>
    <td class="tg-0pky">loading-only</td>
    <td class="tg-c3ow">x</td>
    <td class="tg-c3ow">x</td>
    <td class="tg-c3ow"></td>
    <td class="tg-c3ow"></td>
    <td class="tg-c3ow"></td>
  </tr>
</tbody>
</table>
We provide solutions for all variants. In addition, we provide solutions for the one-dimensional approximation (CVRP) of the 3L-CVRP using weight and volume as capacities and solutions for variant all constraints applying a basic variant of the branch-and-cut algorithm (all-constraints-basic). 

<pre>
.
└── /data/output/3l-cvrp/
    └── <em>variant</em>/
        └── <em>name</em>/run-0/
            ├── solution-validator/ # <b> files for <a href="https://github.com/CorinnaKrebs/Visualizer">solution validator</a> </b>
            │   ├── instance-<em>name</em>.txt
            │   └── solution-<em>name</em>.txt
            ├── <em>name</em>.LOG # <b> Gurobi log-file </b>
            ├── solution-<em>name</em>.json
            ├── solution-statistics-<em>name</em>.json
</pre>

## Branch-and-cut algorithm
The source code of the branch-and-cut algorithm is located in subdirectory [cpp/3L-VehicleRouting](cpp/3L-VehicleRouting). We use CMake (version 3.18 or newer) as cross-platform build system generator.

> [!NOTE]
>  The loading subproblem is solved exactly using the CP-SAT solver from Google OR-Tools to determine the feasibility or infeassibility of a route. The extreme-point based packing heuristic is not part of the source code.

### External code
- [A-MDVRP](https://github.com/michieluithetbroek/A-MDVRP) and [paper](https://doi.org/10.1287/opre.2020.2033) for separation of cuts
- [CVRPSEP](https://github.com/sassoftware/cvrpsep) for separation of cuts 
- [CLI11](https://github.com/CLIUtils/CLI11) for command line parsing
- [nlohmann](https://github.com/nlohmann/json) for json serialization

### Setup General

#### Requirements
- [Gurobi 10.0.3](https://www.gurobi.com/downloads/) or newer
- [Google or-tools 9.8](https://github.com/google/or-tools/releases/tag/v9.8) or newer
- [boost 1.79](https://www.boost.org/users/history/version_1_79_0.html) or newer

The libraries as prerequisites must be installed locally on the machine. The installation paths must be transferred. See for example [settings.json](https://github.com/felicze/3l-cvrp/blob/main/.vscode/settings.json)

#### Linux
Clone the repo.
```
git clone https://github.com/felicze/3l-cvrp.git
```

Install [boost 1.79](https://www.boost.org/users/history/version_1_79_0.html) or newer.

Install [tbb](https://askubuntu.com/questions/1170054/install-newest-tbb-thread-building-blocks-on-ubuntu-18-04).

Download [or-tools 9.8](https://github.com/google/or-tools/releases/tag/v9.8) or newer and [gurobi 10.0.3](https://www.gurobi.com/downloads/) or newer and extract to `/opt`:
```
sudo tar -xvzf or-tools_amd64_ubuntu-22.04_cpp_v9.8.3296.tar.gz -C /opt 
sudo tar -xvzf gurobi10.0.3_linux64.tar.gz -C /opt
```

Rebuild Gurobi for [full compatibility with different compilers](https://support.gurobi.com/hc/en-us/articles/360039093112-How-do-I-resolve-undefined-reference-errors-while-linking-Gurobi-in-C).

If necessary, adjust the or-tools and Gurobi paths in `.vscode/settings.json`.

Configure and build the project (tested on GCC 12.3.0 or newer and Clang 16.0.6 or newer).

#### Windows
Clone the repo.
```
git clone https://github.com/felicze/3l-cvrp.git
```
Install [boost 1.79](https://www.boost.org/users/history/version_1_79_0.html) or newer.

Download [or-tools 9.8](https://github.com/google/or-tools/releases/tag/v9.8) or newer and [gurobi 10.0.3](https://www.gurobi.com/downloads/) or newer.


If necessary, adjust the or-tools and Gurobi paths in `.vscode/settings.json`.

Configure and build the project (tested on MSVC 17.12.12).

### Run the algorithm

To run the code, you must specify four command-line arguments (see `.vscode/launch.json`):
- `-i`: input directory
- `-f`: input file
- `-o`: output directory
- `-p`: parameter file

You can run the code:

- From your editor: 
Provide the command-line arguments in the editor's configuration (e.g., in `.vscode/launch.json`).
- From the command line:
Navigate to the root directory of the repository and run, for example: 
```
build/Release/bin/3L-VehicleRoutingApplication -i data/input/3l-cvrp/ -f E023-03g.json -o data/output/3l-cvrp/test/ -p data/input/3l-cvrp/parameters/BenchmarkParameters_AllConstraints.json
```

## Visualizer
This visualizer is a python app using [Streamlit](https://streamlit.io/). We only provide a visualization of solutions and some solver statistics. The app **cannot** be used to check the feasibility of solutions. If you want to do this, we refer to the paper by  [Krebs & Ehmke (2023)](https://doi.org/10.1007/s10479-023-05238-0) and the accompanying [solution validator](https://github.com/CorinnaKrebs/SolutionValidator) and [visualizer](https://github.com/CorinnaKrebs/Visualizer).

>[!NOTE]  
> It should be noted that items can hover in our solutions if support constraints are disabled as the supported area can be zero. This is prohibited in [Krebs & Ehmke (2023)](https://doi.org/10.1007/s10479-023-05238-0). Thus, our solutions for variants no-support and loading-only might be infeasible using the solution validator. However, in the case of the loading-only variant, all floating items could be lowered enough to touch an underlying object, resulting in a feasible solution. This is not possible for the no-support variant due to the fragility constraint.

### Requirements
- Python version >= 3.10 and < 3.13
- Poetry version 2.0.0 or higher, see [official documentation](https://python-poetry.org/docs/)

### Run visualizer locally

1. Clone repository
2. Go to subdirectory [python](python)
3. Install dependencies with 
<pre>
poetry install
or
pip install -r requirements.txt
</pre>
4. Activate virtual environment if necessary
5. Run streamlit web app locally with 
<pre>
streamlit run visualization/SolutionVisualizer.py
</pre>

### Solutions
Select a solution file from the output directory, e.g, `data/output/3l-cvrp/all-constraints/e023-03g/run-0/solution-E023-03g.json`.

![](assets/images/solution.png)
### Solver information
Select a solution statistics file from the output directory, e.g., `data/output/3l-cvrp/all-constraints/e023-03g/run-0/solution-statistics-E023-03g.json`.

![](assets/images/solver-statistics.png)
