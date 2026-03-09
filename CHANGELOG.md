# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added
- Tracking of incremental feasibility property (IFP) for routes
- DOI Link to accepted article in EJOR

### Changed
- 1D SubtourTracker updated to properly handle disconnected routes
- Update solution files to the final version of the accepted manuscript

### Fixed
- FCI (Framed Capacity Inequality) cut generation restored to original formulation
- Add fractional cuts as LazyConstraints instead of UserCuts in Gurobi callback
- Bug in logic for generating (Meet-in-the-middle) placement points in loading subproblem
- Fixed incorrect namespace scope in callback element serialization string definitions
- Visualization error in solution statistics that occurred when time was spent on fractional cut separation without identifying any cuts

---

## [0.1.0] - 2025-01-11

### Added
- Initial release of the branch-and-cut algorithm for 3L-CVRP
- Constraint programming model for loading subproblem
- Infeasible path elimination with variant-specific lifting
- Start solution procedure and set-partitioning-based heuristic
- Support for five 3L-CVRP variants with different loading constraints
- Benchmark instances and solutions
- Python visualization tools for solutions and statistics
