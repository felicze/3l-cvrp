#pragma once

#include "ortools/sat/cp_model.h"
#include <vector>

using ORIntervalVars = std::vector<operations_research::sat::IntervalVar>;
using ORIntVars1D = std::vector<operations_research::sat::IntVar>;
using ORIntVars2D = std::vector<std::vector<operations_research::sat::IntVar>>;
using ORBoolVars1D = std::vector<operations_research::sat::BoolVar>;
using ORBoolVars2D = std::vector<std::vector<operations_research::sat::BoolVar>>;
using ORBoolVars3D = std::vector<std::vector<std::vector<operations_research::sat::BoolVar>>>;
using ORLinExpr = operations_research::sat::LinearExpr;