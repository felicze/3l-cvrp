#pragma once

#include "gurobi_c++.h"
#include <vector>

using GRBVar1D = std::vector<GRBVar>;
using GRBVar2D = std::vector<std::vector<GRBVar>>;