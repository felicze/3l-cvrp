#pragma once

namespace ContainerLoading
{
namespace Algorithms
{
struct CPSolverParams
{
    int Threads = 8;
    int Seed = 0;
    bool LogFlag = true;
    bool Presolve = true;

    bool EnableCumulativeDimensions = false;
    bool EnableNoOverlap2DFloor = false;
};

}
}