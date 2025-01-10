#include "Algorithms/Cuts/BaseCut.h"

namespace VehicleRouting
{
namespace Algorithms
{
namespace Cuts
{
std::vector<Cut> BaseCut::GetCuts(const std::vector<std::vector<double>>& x) { return FindCuts(x); }

}
}
}