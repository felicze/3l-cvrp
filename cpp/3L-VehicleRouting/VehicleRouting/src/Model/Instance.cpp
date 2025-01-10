#include "Model/Instance.h"

#include <math.h>

namespace VehicleRouting
{
namespace Model
{
void Instance::DetermineDistanceMatrixEuclidean()
{
    for (size_t i = 0; i < Nodes.size(); ++i)
    {
        const Node& nodeI = Nodes[i];
        mDistances[i] = std::map<size_t, double>();
        for (size_t j = 0; j < Nodes.size(); ++j)
        {
            const Node& nodeJ = Nodes[j];

            double dx = nodeI.PositionX - nodeJ.PositionX;
            double dy = nodeI.PositionY - nodeJ.PositionY;

            mDistances[i][j] = sqrt((dx * dx) + (dy * dy));
        }
    }
}

}
}