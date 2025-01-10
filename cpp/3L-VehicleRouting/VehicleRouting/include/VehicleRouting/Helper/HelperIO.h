#pragma once

#include "Algorithms/BCRoutingParams.h"
#include "Model/Instance.h"
#include "Model/Node.h"

#include <boost/algorithm/string.hpp>

#include <fstream>
#include <iostream>

namespace VehicleRouting
{
namespace Helper
{
using namespace VehicleRouting::Model;
using namespace VehicleRouting::Algorithms;
using namespace ContainerLoading::Model;

class HelperIO
{
  public:
    static Instance ParseInstanceJson(std::ifstream& ifs);

    static VehicleRouting::Algorithms::InputParameters ReadInputParameters(std::string& parameterFilePath);

    static std::vector<Route> ParseSolutionJson(std::ifstream& ifs);

    template <typename E> void PrintEnum(E e)
    {
        // Prefix '+' so that std::cout prints 'char' as a number instead of ASCII
        std::cout << +static_cast<std::underlying_type_t<E>>(e) << "\n";
    }
};

}
}