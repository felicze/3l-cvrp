#pragma once
#include "Model/Container.h"
#include "Model/ContainerLoadingInstance.h"

#include <algorithm>
#include <boost/format.hpp>
#include <filesystem>
#include <fstream>

namespace ResultWriter
{
namespace SolutionValidator
{
using boost::format;
using boost::io::group;
using namespace ContainerLoading::Model;

static void WriteInput(const std::string& folderPath,
                       const std::string& name,
                       const std::vector<Group>& nodes,
                       const Container& container,
                       const size_t numberContainers)
{
    std::string path = folderPath + "SolutionValidator/";
    if (!std::filesystem::is_directory(path) || !std::filesystem::exists(path))
    {
        std::filesystem::create_directory(path);
    }

    std::ofstream inputFile;
    inputFile.open(path + "/Instance_" + name + ".txt");

    auto numberItems = nodes.back().Items.back().ExternId + 1;

    inputFile << format("%-30s %s\n") % "Name" % name;
    inputFile << format("%-30s %s\n") % "Number_of_Customers" % (nodes.size() - 1);
    inputFile << format("%-30s %s\n") % "Number_of_Items" % numberItems;
    inputFile << format("%-30s %s\n") % "Number_of_ItemTypes" % numberItems;
    inputFile << format("%-30s %s\n") % "Number_of_Vehicles" % numberContainers;
    inputFile << format("%-30s %s\n") % "TimeWindows" % 0;

    inputFile << "\nVEHICLE\n";
    inputFile << format("%-30s %s\n") % "Mass_Capacity" % container.WeightLimit;
    inputFile << format("%-30s %s\n") % "CargoSpace_Length" % container.Dx;
    inputFile << format("%-30s %s\n") % "CargoSpace_Width" % container.Dy;
    inputFile << format("%-30s %s\n") % "CargoSpace_Height" % container.Dz;
    inputFile << format("%-30s %s\n") % "Wheelbase" % -1;
    inputFile << format("%-30s %s\n") % "Max_Mass_FrontAxle" % -1;
    inputFile << format("%-30s %s\n") % "Max_Mass_RearAxle" % -1;
    inputFile << format("%-30s %s\n") % "Distance_FrontAxle_CargoSpace" % -1;

    inputFile << "\nCUSTOMERS\n";
    inputFile << format("%-15s %-15s %-15s %-15s %-15s %-15s %-15s %-15s %s\n") % "i" % "x" % "y" % "Demand"
                     % "ReadyTime" % "DueDate" % "ServiceTime" % "DemandedMass" % "DemandedVolume";

    auto printNode = [&inputFile](const Group& node)
    {
        inputFile << format("%-15s %-15s %-15s %-15s %-15s %-15s %-15s %-15s %s\n") % node.InternId % node.PositionX
                         % node.PositionY % node.Items.size() % 0 % 1000000 % 0 % node.TotalWeight % node.TotalVolume;
    };

    std::ranges::for_each(nodes, printNode);

    inputFile << "\nITEMS\n";
    inputFile << format("%-15s %-15s %-15s %-15s %-15s %-15s %s\n") % "Type" % "Length" % "Width" % "Height" % "Mass"
                     % "Fragility" % "LoadingBearingStrength";

    auto printItems = [&inputFile](const Group& node)
    {
        for (const auto& item: node.Items)
        {
            auto mass = node.TotalWeight / static_cast<double>(node.Items.size());
            inputFile << format("Bt%-13s %-15s %-15s %-15s %-15d %-15s %s\n") % (item.InternId + 1) % item.Dx % item.Dy
                             % item.Dz % mass % (item.Fragility == Fragility::None ? 0 : 1) % 0;
        }
    };

    std::ranges::for_each(nodes, printItems);

    inputFile << "\nDEMANDS PER CUSTOMER\n";
    inputFile << format("%-4s %-4s %-2s\n") % "i" % "Type" % "Quantity";

    auto printDemand = [&inputFile](const Group& node)
    {
        if (node.Items.size() == 0)
        {
            return;
        }

        inputFile << format("%-5s") % node.InternId;
        for (const auto& item: node.Items)
        {
            inputFile << format("Bt%-2s %-2s") % (item.InternId + 1) % 1;
        }

        inputFile << "\n";
    };

    std::ranges::for_each(nodes, printDemand);
}

static void WriteOutput(const std::string& folderPath,
                        const std::string& name,
                        std::vector<std::vector<Group>>& routes,
                        const double costs)
{
    std::string path = folderPath + "SolutionValidator/";
    if (!std::filesystem::is_directory(path) || !std::filesystem::exists(path))
    {
        std::filesystem::create_directory(path);
    }

    std::ofstream inputFile;
    inputFile.open(path + "/Solution_" + name + ".txt");

    inputFile << format("%-30s %s\n") % "Name:" % name;
    inputFile << format("%-30s %s\n") % "Problem:" % "3L-CVRP";
    inputFile << format("%-30s %s\n") % "Number_of_used_Vehicles:" % routes.size();
    inputFile << format("%-30s %s\n") % "Total_Travel_Distance:" % costs;
    inputFile << format("%-30s %s\n") % "Calculation_Time:" % -1;
    inputFile << format("%-30s %s\n") % "Total_Iterations:" % -1;
    inputFile << format("%-30s %s\n") % "ConstraintSet:" % 1;

    inputFile << "\n";

    int routeId = 1;

    auto printSequence = [](const std::vector<Group>& route)
    {
        std::string sequence;
        for (const auto& node: route)
        {
            sequence.append(std::to_string(node.InternId)).append(" ");
        }

        return sequence;
    };

    auto printStop = [&inputFile](const Group& node)
    {
        auto mass = node.TotalWeight / static_cast<double>(node.Items.size());
        for (const auto& item: node.Items)
        {
            inputFile << format("%-9s %-9s %-9s %-9s %-9s %-9s %-9s %-9s %-9s %-9s %-9d %-9s %s\n") % node.InternId
                             % (item.InternId + 1) % (item.InternId + 1) % (item.Rotated == Rotation::None ? 0 : 1)
                             % item.X % item.Y % item.Z % item.Dx % item.Dy % item.Dz % mass
                             % (item.Fragility == Fragility::None ? 0 : 1) % 0;
        }
    };

    auto printRoute = [&inputFile, &routeId, &printSequence, &printStop](const std::vector<Group>& route)
    {
        auto numberOfCustomers = route.size();
        size_t numberOfItems = 0;
        for (const auto& node: route)
        {
            numberOfItems += node.Items.size();
        }

        inputFile
            << "------------------------------------------------------------------------------------------------\n";
        inputFile << format("%-30s %s\n") % "Tour_Id:" % (routeId);
        inputFile << format("%-30s %s\n") % "No_of_Customers:" % numberOfCustomers;
        inputFile << format("%-30s %s\n") % "No_of_Items:" % numberOfItems;
        inputFile << format("%-30s %s\n") % "Customer_Sequence:" % printSequence(route);

        inputFile << "\n";

        inputFile << format("%-9s %-9s %-9s %-9s %-9s %-9s %-9s %-9s %-9s %-9s %-9s %-9s %s\n") % "CustId" % "Id"
                         % "TypeId" % "Rotated" % "x" % "y" % "z" % "Length" % "Width" % "Height" % "mass" % "Fragility"
                         % "LoadingBearingStrength";

        std::ranges::for_each(route, printStop);

        inputFile << "\n";
        inputFile << "\n";
    };

    std::ranges::for_each(routes, printRoute);
}
}
}