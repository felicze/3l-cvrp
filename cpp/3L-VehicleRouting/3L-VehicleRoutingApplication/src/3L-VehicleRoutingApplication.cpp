// 3L-VehicleRoutingApplication.cpp : This file contains the 'main' function. Program execution begins and ends there.

#include "VehicleRouting/Algorithms/BCRoutingParams.h"
#include "VehicleRouting/Algorithms/BranchAndCutSolver.h"
#include "VehicleRouting/Helper/HelperIO.h"
#include "VehicleRouting/Model/Instance.h"

#include "CLI11/CLI11.hpp"

#include <fstream>
#include <iostream>
#include <string>

#include <ctime>
#include <filesystem>
#include <iomanip>

using namespace VehicleRouting;
using namespace VehicleRouting::Algorithms;
using namespace VehicleRouting::Model;
using namespace VehicleRouting::Helper;

void Run(std::string& inputFilePath,
         std::string& filename,
         std::string& parameterFile,
         std::string& outdir,
         bool enableTimeSuffix,
         int seedOffset)
{
    InputParameters inputParameters;

    if (parameterFile != "")
    {
        inputParameters = HelperIO::ReadInputParameters(parameterFile);
    }
    else
    {
        inputParameters.ContainerLoading.LoadingProblem.Variant = LoadingProblemParams::VariantType::AllConstraints;
    }

    inputParameters.SetLoadingFlags();
    std::ifstream ifs(inputFilePath + filename);
    ////std::ifstream ifs("data/3LVRP/ConvertedInstances/E016-05m.json");

    // https://stackoverflow.com/questions/16357999/current-date-and-time-as-string/16358111
    auto t = std::time(nullptr);
    auto tm = *std::localtime(&t);

    std::ostringstream oss;
    oss << std::put_time(&tm, "%Y-%m-%d_%H-%M-%S");
    std::string dateTimeString = oss.str();

    std::string outputPath = outdir + "/";
    if (enableTimeSuffix)
    {
        outputPath = outdir + dateTimeString + "/";
    }

    // https://stackoverflow.com/a/37524002
    if (!std::filesystem::is_directory(outdir) || !std::filesystem::exists(outdir))
    {
        std::filesystem::create_directory(outdir);
    }

    if (!std::filesystem::is_directory(outputPath) || !std::filesystem::exists(outputPath))
    {
        std::filesystem::create_directory(outputPath);
    }

    if (!ifs.is_open())
    {
        std::cerr << "File does not exist or cannot be opened: " << inputFilePath + filename << std::endl;
        return; // or handle the error as needed
    }

    auto instance = HelperIO::ParseInstanceJson(ifs);

    ////std::ofstream ofs("logfile.txt");
    ////std::cout.rdbuf(ofs.rdbuf());

    // TODO: parametrize
    std::string startSolutionPath = inputFilePath + "../StartSolutions/Zhang/ConvertedSolutions/";
    inputParameters.MIPSolver.Seed += seedOffset;

    for (int i = 0; i < 1; ++i)
    {
        try
        {
            std::cout << "Run: " << i << "\n";
            GRBEnv env = GRBEnv(outputPath + "/" + instance.Name + ".LOG");
            inputParameters.MIPSolver.Seed += i;
            BranchAndCutSolver exactAlgorithm(&instance, &env, inputParameters, startSolutionPath, outputPath);
            exactAlgorithm.Solve();
        }
        catch (GRBException& e)
        {
            std::cout << e.getMessage();
        }
    }
}

int main(int argc, char** argv)
{
    // For example, call with: -i "data/3LVRP/ConvertedInstances/" -f "E016-05m.json" -o "data/3LVRP/Output/"
    CLI::App app;

    std::string inputFilePath = "default";
    std::string filename = "default";
    std::string outdir = "default";
    std::string parameterFile;
    bool enableTimeSuffix = true;
    int seedOffset = 0;

    app.add_option("-i,--inputdir", inputFilePath, "The directory where the input file -f resides")->required();
    app.add_option("-f,--file", filename, "The input file name")->required();
    app.add_option("-o,--outdir", outdir, "The output directory")->required();
    app.add_option("-p,--param", parameterFile, "The .json parameter full file path");
    app.add_option("-t,--timeSuffix",
                   enableTimeSuffix,
                   "If the current time should be appended to the output path as a subfolder (1=true, 0=false)");
    app.add_option("-s,--seedOffset", seedOffset, "The offset to the internal seed");

    CLI11_PARSE(app, argc, argv);

    std::string inputFilePathDelimiter = inputFilePath.substr(inputFilePath.size() - 1, inputFilePath.size());
    std::string inputFileSuffix = filename.substr(filename.size() - 5, filename.size());
    std::string outdirDelimiter = outdir.substr(outdir.size() - 1, outdir.size());

    if (inputFilePathDelimiter != "/" && inputFilePathDelimiter != "\\")
    {
        throw CLI::ConversionError("-i directory path delimiter does neither math '/' nor '\\'");
    }

    if (inputFileSuffix != ".json")
    {
        throw CLI::ConversionError("-f does not have an .json suffix");
    }

    if (outdirDelimiter != "/" && outdirDelimiter != "\\")
    {
        throw CLI::ConversionError("-o directory path delimiter does neither math '/' nor '\\'");
    }
    try
    {
        Run(inputFilePath, filename, parameterFile, outdir, enableTimeSuffix, seedOffset);
        return EXIT_SUCCESS;
    }
    catch (std::exception& e)
    {
        std::cerr << "Exception: " << e.what() << "\n";
        return EXIT_FAILURE;
    }
}

// Run program: Ctrl + F5 or Debug > Start Without Debugging menu
// Debug program: F5 or Debug > Start Debugging menu

// Tips for Getting Started:
//   1. Use the Solution Explorer window to add/manage files
//   2. Use the Team Explorer window to connect to source control
//   3. Use the Output window to see build output and other messages
//   4. Use the Error List window to view errors
//   5. Go to Project > Add New Item to create new code files, or Project > Add Existing Item to add existing code files
//   to the project
//   6. In the future, to open this project again, go to File > Open > Project and select the .sln file
