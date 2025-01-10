#pragma once

#include "CommonBasics/Helper/MIPServices.h"
#include "Model/Container.h"
#include "Model/ContainerLoadingInstance.h"

#include <boost/dynamic_bitset/dynamic_bitset.hpp>

namespace ContainerLoading
{
using namespace Model;
namespace Algorithms
{
class BinPacking1D
{
  public:
    void BuildModel();
    int Solve();
    int ReSolve(const boost::dynamic_bitset<>& selectedGroups);

    GRBVar2D& GetVariablesX() { return mVariablesX; }

    void SetContainer(const std::vector<Container>& containers)
    {
        mContainers = containers;
        mNumberContainers = containers.size();
    }

    void SetGroups(const std::vector<Group>& groups)
    {
        mGroups = groups;
        mNumberGroups = groups.size();
    }

    BinPacking1D(GRBEnv* env,
                 const std::vector<Container>& containers,
                 const std::vector<Group>& groups,
                 const std::string& outputPath = "")
    : mEnv(env),
      mNumberContainers(containers.size()),
      mNumberGroups(groups.size()),
      mContainers(containers),
      mGroups(groups)
    {
        mModel = std::make_unique<GRBModel>(*mEnv);
        mModel->set(GRB_StringParam_LogFile, outputPath + "BinPackLowerBound.log");
    }

    BinPacking1D& operator=(BinPacking1D&& bp) noexcept
    {
        if (this != &bp)
        {
            mEnv = bp.mEnv;
            mContainers = bp.mContainers;
            mGroups = bp.mGroups;
            mNumberContainers = bp.mNumberContainers;
            mNumberGroups = bp.mNumberGroups;
            mModel = std::move(bp.mModel);
        }

        return *this;
    }

    BinPacking1D(BinPacking1D&& bp) noexcept : mModel(std::move(bp.mModel)) {}

  private:
    GRBEnv* mEnv;

    size_t mNumberContainers = 0;
    size_t mNumberGroups = 0;

    std::vector<Container> mContainers;
    std::vector<Group> mGroups;

    std::unique_ptr<GRBModel> mModel = nullptr;

    std::vector<GRBConstr> mAssignmentConstraints;

    GRBVar2D mVariablesX;
    GRBVar1D mVariablesY;

    void AddConstraints();
    void CreateVariables();
    void SetParameters();
    void CreateSolution();
};

}
}