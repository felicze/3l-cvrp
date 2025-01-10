#include "Algorithms/MultiContainer/BP_MIP_1D.h"

namespace ContainerLoading
{
using namespace Model;
namespace Algorithms
{
void BinPacking1D::BuildModel()
{
    CreateVariables();

    AddConstraints();
}

void BinPacking1D::AddConstraints()
{
    for (size_t iGroup = 0; iGroup < mNumberGroups; ++iGroup)
    {
        GRBLinExpr allAssigned = 0;
        for (size_t iContainer = 0; iContainer < mNumberContainers; ++iContainer)
        {
            allAssigned += mVariablesX[iGroup][iContainer];
        }

        mAssignmentConstraints.emplace_back(mModel->addConstr(allAssigned == 1));
    }

    for (size_t iContainer = 0; iContainer < mNumberContainers; ++iContainer)
    {
        const auto& container = mContainers[iContainer];
        GRBLinExpr weight = 0;
        GRBLinExpr volume = 0;
        for (size_t iGroup = 0; iGroup < mNumberGroups; ++iGroup)
        {
            weight += mGroups[iGroup].TotalWeight * mVariablesX[iGroup][iContainer];
            volume += mGroups[iGroup].TotalVolume * mVariablesX[iGroup][iContainer];
        }

        mModel->addConstr(weight <= container.WeightLimit * mVariablesY[iContainer]);
        mModel->addConstr(volume <= container.Volume * mVariablesY[iContainer]);
    }

    for (size_t iContainer = 0; iContainer < mNumberContainers - 1; ++iContainer)
    {
        mModel->addConstr(mVariablesY[iContainer] >= mVariablesY[iContainer + 1]);
    }
}

void BinPacking1D::CreateVariables()
{
    mVariablesX = GRBVar2D();
    for (size_t i = 0; i < mNumberGroups; ++i)
    {
        mVariablesX.emplace_back();
        for (size_t iContainer = 0; iContainer < mNumberContainers; ++iContainer)
        {
            std::string name = "x_" + std::to_string(i) + "_" + std::to_string(iContainer);
            mVariablesX[i].emplace_back(mModel->addVar(0, 1, 0, GRB_BINARY, name));
        }
    }

    mVariablesY = GRBVar1D();
    for (size_t i = 0; i < mNumberContainers; ++i)
    {
        std::string name = "y_" + std::to_string(i);
        mVariablesY.emplace_back(mModel->addVar(0, 1, 1, GRB_BINARY, name));
    }
}

void BinPacking1D::SetParameters()
{
    mModel->set(GRB_IntParam_OutputFlag, 0);
    mModel->set(GRB_IntParam_Threads, 1);

    mModel->set(GRB_IntParam_Seed, 1008);
    mModel->set(GRB_DoubleParam_TimeLimit, 12 * 3600);
    ////mModel.set(GRB_IntParam_SolutionLimit, 1);
}

int BinPacking1D::Solve()
{
    BuildModel();

    SetParameters();

    mModel->optimize();

    auto status = mModel->get(GRB_IntAttr_Status);

    return status == GRB_OPTIMAL ? static_cast<int>(mModel->get(GRB_DoubleAttr_ObjVal))
                                 : static_cast<int>(mModel->get(GRB_DoubleAttr_ObjBound));
}

int BinPacking1D::ReSolve(const boost::dynamic_bitset<>& selectedGroups)
{
    for (size_t i = 0; i < mNumberGroups; ++i)
    {
        if (selectedGroups[i + 1])
        {
            mAssignmentConstraints[i].set(GRB_DoubleAttr_RHS, 1.0);
        }
        else
        {
            mAssignmentConstraints[i].set(GRB_DoubleAttr_RHS, 0.0);
        }
    }

    mModel->set(GRB_DoubleParam_TimeLimit, 10);

    mModel->optimize();

    auto status = mModel->get(GRB_IntAttr_Status);

    return status == GRB_OPTIMAL ? static_cast<int>(mModel->get(GRB_DoubleAttr_ObjVal))
                                 : static_cast<int>(mModel->get(GRB_DoubleAttr_ObjBound));
}

}
}