#pragma once

#include "CommonBasics/Helper/MIPServices.h"

#include "Model/Instance.h"
#include "Model/Solution.h"

namespace VehicleRouting
{
using namespace Model;

namespace Algorithms
{
class TwoIndexVehicleFlow
{
  public:
    TwoIndexVehicleFlow(Instance* instance, GRBEnv* env) : mInstance(instance), mEnv(env) {}

    void BuildModel(const std::vector<Arc>& startSolutionArcs,
                    const std::vector<Arc>& infeasibleArcs,
                    const std::vector<Arc>& infeasibleTailPaths);
    void SetCallback(GRBCallback* callback);
    void Solve(const MIPSolverParams& parameters);

    GRBVar2D* GetXVariables() { return &mVariablesX; };
    double GetRuntime() { return mModel->get(GRB_DoubleAttr_Runtime); }
    double GetMIPGap() { return mModel->get(GRB_DoubleAttr_MIPGap); }
    double GetNodeCount() { return mModel->get(GRB_DoubleAttr_NodeCount); }
    double GetSimIterCount() { return mModel->get(GRB_DoubleAttr_IterCount); }

    std::optional<Solution> GetSolution();

  private:
    const Instance* const mInstance;
    GRBEnv* mEnv;
    std::unique_ptr<GRBModel> mModel = nullptr;

    GRBVar2D mVariablesX;

    void AddVariables();
    void AddConstraints();
    void AddObjective();
    void SetStartSolution(const std::vector<Arc>& startSolutionArcs);
    void SetInfeasibleArcs(const std::vector<Arc>& infeasibleArcs, const std::vector<Arc>& infeasibleTailPaths);
    void SetParameters(const MIPSolverParams& inputParameters);
};

}
}