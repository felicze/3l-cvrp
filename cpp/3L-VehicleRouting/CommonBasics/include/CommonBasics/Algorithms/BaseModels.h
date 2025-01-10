#pragma once

#include "CommonBasics/Helper/MIPServices.h"
#include "CommonBasics/Helper/ModelServices.h"

#include <memory>
#include <unordered_set>
#include <utility>

namespace BaseModels
{
template <typename TContainer> class SetProblems
{
  public:
    SetProblems(GRBEnv* env,
                bool relaxation,
                const TContainer& columns,
                Collections::IdVector rows,
                std::vector<double> costs,
                size_t maxCols)
    : mRelaxation(relaxation), mColumns(columns), mRows(std::move(rows)), mCosts(std::move(costs)), mMaxColumns(maxCols)
    {
        mModel = std::make_unique<GRBModel>(*env);
    };

   [[nodiscard]] bool Solve()
    {
        if (!BuildModel())
        {
            return false;
        }
        mModel->set(GRB_IntParam_OutputFlag, 0);
        mModel->set(GRB_IntParam_Threads, 1);

        mModel->optimize();

        return mModel->get(GRB_IntAttr_SolCount) > 0;
    }

    [[nodiscard]] double GetObjFuncValue() const { return mModel->get(GRB_DoubleAttr_ObjVal); };

    Collections::SequenceVector GetSelectedColumns()
    {
        double cutoff = mRelaxation ? 0.001 : 0.5;

        auto selectedRoutes = Collections::SequenceVector();
        int c = 0;
        for (const auto& column: mColumns)
        {
            if (mXVariables[c].get(GRB_DoubleAttr_X) > cutoff)
            {
                selectedRoutes.push_back(column);
            }

            c++;
        }

        return selectedRoutes;
    }

  protected:
    size_t mMaxColumns;
    TContainer mColumns;
    Collections::IdVector mRows;
    std::vector<double> mCosts;
    std::unique_ptr<GRBModel> mModel = nullptr;
    GRBVar1D mXVariables;
    bool mRelaxation;

    std::vector<GRBConstr> mCoveringConstraints;

    [[nodiscard]] bool BuildModel()
    {
        AddVariables();
        return AddConstraints();
    };

    void AddVariables()
    {
        auto variableType = mRelaxation ? GRB_CONTINUOUS : GRB_INTEGER;

        for (const auto mCost : mCosts)
        {
            mXVariables.emplace_back(mModel->addVar(0, GRB_INFINITY, mCost, variableType));
        }
    };

    virtual bool AddConstraints() = 0;
};

template <typename TContainer> class SetPartitioning : public SetProblems<TContainer>
{
  public:
    SetPartitioning(GRBEnv* env,
                    bool relaxation,
                    const TContainer& columns,
                    Collections::IdVector rows,
                    std::vector<double> costs,
                    size_t maxCols)
    : SetProblems<TContainer>(env, relaxation, columns, rows, costs, maxCols){};

  private:
    [[nodiscard]] bool AddConstraints()
    {
        for (const auto& row: this->mRows)
        {
            GRBLinExpr sumSets = 0;
            int iCol = 0;
            for (const auto& column: this->mColumns)
            {
                if (std::find(std::begin(column), std::end(column), row) != std::end(column))
                {
                    sumSets += this->mXVariables[iCol];
                }

                iCol++;
            }

            if (sumSets.size() == 0)
            {
                return false;
            }

            this->mCoveringConstraints.emplace_back(this->mModel->addConstr(sumSets == 1));
        }

        int cColumn = 0;
        GRBLinExpr sumCols = 0;
        for (const auto& column: this->mColumns)
        {
            sumCols += this->mXVariables[cColumn];
            cColumn++;
        }

        this->mModel->addConstr(sumCols <= this->mMaxColumns);

        return true;
    }
};

template <typename TContainer> class SetCovering : public SetProblems<TContainer>
{
  public:
    SetCovering(GRBEnv* env,
                bool relaxation,
                const TContainer& columns,
                Collections::IdVector rows,
                std::vector<double> costs,
                size_t maxCols)
    : SetProblems<TContainer>(env, relaxation, columns, rows, costs, maxCols){};

  private:
    [[nodiscard]] bool AddConstraints()
    {
        for (const auto& row: this->mRows)
        {
            GRBLinExpr sumSets = 0;
            int iCol = 0;
            for (const auto& column: this->mColumns)
            {
                if (std::find(std::begin(column), std::end(column), row) != std::end(column))
                {
                    sumSets += this->mXVariables[iCol];
                }

                iCol++;
            }

            if (sumSets.size() == 0)
            {
                return false;
            }

            this->mCoveringConstraints.emplace_back(this->mModel->addConstr(sumSets >= 1));
        }

        int cColumn = 0;
        GRBLinExpr sumCols = 0;
        for (const auto& column: this->mColumns)
        {
            sumCols += this->mXVariables[cColumn];
            cColumn++;
        }

        this->mModel->addConstr(sumCols <= this->mMaxColumns);

        return true;
    }
};
}