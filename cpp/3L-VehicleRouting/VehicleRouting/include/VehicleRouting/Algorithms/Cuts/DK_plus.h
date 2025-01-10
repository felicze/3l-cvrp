#pragma once

/// Source Code is adopted from the supplement material given by
/// Michiel A. J. uit het Broek, Albert H. Schrotenboer, Bolor Jargalsaikhan, Kees Jan Roodbergen, Leandro C. Coelho
/// (2021) Asymmetric Multidepot Vehicle Routing Problems: Valid Inequalities and a Branch-and-Cut Algorithm. Operations
/// Research 69(2):380-409. https://doi.org/10.1287/opre.2020.2033

#include "BaseCut.h"

namespace VehicleRouting::Algorithms::Cuts
{
class DK_plus final : public BaseCut
{
  public:
    explicit DK_plus(const Algorithms::InputParameters* inputParameters) : BaseCut(CutType::DKplus, inputParameters){};

  private:
    double mMaxPhi = 0;
    // Virtual function
    std::vector<Cut> FindCuts(const std::vector<std::vector<double>>& x) final;

    // Helper functions

    /// <summary>
    /// Tree search in one weakly connected component to identify all violated DK_plus node sets.
    /// </summary>
    void DepthFirstSearch(const Collections::IdVector& iComponent,
                          const std::vector<std::vector<double>>& x,
                          std::vector<Cut>& cuts);

    [[nodiscard]] std::optional<Cut>
        CreateCut(const Collections::IdVector& sequence, size_t n, const std::vector<std::vector<double>>& x) const;

    static double UpdatePhi(const std::vector<std::vector<double>>& x,
                            const Collections::IdVector& sequence,
                            size_t n,
                            double phi);

    static double
        UpdatePi(const std::vector<std::vector<double>>& x, const Collections::IdVector& sequence, size_t n, double pi);

    void Extend(size_t n,
                double phi,
                double pi,
                const std::vector<std::vector<double>>& x,
                Collections::IdVector& sequence,
                const Collections::IdVector& iComponent,
                std::vector<Cut>& DKcuts);

    static bool AlreadyInSequence(const Collections::IdVector& seq, size_t n, size_t k);
};

inline bool DK_plus::AlreadyInSequence(const Collections::IdVector& sequence, const size_t n, const size_t k)
{
    return (find(cbegin(sequence), cbegin(sequence) + static_cast<int>(n), k)
            != cbegin(sequence) + static_cast<int>(n));
}

}
