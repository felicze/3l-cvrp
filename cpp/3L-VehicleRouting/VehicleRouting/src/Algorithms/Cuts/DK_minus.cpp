#include "Algorithms/Cuts/DK_minus.h"

namespace VehicleRouting::Algorithms::Cuts
{
std::vector<Cut> DK_min::FindCuts(std::vector<std::vector<double>> const& x)
{
    std::vector<Cut> cuts;

    // Get all weakly connected components in support graph.
    for (auto const components = GraphFunctions::GetConnectedComponents(x); auto const& iComponent: components)
    {
        if (iComponent.size() == 1)
        {
            continue;
        }

        DepthFirstSearch(iComponent, x, cuts);
    }

    return cuts;
}

/// Separation procedure adapted from Fischetti & Toth (1997): https://doi.org/10.1287/mnsc.43.11.1520
void DK_min::DepthFirstSearch(const Collections::IdVector& iComponent,
                              const std::vector<std::vector<double>>& x,
                              std::vector<Cut>& cuts)
{
    Collections::IdVector sequence(iComponent.size(), 0);

    for (auto const idx: iComponent)
    {
        mMaxPhi = 0;
        sequence[0] = idx;
        Extend(1, 0, 0, x, sequence, iComponent, cuts);
    }
}

std::optional<Cut> DK_min::CreateCut(const Collections::IdVector& sequence,
                                     const size_t n,
                                     const std::vector<std::vector<double>>& x) const
{
    auto const i = sequence[0];
    auto const k = sequence[n - 1];

    auto cut = Cut(CutType::DKminus);
    cut.RHS = -(static_cast<int>(n) - 1);

    cut.AddArc(-1.0, k, i, x[k][i]);

    for (size_t h = 1; h < n; ++h)
    {
        const auto nodeI = sequence[h - 1];
        const auto nodeJ = sequence[h];
        cut.AddArc(-1.0, nodeI, nodeJ, x[nodeI][nodeJ]);
    }

    for (size_t h = 1; h < n - 1; ++h)
    {
        const auto nodeJ = sequence[h];
        cut.AddArc(-2.0, nodeJ, i, x[nodeJ][i]);
    }

    for (size_t h = 1; h < n - 2; ++h)
    {
        const auto nodeJ = sequence[h];
        for (size_t h2 = h + 1; h2 < n - 1; ++h2)
        {
            const auto nodeI = sequence[h2];
            cut.AddArc(-1.0, nodeI, nodeJ, x[nodeI][nodeJ]);
        }
    }

    cut.CalcViolation();
    if (cut.Violation <= InputParameters->UserCut.ViolationThreshold.at(Type))
    {
        return std::nullopt;
    }

    return cut;
}

double DK_min::UpdatePhi(const std::vector<std::vector<double>>& x,
                         const Collections::IdVector& sequence,
                         const size_t n,
                         const double phi)
{
    double tmpPhi = phi + x[sequence[n]][sequence[0]] + x[sequence[n - 1]][sequence[n]] - 1;

    for (size_t h = 0; h < n - 1; ++h)
    {
        tmpPhi += x[sequence[n - 1]][sequence[h]];
    }

    return tmpPhi;
}

double DK_min::UpdatePi(const std::vector<std::vector<double>>& x,
                        const Collections::IdVector& sequence,
                        const size_t n,
                        const double pi)
{
    double tmpPi = pi + x[sequence[n - 1]][sequence[n]] - 1;

    for (size_t h = 0; h < n; ++h)
    {
        tmpPi += x[sequence[n]][sequence[h]];
    }

    return tmpPi;
}

void DK_min::Extend(const size_t n,
                    const double phi,
                    const double pi,
                    const std::vector<std::vector<double>>& x,
                    Collections::IdVector& sequence,
                    const Collections::IdVector& iComponent,
                    std::vector<Cut>& DKcuts)
{
    if (n == iComponent.size())
    {
        return;
    }

    auto const j = sequence[n - 1];

    for (const auto k: iComponent)
    {
        // Only consider promising children. See equation 10 of paper Fischetti (1997)
        if (x[k][j] <= mMaxPhi - pi)
        {
            continue;
        }

        if (AlreadyInSequence(sequence, n, k))
        {
            continue;
        }

        sequence[n] = k;

        double const tmpPi = UpdatePi(x, sequence, n, pi);
        double const tmpPhi = UpdatePhi(x, sequence, n, phi);

        if (tmpPhi > mMaxPhi + InputParameters->UserCut.EpsForIntegrality)
        {
            mMaxPhi = tmpPhi;

            if (auto cut = CreateCut(sequence, n + 1, x))
            {
                DKcuts.emplace_back(std::move(cut.value()));
            }
        }

        Extend(n + 1, tmpPhi, tmpPi, x, sequence, iComponent, DKcuts);
    }
}

}
