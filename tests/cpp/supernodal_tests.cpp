// The supernodal LDL^T of freedyn2_modal (runtime/app/supernodal.hpp) against Eigen's
// SimplicialLDLT: solutions and inertia (negative pivots) of a 3D grid stiffness K - sigma M with
// sigma below and above some eigenvalues, and a zero pivot reported. Exit status 1 on a failure.

#include <Eigen/SparseCholesky>

#include <cstdio>
#include <vector>

#include "supernodal.hpp"

using Sp = Eigen::SparseMatrix<double>;

namespace {

// 7-point Laplacian of an n x n x n grid (K) and a lumped mass with some variation (M)
void grid(int n, Sp& K, Sp& M) {
  const int N = n * n * n;
  auto id = [n](int i, int j, int k) { return i + n * (j + n * k); };
  std::vector<Eigen::Triplet<double>> tk, tm;
  for (int k = 0; k < n; ++k)
    for (int j = 0; j < n; ++j)
      for (int i = 0; i < n; ++i) {
        const int p = id(i, j, k);
        tk.emplace_back(p, p, 6.0 + 0.01 * (p % 7));
        tm.emplace_back(p, p, 1.0 + 0.1 * (p % 5));
        const int nb[3][3] = {{i + 1, j, k}, {i, j + 1, k}, {i, j, k + 1}};
        for (const auto& q : nb)
          if (q[0] < n && q[1] < n && q[2] < n) {
            tk.emplace_back(p, id(q[0], q[1], q[2]), -1.0);
            tk.emplace_back(id(q[0], q[1], q[2]), p, -1.0);
          }
      }
  K.resize(N, N), M.resize(N, N);
  K.setFromTriplets(tk.begin(), tk.end()), M.setFromTriplets(tm.begin(), tm.end());
}

int failures = 0;

void check(bool ok, const char* what, double value) {
  std::printf("%-44s %10.3g  %s\n", what, value, ok ? "ok" : "FAIL");
  if (!ok) ++failures;
}

}  // namespace

int main() {
  Sp K, M;
  grid(24, K, M);  // 13 824 unknowns: fronts large enough for the blocked and threaded paths
  fd::SupernodalLDLT f;
  {
    Sp A = K;
    A.makeCompressed();
    f.analyze(A);
  }
  const Eigen::VectorXd b = Eigen::VectorXd::LinSpaced(K.rows(), -1.0, 2.0);
  for (double sigma : {0.0, 0.3, 1.1}) {
    Sp A = K - sigma * M;
    A.makeCompressed();
    const bool ok = f.factorize(A);
    check(ok, "factorize", sigma);
    Eigen::SimplicialLDLT<Sp> ref(A);
    const Eigen::VectorXd x = f.solve(b), xr = ref.solve(b);
    const long neg_ref = (ref.vectorD().array() < 0).count();
    check((A * x - b).norm() / b.norm() < 1e-10, "residual", (A * x - b).norm() / b.norm());
    check((x - xr).norm() / xr.norm() < 1e-8, "against SimplicialLDLT", (x - xr).norm() / xr.norm());
    check(f.negatives() == neg_ref, "negative pivots (inertia)", f.negatives());
  }
  {
    Sp Z(3, 3);  // a zero pivot without pivoting
    std::vector<Eigen::Triplet<double>> t{{0, 1, 1.0}, {1, 0, 1.0}, {2, 2, 1.0}};
    Z.setFromTriplets(t.begin(), t.end());
    Z.makeCompressed();
    fd::SupernodalLDLT g;
    g.analyze(Z);
    check(!g.factorize(Z), "zero pivot reported", 0.0);
  }
  return failures ? 1 : 0;
}
