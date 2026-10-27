#pragma once
// Linear solver interface. The sparsity pattern of A is fixed after the first
// factorize() call; later calls only refactorize numerically. MKL Pardiso is
// meant to be plugged in behind this interface later.

#include <memory>
#include <string>

#include "freedyn2/scatter.hpp"

namespace fd {

class LinearSolver {
 public:
  virtual ~LinearSolver() = default;
  virtual void factorize(const CsrMatrix& A) = 0;  // throws if singular
  virtual void solve(const double* b, double* x) = 0;
  // Factorizations that chose a new pivot order (KLU: not a refactorization with the old one).
  virtual long full_factorizations() const { return 0; }
  // fixed: keep the pivot order of the first factorization as long as it gives a nonsingular
  // factorization (same work in every call; KLU: no new pivots when the pivot quality
  // estimate degrades). Solvers that pivot in every factorization ignore it.
  virtual void set_fixed_pivots(bool fixed) { (void)fixed; }
};

// "klu" (SuiteSparse KLU, if built with it), "sparselu" (Eigen::SparseLU) or "dense"
// (Eigen::PartialPivLU).
std::unique_ptr<LinearSolver> make_linear_solver(const std::string& name);

// "klu" if available, else "sparselu".
const char* default_linear_solver();

}  // namespace fd
