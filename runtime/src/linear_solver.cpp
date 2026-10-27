#include "freedyn2/linear_solver.hpp"

#include <Eigen/Dense>
#include <Eigen/SparseCore>
#include <Eigen/SparseLU>
#include <algorithm>
#include <cmath>
#include <stdexcept>

#ifdef FD_WITH_KLU
#include <klu.h>
#endif

namespace fd {

namespace {

using CsrMap = Eigen::Map<const Eigen::SparseMatrix<double, Eigen::RowMajor, int>>;

CsrMap as_eigen(const CsrMatrix& A) {
  return CsrMap(A.n, A.n, A.nnz(), A.rowptr.data(), A.col.data(), A.val.data());
}

class EigenSparseLU : public LinearSolver {
 public:
  void factorize(const CsrMatrix& A) override {
    A_ = as_eigen(A);  // to column major, as SparseLU expects
    if (!analyzed_) {
      lu_.analyzePattern(A_);
      analyzed_ = true;
    }
    lu_.factorize(A_);
    if (lu_.info() != Eigen::Success) throw std::runtime_error("SparseLU: factorization failed");
  }
  void solve(const double* b, double* x) override {
    Eigen::Map<Eigen::VectorXd>(x, A_.rows()) = lu_.solve(Eigen::Map<const Eigen::VectorXd>(b, A_.rows()));
  }

 private:
  Eigen::SparseMatrix<double> A_;
  Eigen::SparseLU<Eigen::SparseMatrix<double>, Eigen::COLAMDOrdering<int>> lu_;
  bool analyzed_ = false;
};

class DenseLU : public LinearSolver {
 public:
  void factorize(const CsrMatrix& A) override {
    Eigen::MatrixXd D = Eigen::MatrixXd(as_eigen(A));
    lu_.compute(D);
    if (!(std::abs(lu_.determinant()) > 0)) throw std::runtime_error("DenseLU: singular matrix");
  }
  void solve(const double* b, double* x) override {
    const auto n = lu_.rows();
    Eigen::Map<Eigen::VectorXd>(x, n) = lu_.solve(Eigen::Map<const Eigen::VectorXd>(b, n));
  }

 private:
  Eigen::PartialPivLU<Eigen::MatrixXd> lu_;
};

#ifdef FD_WITH_KLU
// SuiteSparse KLU, made for small, very sparse unsymmetric systems: symbolic analysis
// (block triangular form, AMD ordering) once, then numerical refactorization with the
// pivot order of the last full factorization; a full factorization with new pivots when
// the refactorization fails or the reciprocal pivot growth estimate degrades.
// The CSR arrays of A are the CSC arrays of A^T, so A^T is factorized and klu_tsolve
// solves A x = b.
class Klu : public LinearSolver {
 public:
  Klu() { klu_defaults(&common_); }
  ~Klu() override {
    if (numeric_) klu_free_numeric(&numeric_, &common_);
    if (symbolic_) klu_free_symbolic(&symbolic_, &common_);
  }
  void factorize(const CsrMatrix& A) override {
    n_ = A.n;
    // KLU wants non-const pointers; it does not modify the pattern or the values.
    auto* ap = const_cast<int*>(A.rowptr.data());
    auto* ai = const_cast<int*>(A.col.data());
    auto* ax = const_cast<double*>(A.val.data());
    if (!symbolic_) {
      symbolic_ = klu_analyze(n_, ap, ai, &common_);
      if (!symbolic_) throw std::runtime_error("KLU: analysis failed");
    }
    if (numeric_ && klu_refactor(ap, ai, ax, symbolic_, numeric_, &common_) &&
        (fixed_pivots_ || (klu_rcond(symbolic_, numeric_, &common_) && common_.rcond > 1e-3 * rcond_full_)))
      return;
    if (numeric_) klu_free_numeric(&numeric_, &common_);
    numeric_ = klu_factor(ap, ai, ax, symbolic_, &common_);
    ++full_;
    if (!numeric_ || common_.status == KLU_SINGULAR) throw std::runtime_error("KLU: singular matrix");
    klu_rcond(symbolic_, numeric_, &common_);
    rcond_full_ = common_.rcond;
  }
  void solve(const double* b, double* x) override {
    std::copy(b, b + n_, x);
    if (!klu_tsolve(symbolic_, numeric_, n_, 1, x, &common_)) throw std::runtime_error("KLU: solve failed");
  }
  long full_factorizations() const override { return full_; }
  void set_fixed_pivots(bool fixed) override { fixed_pivots_ = fixed; }

 private:
  klu_common common_;
  klu_symbolic* symbolic_ = nullptr;
  klu_numeric* numeric_ = nullptr;
  double rcond_full_ = 0.0;
  int n_ = 0;
  long full_ = 0;
  bool fixed_pivots_ = false;
};
#endif

}  // namespace

const char* default_linear_solver() {
#ifdef FD_WITH_KLU
  return "klu";
#else
  return "sparselu";
#endif
}

std::unique_ptr<LinearSolver> make_linear_solver(const std::string& name) {
#ifdef FD_WITH_KLU
  if (name == "klu") return std::make_unique<Klu>();
#endif
  if (name == "sparselu") return std::make_unique<EigenSparseLU>();
  if (name == "dense") return std::make_unique<DenseLU>();
  throw std::runtime_error("unknown linear solver '" + name + "'");
}

}  // namespace fd
