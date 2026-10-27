#pragma once
// Thin C++ wrapper around one CasADi-generated C function (no CasADi library needed).

#include <string>
#include <vector>

#include "fd_registry.h"

namespace fd {

// Compressed column storage pattern, as CasADi defines it.
struct Sparsity {
  int nrow = 0, ncol = 0;
  std::vector<int> colind;  // size ncol + 1
  std::vector<int> row;     // size nnz
  int nnz() const { return static_cast<int>(row.size()); }
  bool dense() const { return nnz() == nrow * ncol; }
};

class GenFunction {
 public:
  explicit GenFunction(const fd_function& f);

  const char* name() const { return f_->name; }
  int n_in() const { return n_in_; }
  int n_out() const { return n_out_; }
  const Sparsity& sparsity_in(int i) const { return sp_in_[i]; }
  const Sparsity& sparsity_out(int i) const { return sp_out_[i]; }
  std::string name_out(int i) const { return f_->name_out(i); }

  // in[i]: nonzeros of input i (nullptr means all zero).
  // out[i]: buffer for the nonzeros of output i (nullptr skips that output).
  void eval(const double* const* in, double* const* out);

 private:
  const fd_function* f_;
  int n_in_, n_out_;
  std::vector<Sparsity> sp_in_, sp_out_;
  std::vector<const casadi_real*> arg_;
  std::vector<casadi_real*> res_;
  std::vector<casadi_int> iw_;
  std::vector<casadi_real> w_;
};

}  // namespace fd
