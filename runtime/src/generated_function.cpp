#include "freedyn2/generated_function.hpp"

#include <stdexcept>

namespace fd {

namespace {
Sparsity to_sparsity(const casadi_int* s) {
  Sparsity sp;
  sp.nrow = static_cast<int>(s[0]);
  sp.ncol = static_cast<int>(s[1]);
  if (s[2] == 1) {
    // Compact dense form {nrow, ncol, 1}: a full pattern always has colind[0] == 0.
    for (int j = 0; j <= sp.ncol; ++j) sp.colind.push_back(j * sp.nrow);
    for (int j = 0; j < sp.ncol; ++j)
      for (int i = 0; i < sp.nrow; ++i) sp.row.push_back(i);
    return sp;
  }
  const casadi_int* colind = s + 2;
  sp.colind.assign(colind, colind + sp.ncol + 1);
  sp.row.assign(colind + sp.ncol + 1, colind + sp.ncol + 1 + colind[sp.ncol]);
  return sp;
}
}  // namespace

GenFunction::GenFunction(const fd_function& f) : f_(&f) {
  n_in_ = static_cast<int>(f.n_in());
  n_out_ = static_cast<int>(f.n_out());
  for (int i = 0; i < n_in_; ++i) {
    sp_in_.push_back(to_sparsity(f.sparsity_in(i)));
    if (!sp_in_.back().dense())
      throw std::runtime_error(std::string(f.name) + ": sparse inputs are not supported");
  }
  for (int i = 0; i < n_out_; ++i) sp_out_.push_back(to_sparsity(f.sparsity_out(i)));
  casadi_int sz_arg, sz_res, sz_iw, sz_w;
  f.work(&sz_arg, &sz_res, &sz_iw, &sz_w);
  arg_.assign(sz_arg, nullptr);
  res_.assign(sz_res, nullptr);
  iw_.resize(sz_iw);
  w_.resize(sz_w);
}

void GenFunction::eval(const double* const* in, double* const* out) {
  for (int i = 0; i < n_in_; ++i) arg_[i] = in[i];
  for (int i = 0; i < n_out_; ++i) res_[i] = out[i];
  if (f_->eval(arg_.data(), res_.data(), iw_.data(), w_.data(), 0) != 0)
    throw std::runtime_error(std::string(f_->name) + ": evaluation failed");
}

}  // namespace fd
