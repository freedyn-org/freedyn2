#include "freedyn2/scatter.hpp"

#include <algorithm>
#include <stdexcept>

namespace fd {

int MatrixAssembler::add_block(const Sparsity& sp, const std::vector<int>& map) {
  if (sp.nrow != static_cast<int>(map.size()) || sp.ncol != static_cast<int>(map.size()))
    throw std::runtime_error("MatrixAssembler: block size does not match its index map");
  blocks_.push_back({sp, map, {}});
  return static_cast<int>(blocks_.size()) - 1;
}

void MatrixAssembler::finalize(int n) {
  // Collect the global column indices of each row.
  std::vector<std::vector<int>> rows(n);
  for (const Block& b : blocks_)
    for (int j = 0; j < b.sp.ncol; ++j)
      for (int k = b.sp.colind[j]; k < b.sp.colind[j + 1]; ++k)
        rows.at(b.map[b.sp.row[k]]).push_back(b.map[j]);

  A_.n = n;
  A_.rowptr.assign(n + 1, 0);
  A_.col.clear();
  for (int i = 0; i < n; ++i) {
    auto& r = rows[i];
    std::sort(r.begin(), r.end());
    r.erase(std::unique(r.begin(), r.end()), r.end());
    A_.col.insert(A_.col.end(), r.begin(), r.end());
    A_.rowptr[i + 1] = static_cast<int>(A_.col.size());
  }
  A_.val.assign(A_.col.size(), 0.0);

  // Scatter map: local nonzero -> CSR value index.
  for (Block& b : blocks_) {
    b.pos.resize(b.sp.nnz());
    for (int j = 0; j < b.sp.ncol; ++j)
      for (int k = b.sp.colind[j]; k < b.sp.colind[j + 1]; ++k) {
        const int gi = b.map[b.sp.row[k]], gj = b.map[j];
        auto first = A_.col.begin() + A_.rowptr[gi], last = A_.col.begin() + A_.rowptr[gi + 1];
        b.pos[k] = static_cast<int>(std::lower_bound(first, last, gj) - A_.col.begin());
      }
  }
}

void MatrixAssembler::zero() { std::fill(A_.val.begin(), A_.val.end(), 0.0); }

void MatrixAssembler::add(int block, const double* local_nz) {
  const Block& b = blocks_[block];
  for (size_t k = 0; k < b.pos.size(); ++k) A_.val[b.pos[k]] += local_nz[k];
}

std::vector<int> vector_scatter_map(const Sparsity& sp, const std::vector<int>& map) {
  if (sp.ncol != 1 || sp.nrow != static_cast<int>(map.size()))
    throw std::runtime_error("vector_scatter_map: output is not a local column vector");
  std::vector<int> rows(sp.nnz());
  for (int k = 0; k < sp.nnz(); ++k) rows[k] = map[sp.row[k]];
  return rows;
}

}  // namespace fd
