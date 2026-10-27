#pragma once
// Scatter assembly: the global CSR pattern and, for every local nonzero of every
// element block, its position in the CSR value array are computed once at setup.
// Per update only the value array is zeroed and local values are added via that map.

#include <vector>

#include "freedyn2/generated_function.hpp"

namespace fd {

struct CsrMatrix {
  int n = 0;                // square, n x n
  std::vector<int> rowptr;  // size n + 1
  std::vector<int> col;     // size nnz, sorted within each row
  std::vector<double> val;  // size nnz
  int nnz() const { return static_cast<int>(col.size()); }
};

class MatrixAssembler {
 public:
  // Register a local block with CCS pattern `sp`; map[i] is the global index of
  // local row/column i (rows and columns share the map). Returns the block id.
  int add_block(const Sparsity& sp, const std::vector<int>& map);
  // Build the global pattern of size n x n and all scatter maps.
  void finalize(int n);

  void zero();
  void add(int block, const double* local_nz);
  const CsrMatrix& matrix() const { return A_; }

 private:
  struct Block {
    Sparsity sp;
    std::vector<int> map;
    std::vector<int> pos;  // CSR value index of each local nonzero
  };
  std::vector<Block> blocks_;
  CsrMatrix A_;
};

// Global row of every nonzero of a local column vector with pattern `sp`.
std::vector<int> vector_scatter_map(const Sparsity& sp, const std::vector<int>& map);

inline void scatter_add(const std::vector<int>& rows, const double* local_nz, double* global) {
  for (size_t k = 0; k < rows.size(); ++k) global[rows[k]] += local_nz[k];
}

}  // namespace fd
