// freedyn2_modal: modal reduction of a linear FE model (Craig-Bampton), the sparse part of
// fdgen.fem. Element matrices, constraints (RBE2, RBE3) and the invariants are done in Python;
// this program eliminates the constraints, factorizes and solves.
//
//   freedyn2_modal input.bin output.bin
//
// Input (little endian; int64 i, float64 d):
//   i: magic 0x464D4F44 ("DOMF"), version 1, n, nnzK, nnzM, nnzG, nb, nmodes, maxit
//   d: shift, tol (largest relative residual |K x - lambda M x| / |(K - shift M) x| of the modes)
//   K (full symmetric pattern, duplicates are summed): i rows[nnzK], i cols[nnzK], d vals[nnzK]
//   M: the same
//   G: dependent dofs u_r = sum G_rc u_c with c independent: i rows, i cols, d vals
//   i boundary dofs[nb] (independent)
// Output:
//   i: n, ncols = nb + nmodes, nb, iterations
//   d: eigenvalues of the fixed-interface (nb > 0) or free (nb = 0) modes [nmodes]
//   d: V (n x ncols, column major): [constraint modes (unit boundary dof), normal modes] in all dofs
//   d: Kr, Mr (ncols x ncols, column major): V^T K V, V^T M V
//
// Constraint modes: interior response to a unit displacement of one boundary dof (K_ii Psi =
// -K_ib), normal modes: K_ii phi = lambda M_ii phi with the boundary fixed, by subspace iteration
// with (K_ii - shift M_ii)^-1 (sparse LDL^T); shift < 0 for a free body (nb = 0).

#include <Eigen/Dense>
#include <Eigen/SparseCholesky>
#include <Eigen/SparseCore>

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <random>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

using Sp = Eigen::SparseMatrix<double>;
using Mat = Eigen::MatrixXd;
using Vec = Eigen::VectorXd;
using Trip = Eigen::Triplet<double>;

namespace {

struct Reader {
  std::ifstream in;
  explicit Reader(const char* path) : in(path, std::ios::binary) {
    if (!in) throw std::runtime_error(std::string("cannot open ") + path);
  }
  template <class T>
  std::vector<T> read(size_t n) {
    std::vector<T> v(n);
    in.read(reinterpret_cast<char*>(v.data()), static_cast<std::streamsize>(n * sizeof(T)));
    if (!in) throw std::runtime_error("input file too short");
    return v;
  }
};

Sp triplets(Reader& r, int64_t n, int64_t nnz, int64_t ncols = -1) {
  auto rows = r.read<int64_t>(nnz), cols = r.read<int64_t>(nnz);
  auto vals = r.read<double>(nnz);
  std::vector<Trip> t;
  t.reserve(nnz);
  for (int64_t k = 0; k < nnz; ++k) t.emplace_back(static_cast<int>(rows[k]), static_cast<int>(cols[k]), vals[k]);
  Sp A(n, ncols < 0 ? n : ncols);
  A.setFromTriplets(t.begin(), t.end());
  return A;
}

// Rows/columns sel of A (sel: indices into A).
Sp select(const Sp& A, const std::vector<int>& rows, const std::vector<int>& cols) {
  std::vector<int> rmap(A.rows(), -1), cmap(A.cols(), -1);
  for (size_t i = 0; i < rows.size(); ++i) rmap[rows[i]] = static_cast<int>(i);
  for (size_t i = 0; i < cols.size(); ++i) cmap[cols[i]] = static_cast<int>(i);
  std::vector<Trip> t;
  for (int j = 0; j < A.outerSize(); ++j)
    for (Sp::InnerIterator it(A, j); it; ++it)
      if (rmap[it.row()] >= 0 && cmap[it.col()] >= 0) t.emplace_back(rmap[it.row()], cmap[it.col()], it.value());
  Sp B(static_cast<int>(rows.size()), static_cast<int>(cols.size()));
  B.setFromTriplets(t.begin(), t.end());
  return B;
}

}  // namespace

int main(int argc, char** argv) {
  try {
    if (argc != 3) {
      std::cerr << "usage: freedyn2_modal input.bin output.bin\n";
      return 2;
    }
    const bool verbose = std::getenv("FD_MODAL_VERBOSE") != nullptr;
    auto t0 = std::chrono::steady_clock::now();
    auto lap = [&](const char* what) {
      if (!verbose) return;
      const auto t1 = std::chrono::steady_clock::now();
      std::cerr << what << ": " << std::chrono::duration<double>(t1 - t0).count() << " s\n";
      t0 = t1;
    };
    Reader r(argv[1]);
    auto h = r.read<int64_t>(9);
    if (h[0] != 0x464D4F44 || h[1] != 1) throw std::runtime_error("not a freedyn2_modal input (version 1)");
    const int64_t n = h[2], nnzK = h[3], nnzM = h[4], nnzG = h[5], nb = h[6], nmodes = h[7], maxit = h[8];
    auto hd = r.read<double>(2);
    const double shift = hd[0], tol = hd[1];
    Sp K = triplets(r, n, nnzK), M = triplets(r, n, nnzM), G = triplets(r, n, nnzG);
    auto bnd = r.read<int64_t>(nb);

    // independent dofs and T: u = T u_indep
    std::vector<char> dep(n, 0);
    for (int j = 0; j < G.outerSize(); ++j)
      for (Sp::InnerIterator it(G, j); it; ++it) dep[it.row()] = 1;
    std::vector<int> indep_of(n, -1), indep;
    for (int64_t i = 0; i < n; ++i)
      if (!dep[i]) indep_of[i] = static_cast<int>(indep.size()), indep.push_back(static_cast<int>(i));
    const int ni_all = static_cast<int>(indep.size());
    std::vector<Trip> tt;
    for (int i : indep) tt.emplace_back(i, indep_of[i], 1.0);
    for (int j = 0; j < G.outerSize(); ++j)
      for (Sp::InnerIterator it(G, j); it; ++it) {
        if (dep[it.col()]) throw std::runtime_error("a dependent dof depends on a dependent dof");
        tt.emplace_back(static_cast<int>(it.row()), indep_of[it.col()], it.value());
      }
    Sp T(n, ni_all);
    T.setFromTriplets(tt.begin(), tt.end());
    Sp Kn = Sp(T.transpose() * K * T), Mn = Sp(T.transpose() * M * T);
    lap("read, eliminate");

    // boundary / interior (indices into the independent dofs)
    std::vector<char> isb(ni_all, 0);
    std::vector<int> b, in;
    for (int64_t k = 0; k < nb; ++k) {
      const int d = indep_of.at(bnd[k]);
      if (d < 0) throw std::runtime_error("boundary dof " + std::to_string(bnd[k]) + " is dependent");
      isb[d] = 1, b.push_back(d);
    }
    for (int i = 0; i < ni_all; ++i)
      if (!isb[i]) in.push_back(i);
    const int ni = static_cast<int>(in.size());
    Sp Kii = select(Kn, in, in), Mii = select(Mn, in, in), Kib = select(Kn, in, b);

    const double s = nb > 0 ? 0.0 : shift;
    Sp A = Kii - s * Mii;
    Eigen::SimplicialLDLT<Sp> ldlt(A);
    if (ldlt.info() != Eigen::Success) throw std::runtime_error("factorization failed (body not restrained by the boundary?)");
    if ((ldlt.vectorD().array() <= 0).any() && s <= 0)
      throw std::runtime_error("matrix not positive definite: the boundary does not restrain all rigid body motions"
                               " (or a free body without a negative shift)");

    lap("factorize");
    if (verbose) std::cerr << "n " << ni << ", nnz(K_ii) " << Kii.nonZeros() << ", nnz(L) " << ldlt.matrixL().nestedExpression().nonZeros() << "\n";
    // constraint modes: the columns solved in parallel (the factor is only read)
    Mat Psi(ni, nb);
    if (nb > 0) {
      const Mat Kibd = Mat(Kib);
      const int nt = static_cast<int>(std::max(1u, std::min<unsigned>(std::thread::hardware_concurrency(), 16)));
      std::vector<std::thread> pool;
      for (int t = 0; t < nt; ++t)
        pool.emplace_back([&, t] {
          for (int c = t; c < nb; c += nt) Psi.col(c) = -ldlt.solve(Vec(Kibd.col(c)));
        });
      for (auto& th : pool) th.join();
      lap("constraint modes");
    }

    // Lanczos with shift-invert in the M inner product, full reorthogonalization: the Krylov
    // space of (K_ii - s M_ii)^-1 M_ii grows until the m eigenpairs nearest the shift converge
    // (residual |K x - lambda M x| / |(K - s M) x| < tol); a few solves per mode instead of the
    // 2m per iteration of a subspace iteration.
    const int m = static_cast<int>(std::min<int64_t>(nmodes, ni));
    Vec lam = Vec::Zero(std::max(m, 1));
    Mat Phi(ni, m);
    int it = 0;
    if (m > 0) {
      const int kmax = std::min(ni, std::max(4 * m + 40, 3 * m + 60) * 4);
      Mat Q(ni, std::min(kmax + 1, ni));
      std::vector<double> alpha, beta;
      std::mt19937 gen(12345);
      std::normal_distribution<double> nd;
      Vec q(ni);
      for (int i = 0; i < ni; ++i) q(i) = nd(gen);
      Vec Mq = Mii * q;
      q /= std::sqrt(q.dot(Mq));
      Q.col(0) = q;
      int k = 0, next_check = std::min(ni, 2 * m + 20);
      double best = 1e300;
      int best_k = 0;
      for (;;) {
        // one Lanczos step: w = A^-1 M q_k, orthogonalized (M) against all q (twice)
        Vec w = ldlt.solve(Vec(Mii * Q.col(k)));
        for (int pass = 0; pass < 2; ++pass) {
          const Vec c = Q.leftCols(k + 1).transpose() * (Mii * w);
          w -= Q.leftCols(k + 1) * c;
          if (pass == 0) alpha.push_back(c(k));
        }
        const double b = std::sqrt(std::max(w.dot(Mii * w), 0.0));
        ++k, ++it;
        if (k < next_check && k < ni && b > 1e-14) {
          beta.push_back(b);
          Q.col(k) = w / b;
          continue;
        }
        // Ritz values of T (k x k): theta = 1 / (lambda - s), the largest nearest the shift
        Mat Tm = Mat::Zero(k, k);
        for (int i = 0; i < k; ++i) Tm(i, i) = alpha[i];
        for (int i = 0; i + 1 < k; ++i) Tm(i, i + 1) = Tm(i + 1, i) = beta[i];
        Eigen::SelfAdjointEigenSolver<Mat> es(Tm);
        const int mm = std::min(m, k);
        Mat S = es.eigenvectors().rightCols(mm).rowwise().reverse();
        Vec th = es.eigenvalues().tail(mm).reverse();
        Mat X = Q.leftCols(k) * S;
        const Mat MX = Mii * X, KX = Kii * X;
        double err = 0;
        for (int j = 0; j < mm; ++j) {
          lam(j) = s + 1.0 / th(j);
          const double ref = (KX.col(j) - s * MX.col(j)).norm();
          err = std::max(err, (KX.col(j) - lam(j) * MX.col(j)).norm() / std::max(ref, 1e-300));
        }
        if (verbose) std::cerr << "lanczos " << k << " residual " << err << "\n";
        const bool done = mm == m && (err < tol || (best < 1e3 * tol && err > 0.9 * best && k - best_k >= m));
        if (err < 0.9 * best) best = err, best_k = k;
        if (done || k >= kmax || k >= ni || b <= 1e-14) {
          if (!done) std::cerr << "freedyn2_modal: Lanczos residual " << err << " after " << k << " vectors\n";
          // M-orthonormal Ritz vectors (reorthogonalized Lanczos: Q is M-orthonormal)
          Phi = X;
          for (int j = 0; j < mm; ++j) Phi.col(j) /= std::sqrt(Phi.col(j).dot(MX.col(j)));
          break;
        }
        beta.push_back(b);
        Q.col(k) = w / b;
        next_check = std::min(ni, k + std::max(10, m / 2));
      }
      lap("lanczos");
      if (verbose) std::cerr << "solves " << it << "\n";
    }

    // basis in the independent dofs, then all dofs
    const int nc = static_cast<int>(nb) + m;
    Mat Vn = Mat::Zero(ni_all, nc);
    for (int i = 0; i < ni; ++i) {
      for (int k = 0; k < nb; ++k) Vn(in[i], k) = Psi(i, k);
      for (int k = 0; k < m; ++k) Vn(in[i], nb + k) = Phi(i, k);
    }
    for (int k = 0; k < nb; ++k) Vn(b[k], k) = 1.0;
    Mat V = T * Vn;
    Mat Kr = Vn.transpose() * (Kn * Vn), Mr = Vn.transpose() * (Mn * Vn);

    std::ofstream out(argv[2], std::ios::binary);
    const int64_t oh[4] = {n, nc, nb, it};
    out.write(reinterpret_cast<const char*>(oh), sizeof oh);
    out.write(reinterpret_cast<const char*>(lam.data()), sizeof(double) * m);
    out.write(reinterpret_cast<const char*>(V.data()), sizeof(double) * V.size());
    out.write(reinterpret_cast<const char*>(Kr.data()), sizeof(double) * Kr.size());
    out.write(reinterpret_cast<const char*>(Mr.data()), sizeof(double) * Mr.size());
    if (!out) throw std::runtime_error(std::string("cannot write ") + argv[2]);
    return 0;
  } catch (const std::exception& e) {
    std::cerr << "freedyn2_modal: " << e.what() << "\n";
    return 1;
  }
}
