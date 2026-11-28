// Multifrontal supernodal LDL^T of a sparse symmetric matrix (no pivoting), with Eigen's dense
// kernels: the columns of the factor with the same structure (supernodes) are factorized as dense
// blocks, the update of the rest is a dense matrix product (Eigen's GEMM; the large updates and
// assemblies in row strips / columns on all cores, a small thread pool). For freedyn2_modal:
// K - sigma M of an FE model, 5 to 12 times faster than Eigen's SimplicialLDLT (which works column
// by column; docs/findings.md), the same results to round-off. Apache-2.0, no other dependency
// than Eigen.
//
//   SupernodalLDLT f;
//   f.analyze(A);       // ordering (AMD), elimination tree, supernodes, their row structure
//   f.factorize(A);     // numeric; again for another matrix with the same pattern
//   x = f.solve(b);     // thread safe (const)
//   f.negatives();      // number of negative pivots (inertia: eigenvalues below a shift)
//
// A: the full symmetric matrix (both triangles, column major). Without pivoting the factorization
// of an indefinite matrix can break down on a zero pivot (as SimplicialLDLT): factorize() returns
// false then.

#pragma once

#include <Eigen/Dense>
#include <Eigen/OrderingMethods>
#include <Eigen/SparseCore>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <mutex>
#include <thread>
#include <vector>

namespace fd {

// A small pool of worker threads for parallel_for(n, f): f(0) ... f(n - 1), the calling thread
// working along; returns when all are done. One job at a time (the factorization is sequential
// between them).
class ThreadPool {
 public:
  explicit ThreadPool(unsigned n = std::max(1u, std::min(std::thread::hardware_concurrency(), 16u))) {
    for (unsigned t = 1; t < n; ++t) workers_.emplace_back([this] { work(); });
  }
  ~ThreadPool() {
    {
      std::lock_guard<std::mutex> lock(mu_);
      stop_ = true;
    }
    cv_.notify_all();
    for (auto& w : workers_) w.join();
  }
  int size() const { return static_cast<int>(workers_.size()) + 1; }
  void parallel_for(int n, const std::function<void(int)>& f) {
    if (workers_.empty() || n <= 1) {
      for (int i = 0; i < n; ++i) f(i);
      return;
    }
    {
      std::lock_guard<std::mutex> lock(mu_);
      job_ = &f, n_ = n, next_ = 0, active_ = static_cast<int>(workers_.size()), ++gen_;
    }
    cv_.notify_all();
    run();
    std::unique_lock<std::mutex> lock(mu_);
    done_.wait(lock, [this] { return active_ == 0; });
    job_ = nullptr;
  }

 private:
  void run() {
    for (int i; (i = next_.fetch_add(1)) < n_;) (*job_)(i);
  }
  void work() {
    unsigned seen = 0;
    for (;;) {
      {
        std::unique_lock<std::mutex> lock(mu_);
        cv_.wait(lock, [&] { return stop_ || gen_ != seen; });
        if (stop_) return;
        seen = gen_;
      }
      run();
      std::lock_guard<std::mutex> lock(mu_);
      if (--active_ == 0) done_.notify_one();
    }
  }
  std::vector<std::thread> workers_;
  std::mutex mu_;
  std::condition_variable cv_, done_;
  const std::function<void(int)>* job_ = nullptr;
  std::atomic<int> next_{0};
  int n_ = 0, active_ = 0;
  unsigned gen_ = 0;
  bool stop_ = false;
};

class SupernodalLDLT {
 public:
  using Sp = Eigen::SparseMatrix<double>;
  using Mat = Eigen::MatrixXd;
  using Vec = Eigen::VectorXd;

  // A must be compressed (as after makeCompressed or an expression assigned to a SparseMatrix).
  void analyze(const Sp& A) {
    n_ = static_cast<int>(A.rows());
    // ordering: AMD on the pattern of A; old_of_[new] = old, new_of_[old] = new
    Eigen::PermutationMatrix<Eigen::Dynamic, Eigen::Dynamic, int> pinv;
    Eigen::AMDOrdering<int> amd;
    amd(A, pinv);
    old_of_.assign(pinv.indices().data(), pinv.indices().data() + n_);
    new_of_.assign(n_, 0);
    for (int k = 0; k < n_; ++k) new_of_[old_of_[k]] = k;
    // permuted lower triangle (row, position of the value in A) per column
    std::vector<int> cnt(n_ + 1, 0);
    for (int j = 0; j < n_; ++j)
      for (Sp::InnerIterator it(A, j); it; ++it)
        if (new_of_[it.row()] >= new_of_[j]) ++cnt[new_of_[j] + 1];
    for (int j = 0; j < n_; ++j) cnt[j + 1] += cnt[j];
    lp_ = cnt;
    li_.assign(cnt[n_], 0);
    lsrc_.assign(cnt[n_], 0);
    {
      std::vector<int> next(cnt.begin(), cnt.end() - 1);
      const auto* op = A.outerIndexPtr();
      const auto* ip = A.innerIndexPtr();
      for (int j = 0; j < n_; ++j)
        for (auto p = op[j]; p < op[j + 1]; ++p) {
          const int ni = new_of_[ip[p]], nj = new_of_[j];
          if (ni < nj) continue;
          li_[next[nj]] = ni;
          lsrc_[next[nj]] = static_cast<int64_t>(p);
          ++next[nj];
        }
    }
    // upper pattern by columns (rows i < k of column k): the transpose of the strict lower part
    std::vector<int> up(n_ + 1, 0), ui;
    for (int j = 0; j < n_; ++j)
      for (int p = lp_[j]; p < lp_[j + 1]; ++p)
        if (li_[p] > j) ++up[li_[p] + 1];
    for (int k = 0; k < n_; ++k) up[k + 1] += up[k];
    ui.assign(up[n_], 0);
    {
      std::vector<int> next(up.begin(), up.end() - 1);
      for (int j = 0; j < n_; ++j)
        for (int p = lp_[j]; p < lp_[j + 1]; ++p)
          if (li_[p] > j) ui[next[li_[p]]++] = j;
    }
    // elimination tree and the number of off-diagonal entries per column of L (Liu)
    std::vector<int> parent(n_, -1), mark(n_, -1), count(n_, 0);
    for (int k = 0; k < n_; ++k) {
      mark[k] = k;
      for (int p = up[k]; p < up[k + 1]; ++p) {
        for (int i = ui[p]; mark[i] != k; i = parent[i]) {
          if (parent[i] == -1) parent[i] = k;
          ++count[i];
          mark[i] = k;
        }
      }
    }
    // fundamental supernodes: j + 1 continues j's supernode if it is j's parent, j its only child,
    // and the structures nest (count[j] = count[j + 1] + 1)
    std::vector<int> children(n_, 0);
    for (int j = 0; j < n_; ++j)
      if (parent[j] >= 0) ++children[parent[j]];
    first_.clear();
    for (int j = 0; j < n_; ++j) {
      const bool cont = j > 0 && parent[j - 1] == j && children[j] == 1 && count[j - 1] == count[j] + 1;
      if (!cont) first_.push_back(j);
    }
    first_.push_back(n_);
    const int ns = static_cast<int>(first_.size()) - 1;
    super_of_.assign(n_, 0);
    for (int s = 0; s < ns; ++s)
      for (int j = first_[s]; j < first_[s + 1]; ++j) super_of_[j] = s;
    // row structure of each supernode below its columns: the rows of A in its columns and the
    // structures of its children (supernodes in increasing order: children come first)
    sparent_.assign(ns, -1);
    rows_.assign(ns, {});
    std::vector<std::vector<int>> kids(ns);
    std::vector<int> flag(n_, -1);
    for (int s = 0; s < ns; ++s) {
      const int f = first_[s], l = first_[s + 1];
      std::vector<int>& r = rows_[s];
      for (int j = f; j < l; ++j)
        for (int p = lp_[j]; p < lp_[j + 1]; ++p)
          if (li_[p] >= l && flag[li_[p]] != s) flag[li_[p]] = s, r.push_back(li_[p]);
      for (int c : kids[s])
        for (int i : rows_[c])
          if (i >= l && flag[i] != s) flag[i] = s, r.push_back(i);
      std::sort(r.begin(), r.end());
      if (!r.empty()) {
        sparent_[s] = super_of_[r[0]];
        kids[sparent_[s]].push_back(s);
      }
    }
    kids_ = std::move(kids);
    nnz_l_ = 0;
    for (int s = 0; s < ns; ++s) {
      const int64_t k = first_[s + 1] - first_[s], m = k + static_cast<int64_t>(rows_[s].size());
      nnz_l_ += k * m - k * (k + 1) / 2;
    }
  }

  // Numeric factorization of A (the pattern of analyze). False on a zero or non-finite pivot.
  bool factorize(const Sp& A) {
    const int ns = static_cast<int>(first_.size()) - 1;
    const double* val = A.valuePtr();
    L_.assign(ns, Mat());
    d_.setZero(n_);
    negatives_ = 0;
    std::vector<Mat> update(ns);
    std::vector<int> loc(n_, -1);
    for (int s = 0; s < ns; ++s) {
      const int f = first_[s], k = first_[s + 1] - f;
      const std::vector<int>& r = rows_[s];
      const int m = k + static_cast<int>(r.size());
      for (int a = 0; a < k; ++a) loc[f + a] = a;
      for (int a = 0; a < static_cast<int>(r.size()); ++a) loc[r[a]] = k + a;
      Mat F = Mat::Zero(m, m);
      for (int j = f; j < f + k; ++j)
        for (int p = lp_[j]; p < lp_[j + 1]; ++p) F(loc[li_[p]], j - f) += val[lsrc_[p]];
      for (int c : kids_[s]) {  // extend-add of the children's updates (their fronts' lower right)
        const std::vector<int>& rc = rows_[c];
        const int nc = static_cast<int>(rc.size());
        const auto U = update[c].bottomRightCorner(nc, nc);
        std::vector<int> map(nc);
        for (int a = 0; a < nc; ++a) map[a] = loc[rc[a]];
        // column b of U goes to column map[b] of F: no two b write the same (in chunks on all cores)
        auto add = [&](int b0, int b1) {
          for (int b = b0; b < b1; ++b) {
            double* Fc = F.col(map[b]).data();
            const double* Uc = U.col(b).data();
            for (int a = b; a < nc; ++a) Fc[map[a]] += Uc[a];
          }
        };
        if (nc >= kParallelRows && pool_.size() > 1) {
          const int nch = 4 * pool_.size();
          std::vector<int> e(nch + 1);  // the columns get shorter: equal work, not equal width
          for (int t = 0; t <= nch; ++t) e[t] = static_cast<int>(std::lround(nc * (1 - std::sqrt(1 - double(t) / nch))));
          pool_.parallel_for(nch, [&](int t) { add(e[t], e[t + 1]); });
        } else {
          add(0, nc);
        }
        update[c] = Mat();
      }
      if (!dense_ldlt(F, k, d_.segment(f, k), pool_)) return false;
      for (int a = 0; a < k; ++a)
        if (d_(f + a) < 0) ++negatives_;
      L_[s] = F.leftCols(k);
      if (m > k) update[s] = std::move(F);  // the parent reads the Schur complement from the front
      for (int a = 0; a < k; ++a) loc[f + a] = -1;
      for (int i : r) loc[i] = -1;
    }
    return true;
  }

  Vec solve(const Vec& b) const {
    const int ns = static_cast<int>(first_.size()) - 1;
    Vec x(n_);
    for (int i = 0; i < n_; ++i) x(new_of_[i]) = b(i);
    for (int s = 0; s < ns; ++s) {  // L y = x
      const int f = first_[s], k = first_[s + 1] - f;
      const Mat& L = L_[s];
      L.topRows(k).triangularView<Eigen::UnitLower>().solveInPlace(x.segment(f, k));
      const std::vector<int>& r = rows_[s];
      if (r.empty()) continue;
      const Vec t = L.bottomRows(L.rows() - k) * x.segment(f, k);
      for (size_t a = 0; a < r.size(); ++a) x(r[a]) -= t(static_cast<Eigen::Index>(a));
    }
    x.array() /= d_.array();
    for (int s = ns - 1; s >= 0; --s) {  // L^T x = y
      const int f = first_[s], k = first_[s + 1] - f;
      const Mat& L = L_[s];
      const std::vector<int>& r = rows_[s];
      if (!r.empty()) {
        Vec y(r.size());
        for (size_t a = 0; a < r.size(); ++a) y(static_cast<Eigen::Index>(a)) = x(r[a]);
        x.segment(f, k) -= L.bottomRows(L.rows() - k).transpose() * y;
      }
      L.topRows(k).transpose().triangularView<Eigen::UnitUpper>().solveInPlace(x.segment(f, k));
    }
    Vec out(n_);
    for (int i = 0; i < n_; ++i) out(i) = x(new_of_[i]);
    return out;
  }

  int negatives() const { return negatives_; }
  const Vec& pivots() const { return d_; }  // D (in the permuted order)
  int64_t nnz_l() const { return nnz_l_; }  // entries of L below the diagonal
  int supernodes() const { return static_cast<int>(first_.size()) - 1; }
  // columns and rows (with the columns) of the front of supernode s
  int front_cols(int s) const { return first_[s + 1] - first_[s]; }
  int front_rows(int s) const { return front_cols(s) + static_cast<int>(rows_[s].size()); }
  int super_parent(int s) const { return sparent_[s]; }

 private:
  // LDL^T of the first k columns of the front F (m x m, lower triangle; no pivoting), blocked:
  // L in F's first k columns (unit diagonal), D in d, the Schur complement in the rest of F.
  static constexpr int kBlock = 96;          // columns per block of the dense factorization
  static constexpr int kParallelRows = 384;  // updates of fewer rows stay on one thread

  static bool dense_ldlt(Mat& F, int k, Eigen::Ref<Vec> d, ThreadPool& pool) {
    const int m = static_cast<int>(F.rows());
    for (int j0 = 0; j0 < k; j0 += kBlock) {
      const int j1 = std::min(k, j0 + kBlock), nb = j1 - j0;
      // the diagonal block, left-looking within it
      for (int j = j0; j < j1; ++j) {
        if (j > j0) {
          const Vec w = d.segment(j0, j - j0).cwiseProduct(F.row(j).segment(j0, j - j0).transpose());
          F.col(j).segment(j, j1 - j) -= F.block(j, j0, j1 - j, j - j0) * w;
        }
        const double piv = F(j, j);
        if (piv == 0.0 || !std::isfinite(piv)) return false;
        d(j) = piv;
        F.col(j).segment(j + 1, j1 - j - 1) /= piv;
      }
      if (j1 == m) break;
      const int r = m - j1;
      // the panel below: W = F21 L11^-T (= L21 D), L21 = W D^-1 (triangular solve, BLAS 3)
      auto F21 = F.block(j1, j0, r, nb);
      F.block(j0, j0, nb, nb).triangularView<Eigen::UnitLower>().transpose().solveInPlace<Eigen::OnTheRight>(F21);
      const Mat W = F21;
      F21 = W * d.segment(j0, nb).cwiseInverse().asDiagonal();
      // the rest: F22 -= W L21^T (lower triangle, GEMM)
      auto F22 = F.bottomRightCorner(r, r);
      if (r >= kParallelRows && pool.size() > 1) {
        // row strips of the lower triangle with about equal work (strip t ends at r sqrt(t / ns)),
        // each a GEMM over the columns left of its end; what it writes above the diagonal is
        // never read (only lower triangles are)
        const int ns = 4 * pool.size();
        std::vector<int> e(ns + 1);
        for (int t = 0; t <= ns; ++t) e[t] = static_cast<int>(std::lround(r * std::sqrt(double(t) / ns)));
        pool.parallel_for(ns, [&](int t) {
          const int r0 = e[t], r1 = e[t + 1];
          if (r1 > r0) F22.block(r0, 0, r1 - r0, r1).noalias() -= W.middleRows(r0, r1 - r0) * F21.topRows(r1).transpose();
        });
      } else {
        F22.triangularView<Eigen::Lower>() -= W * F21.transpose();
      }
    }
    return true;
  }

  ThreadPool pool_;
  int n_ = 0;
  std::vector<int> old_of_, new_of_;
  std::vector<int> lp_, li_;          // permuted lower pattern
  std::vector<int64_t> lsrc_;         // position of each value in A
  std::vector<int> first_, super_of_, sparent_;
  std::vector<std::vector<int>> rows_, kids_;
  std::vector<Mat> L_;
  Vec d_;
  int negatives_ = 0;
  int64_t nnz_l_ = 0;
};

}  // namespace fd
