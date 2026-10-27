// Stand-in for tbb::concurrent_vector: std::deque keeps element addresses stable on push_back.
// Only push_back and indexed access from the result manager are used.
#pragma once
#include <deque>
#include <mutex>

namespace tbb {
template <class T>
class concurrent_vector {
 public:
  void push_back(const T& v) { std::lock_guard<std::mutex> l(m_); d_.push_back(v); }
  T& operator[](size_t i) { return d_[i]; }
  const T& operator[](size_t i) const { return d_[i]; }
  size_t size() const { return d_.size(); }
  void clear() { d_.clear(); }
  void assign(size_t n, const T& v) { d_.assign(n, v); }
 private:
  std::mutex m_;
  std::deque<T> d_;
};
}  // namespace tbb
