// Mutex-based stand-in for MSVC PPL concurrency::concurrent_queue.
#pragma once
#include <mutex>
#include <queue>

namespace concurrency {
template <class T>
class concurrent_queue {
 public:
  void push(const T& v) { std::lock_guard<std::mutex> l(m_); q_.push(v); }
  bool try_pop(T& v) {
    std::lock_guard<std::mutex> l(m_);
    if (q_.empty()) return false;
    v = q_.front();
    q_.pop();
    return true;
  }
  bool empty() const { std::lock_guard<std::mutex> l(m_); return q_.empty(); }
  size_t unsafe_size() const { return q_.size(); }
  void clear() { std::lock_guard<std::mutex> l(m_); q_ = {}; }
 private:
  mutable std::mutex m_;
  std::queue<T> q_;
};
}  // namespace concurrency
namespace Concurrency = concurrency;
