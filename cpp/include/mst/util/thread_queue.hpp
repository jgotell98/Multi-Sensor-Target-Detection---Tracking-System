#pragma once

#include <condition_variable>
#include <mutex>
#include <optional>
#include <queue>

namespace mst {

template <class T>
class ThreadQueue final {
 public:
  void push(T v) {
    {
      std::lock_guard<std::mutex> lock(mu_);
      q_.push(std::move(v));
    }
    cv_.notify_one();
  }

  std::optional<T> pop_blocking() {
    std::unique_lock<std::mutex> lock(mu_);
    cv_.wait(lock, [&] { return stopped_ || !q_.empty(); });
    if (q_.empty()) return std::nullopt;
    T v = std::move(q_.front());
    q_.pop();
    return v;
  }

  void stop() {
    {
      std::lock_guard<std::mutex> lock(mu_);
      stopped_ = true;
    }
    cv_.notify_all();
  }

 private:
  std::mutex mu_;
  std::condition_variable cv_;
  std::queue<T> q_;
  bool stopped_ = false;
};

}  // namespace mst

