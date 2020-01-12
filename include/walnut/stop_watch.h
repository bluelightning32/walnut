#ifndef WALNUT_STOP_WATCH_H__
#define WALNUT_STOP_WATCH_H__

#include <chrono>

namespace walnut {

class StopWatch {
 public:
  StopWatch() {
    Start();
  }

  void Start() {
    start_ = std::chrono::steady_clock::now();
  }

  void Stop() {
    end_ = std::chrono::steady_clock::now();
  }

  auto diff() const {
    return end_ - start_;
  }

 private:
  std::chrono::time_point<std::chrono::steady_clock> start_;
  std::chrono::time_point<std::chrono::steady_clock> end_;
};

inline std::ostream& operator<<(std::ostream& out, const StopWatch& watch) {
  out << std::chrono::duration<double>(watch.diff()).count() << "s";
  return out;
}

}  // walnut

#endif // WALNUT_STOP_WATCH_H__
