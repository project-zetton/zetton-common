#pragma once

#include <chrono>
#include <string>

#include "zetton_common/log/log.h"

namespace zetton {
namespace common {

class TimeCounter {
 public:
  void Start() { begin_ = std::chrono::system_clock::now(); }

  void End() { end_ = std::chrono::system_clock::now(); }

  double Duration() {
    auto duration =
        std::chrono::duration_cast<std::chrono::microseconds>(end_ - begin_);
    return static_cast<double>(duration.count()) *
           std::chrono::microseconds::period::num /
           std::chrono::microseconds::period::den;
  }

  void PrintInfo(const std::string& prefix = "TimeCounter: ",
                 bool print_out = true) {
    if (!print_out) {
      return;
    }
    AINFO_F("{} duration = {}s.", prefix, Duration());
  }

 private:
  std::chrono::time_point<std::chrono::system_clock> begin_;
  std::chrono::time_point<std::chrono::system_clock> end_;
};

}  // namespace common
}  // namespace zetton
