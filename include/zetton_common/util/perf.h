#pragma once

#include <chrono>
#include <map>
#include <string>
#include <utility>
#include <vector>

#include "zetton_common/log/log.h"

namespace zetton {
namespace common {

class TimeCounter {
 public:
  void Start() { begin_ = std::chrono::system_clock::now(); }

  void End() { end_ = std::chrono::system_clock::now(); }
  void End(const std::string& msg) {
    end_ = std::chrono::system_clock::now();
    auto elapsed_time =
        std::chrono::duration_cast<std::chrono::nanoseconds>(end_ - begin_)
            .count();
    AINFO_F("TIMER {} elapsed_time: {} ms", msg,
            static_cast<double>(elapsed_time) / 1000000.0);
    begin_ = end_;
  }

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

class TimeCounterWrapper {
 public:
  explicit TimeCounterWrapper(std::string msg) : msg_(std::move(msg)) {
    timer_.Start();
  }

  ~TimeCounterWrapper() { timer_.End(msg_); }

  TimeCounterWrapper(const TimeCounterWrapper&) = delete;
  TimeCounterWrapper& operator=(const TimeCounterWrapper&) = delete;

 private:
  TimeCounter timer_;
  std::string msg_;
};

class FpsCalculator {
 public:
  void Start() { tc_.Start(); }
  void End() {
    tc_.End();
    time_of_runtime_.push_back(tc_.Duration());
  }
  void Clear() { std::vector<double>().swap(time_of_runtime_); }

  const std::size_t GetSize() const { return time_of_runtime_.size(); }

  const std::map<std::string, double> GetStats() const {
    std::map<std::string, double> stats_info_of_runtime_dict;
    double warmup_time = 0.0;
    double remain_time = 0.0;
    std::size_t warmup_iter = time_of_runtime_.size() / 5;
    for (std::size_t i = 0; i < time_of_runtime_.size(); ++i) {
      if (i < warmup_iter) {
        warmup_time += time_of_runtime_[i];
      } else {
        remain_time += time_of_runtime_[i];
      }
    }
    double avg_time = remain_time / static_cast<double>(
                                        time_of_runtime_.size() - warmup_iter);

    stats_info_of_runtime_dict["total_time"] = warmup_time + remain_time;
    stats_info_of_runtime_dict["warmup_time"] = warmup_time;
    stats_info_of_runtime_dict["remain_time"] = remain_time;
    stats_info_of_runtime_dict["warmup_iter"] =
        static_cast<double>(warmup_iter);
    stats_info_of_runtime_dict["avg_time"] = avg_time;
    stats_info_of_runtime_dict["iterations"] =
        static_cast<double>(time_of_runtime_.size());

    return stats_info_of_runtime_dict;
  }

  void PrintInfo(const std::string& prefix = "AverageTimeCollector") {
    auto stats = GetStats();
    PrintInfo(stats, prefix);
  }

  void PrintInfo(const std::map<std::string, double>& stats,
                 const std::string& prefix = "AverageTimeCollector") {
    if (time_of_runtime_.size() < 10) {
      AWARN_F("Too few inference times to print stats info.");
    }
    AINFO_F("============= Runtime Stats Info ({}) =============", prefix);
    AINFO_F("Total iterations: {}", time_of_runtime_.size());
    AINFO_F("Total time of runtime: {}s.", stats.at("total_time"));
    AINFO_F("Warmup iterations: {}.",
            static_cast<int>(stats.at("warmup_iter")));
    AINFO_F("Total time of runtime in warmup step: {}s.",
            stats.at("warmup_time"));
    AINFO_F("Average time of runtime (w/o warmup step): {}ms.",
            stats.at("avg_time") * 1000);
    AINFO_F("Average FPS (w/o warmup step): {}.", 1.0 / stats.at("avg_time"));
    AINFO_F("============= Runtime Stats Info ({}) =============", prefix);
  }

 private:
  zetton::common::TimeCounter tc_;
  std::vector<double> time_of_runtime_;
};

#ifdef ZETTON_DISABLE_PERF

// disable macros.
#define ZETTON_PERF_FUNCTION()

#define ZETTON_PERF_FUNCTION_WITH_INDICATOR(indicator)

#define ZETTON_PERF_BLOCK_START()

#define ZETTON_PERF_BLOCK_END(msg)

#define ZETTON_PERF_BLOCK_END_WITH_INDICATOR(indicator, msg)

#else

inline std::string get_full_name(const std::string& full_name) {
  std::size_t end = full_name.find('(');
  if (end == std::string::npos) {
    return full_name;
  }
  std::string new_str = full_name.substr(0, end);
  std::size_t start = new_str.rfind(' ');
  if (start == std::string::npos) {
    return full_name;
  }
  return new_str.substr(start + 1);
}

inline std::string get_full_name(const std::string& full_name,
                                 const std::string& indicator) {
  return indicator + "_" + get_full_name(full_name);
}

#define ZETTON_PERF_FUNCTION()                        \
  zetton::common::TimeCounterWrapper _timer_wrapper_( \
      zetton::common::get_full_name(__PRETTY_FUNCTION__))

#define ZETTON_PERF_FUNCTION_WITH_INDICATOR(indicator) \
  zetton::common::TimeCounterWrapper _timer_wrapper_(  \
      zetton::common::get_full_name(__PRETTY_FUNCTION__, indicator))

#define ZETTON_PERF_BLOCK_START()      \
  zetton::common::TimeCounter _timer_; \
  _timer_.Start()

#define ZETTON_PERF_BLOCK_END(msg) _timer_.End(msg)

#define ZETTON_PERF_BLOCK_END_WITH_INDICATOR(indicator, msg) \
  _timer_.End(indicator + "_" + msg)

#endif  // ZETTON_DISABLE_PERF

}  // namespace common
}  // namespace zetton
