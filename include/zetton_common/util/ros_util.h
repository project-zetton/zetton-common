#pragma once

#include "ros/ros.h"
#include "zetton_common/util/log.h"
#include "zetton_common/util/macros.h"

namespace zetton {
namespace common {

#ifndef GPARAM
#define GPARAM(x, y)                                    \
  do {                                                  \
    if (!nh_->getParam(x, y)) {                         \
      AFATAL << "get param from [" << x << "] failed."; \
    }                                                   \
  } while (0)
#endif

#ifndef GPARAM_D
#define GPARAM_D(T, x, y, d) zetton::common::RosNodeHandler::GetParam<T>(x, y, d)
#endif

#ifndef DECLARE_ROS_INITIALIZATION
#define DECLARE_ROS_INITIALIZATION(classname)                 \
 public:                                                      \
  explicit inline classname(ros::NodeHandle *nh) : nh_(nh){}; \
                                                              \
 protected:                                                   \
  ros::NodeHandle *nh_;
#endif

class RosNodeHandler {
 public:
  ros::NodeHandle *GetNh() {
    if (nh_ == nullptr) {
      nh_ = new ros::NodeHandle("~");
    }
    //    AERROR_IF(nh_ == nullptr) << "Invalid node handler for ROS!";
    //    assert(nh_ != nullptr);
    return nh_;
  }

  void SetNh(ros::NodeHandle *nh) { nh_ = nh; }

  template <class T>
  static void GetParam(std::string param_path, T &param,
                       const T &default_value) {
    auto nh = zetton::common::RosNodeHandler::Instance()->GetNh();
    if (!nh->hasParam(param_path))
      ROS_ERROR_STREAM("get param from [" << param_path
                                          << "] failed, use default value ["
                                          << default_value << "] instead.");
    nh->param<T>(param_path, param, default_value);
  }

 private:
  ros::NodeHandle *nh_{};

  DECLARE_SINGLETON(RosNodeHandler)
};

}  // namespace common
}  // namespace zetton