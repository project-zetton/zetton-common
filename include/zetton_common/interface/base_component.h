#pragma once

#include <ros/ros.h>

#include "zetton_common/util/ros_util.h"

namespace zetton {
namespace fusion {

class BaseComponent {
 public:
  BaseComponent() = default;
  virtual inline bool Init() = 0;

  virtual std::string Name() const { return "BaseComponent"; }
};

}  // namespace fusion
}  // namespace zetton
