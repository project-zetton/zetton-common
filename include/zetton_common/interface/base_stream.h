#pragma once

#include <string>

#include "opencv2/opencv.hpp"
#include "zetton_common/util/registerer.h"

namespace zetton {
namespace common {

class BaseStream {
 public:
  explicit BaseStream() = default;
  ~BaseStream() = default;

  virtual bool open(const std::string &url) = 0;
  virtual bool isOpened() = 0;
  virtual void release() = 0;
  virtual bool read(cv::Mat &frame) = 0;
  virtual bool read(void *frame_data) = 0;

 protected:
  std::string url_;

  ZETTON_REGISTER_REGISTERER(BaseStream);
#define ZETTON_REGISTER_STREAM(name) ZETTON_REGISTER_CLASS(BaseStream, name)
};

}  // namespace common
}  // namespace zetton
