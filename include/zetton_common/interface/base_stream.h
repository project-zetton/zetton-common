#pragma once

#include <string>

#include "opencv2/opencv.hpp"

namespace zetton {
namespace common {

class BaseStream {
 public:
  explicit BaseStream(const std::string &url) : url_(url){};
  ~BaseStream() = default;

  virtual bool open() = 0;
  virtual bool isOpened() = 0;
  virtual void release() = 0;
  virtual bool read(cv::Mat &frame) = 0;
  virtual bool read(void *frame_data) = 0;

 protected:
  std::string url_;
};

}  // namespace common
}  // namespace zetton
