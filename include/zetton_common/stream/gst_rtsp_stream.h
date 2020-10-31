#pragma once

#include <string>

#include "zetton_common/interface/base_stream.h"

namespace zetton {
namespace common {

class GstRtspStream : public BaseStream {
 public:
  explicit GstRtspStream() = default;
  ~GstRtspStream() = default;

  bool open(const std::string &url) override;
  bool open(const std::string &url, const std::string &decoder = "omxh264dec",
            const std::string &user_id = "", const std::string &user_pw = "");
  bool isOpened() override;
  void release() override;
  bool read(cv::Mat &frame) override;
  bool read(void *frame_data) override;

 protected:
  std::string pipeline_;
  int api_ = cv::CAP_GSTREAMER;
  cv::VideoCapture cap_;
};

}  // namespace common
}  // namespace zetton