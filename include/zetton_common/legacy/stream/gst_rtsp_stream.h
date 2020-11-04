#pragma once

#include <string>

#include "zetton_common/legacy/interface/base_stream.h"

namespace zetton {
namespace common {
namespace legacy {

class GstRtspStream : public BaseStream {
 public:
  explicit GstRtspStream() = default;
  ~GstRtspStream() = default;

  bool open(const std::string &url) override;
  bool open(const std::string &url, const std::string &decoder,
            const std::string &user_id = "", const std::string &user_pw = "");
  bool isOpened() override;
  void release() override;
  bool read(cv::Mat &frame) override;
  bool read(void *frame_data) override;

 protected:
  /**
   * @brief GStreamer pipeline string
   */
  std::string pipeline_;

  /**
   * @brief API reference for OpenCV VideoCapture
   */
  int api_ = cv::CAP_GSTREAMER;

  /**
   * @brief instance of OpenCV VideoCapture
   */
  cv::VideoCapture cap_;
};

}  // namespace legacy
}  // namespace common
}  // namespace zetton