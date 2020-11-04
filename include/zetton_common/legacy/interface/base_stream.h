#pragma once

#include <string>

#include "opencv2/opencv.hpp"
#include "zetton_common/util/registerer.h"

namespace zetton {
namespace common {
namespace legacy {

class BaseStream {
 public:
  explicit BaseStream() = default;
  ~BaseStream() = default;

  /**
   * @brief Opens a video stream for video capturing.
   *
   * @param url The stream url
   * @return `true` if the operation is successful
   */
  virtual bool open(const std::string &url) = 0;

  /**
   * @brief Returns true if video capturing has been initialized already.
   *
   * @return true if video capturing has been initialized already
   */
  virtual bool isOpened() = 0;

  /**
   * @brief Closes video stream.
   */
  virtual void release() = 0;

  /**
   * @brief Grabs, decodes and returns the next video frame.
   *
   * @param frame The captured frame as output
   * @return `false` if no frames has been grabbed
   */
  virtual bool read(cv::Mat &frame) = 0;

  /**
   * @brief Grabs, decodes and returns the next video frame.
   *
   * @param frame The captured frame as output
   * @return `false` if no frames has been grabbed
   */
  virtual bool read(void *frame_data) = 0;

 protected:
  std::string url_;

  ZETTON_REGISTER_REGISTERER(BaseStream);
#define ZETTON_REGISTER_STREAM(name) ZETTON_REGISTER_CLASS(BaseStream, name)
};

}  // namespace lagecy
}  // namespace common
}  // namespace zetton
