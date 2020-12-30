#pragma once

#include <atomic>
#include <thread>

#include "opencv2/opencv.hpp"
#include "zetton_common/interface/base_stream_source.h"
#include "zetton_common/thread/ring_buffer.h"

namespace zetton {
namespace common {

class CvGstStreamSource : public BaseStreamSource {
 public:
  ~CvGstStreamSource() = default;

  static bool IsSupportedExtension(const char* ext);

  bool Init(const StreamOptions& options);
  bool Init(const std::string& pipeline);
  bool Open() override;
  void Close() override;
  bool Capture(cv::Mat& frame);

  static const char* SupportedExtensions[];

 protected:
  bool BuildPipelineString();
  void CaptureFrames();

  std::string pipeline_string_;
  std::shared_ptr<cv::VideoCapture> cap_;
  bool use_custom_size_;
  bool use_custom_rate_;

  std::shared_ptr<CircularBuffer<cv::Mat>> buffer_;
  std::shared_ptr<std::thread> thread_capturing_;
  std::atomic<bool> stop_flag_{false};
};

}  // namespace common
}  // namespace zetton
