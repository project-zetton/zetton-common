#pragma once

#include <atomic>
#include <opencv2/videoio.hpp>
#include <thread>

#include "opencv2/opencv.hpp"
#include "zetton_common/interface/base_stream_output.h"
#include "zetton_common/thread/ring_buffer.h"

namespace zetton {
namespace common {

class CvGstStreamOutput : public BaseStreamOutput {
 public:
  ~CvGstStreamOutput() = default;

  static bool IsSupportedExtension(const char* ext);
  static const char* SupportedExtensions[];

  bool Init(const StreamOptions& options);
  bool Open() override;
  void Close() override;
  bool Render(void* image, uint32_t width, uint32_t height) override;

  void SetStatus(const char* str) override;

 protected:
  bool BuildPipelineString();

  std::string pipeline_string_;
  std::shared_ptr<cv::VideoWriter> writer_;

  // std::shared_ptr<std::thread> thread_rendering_;
  // std::atomic<bool> stop_flag_{false};
};

}  // namespace common
}  // namespace zetton