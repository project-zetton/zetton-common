#include "zetton_common/stream/cv_gst_stream_output.h"

#include <memory>
#include <string>

#include "zetton_common/stream/stream_options.h"
#include "zetton_common/stream/stream_util.h"
#include "zetton_common/util/log.h"

namespace zetton {
namespace common {

const char* CvGstStreamOutput::SupportedExtensions[] = {
    "mkv", "mp4", "qt", "flv", "avi", "h264", "h265", NULL};

bool CvGstStreamOutput::IsSupportedExtension(const char* ext) {
  return IsGstSupportedExtension(ext, SupportedExtensions);
}

bool CvGstStreamOutput::Init(const StreamOptions& options) {
  options_ = options;

  // check for default codec
  if (options_.codec == StreamCodec::CODEC_UNKNOWN) {
    ROS_ERROR("Codec not specified, defaulting to H.264\n");
    options_.codec = StreamCodec::CODEC_H264;
  }

  // check if default framerate is needed
  if (options_.frame_rate <= 0) options_.frame_rate = 30;

  // build pipeline string
  if (!BuildPipelineString()) {
    ROS_ERROR("Failed to build pipeline string\n");
    return false;
  }

  // create pipeline
  writer_ =
      std::make_shared<cv::VideoWriter>(pipeline_string_, cv::CAP_GSTREAMER);

  return true;
};

bool CvGstStreamOutput::Open() {
  is_streaming_ = true;
  return true;
};

void CvGstStreamOutput::Close() { is_streaming_ = false; };

bool CvGstStreamOutput::Render(void* image, uint32_t width, uint32_t height) {
  return false;
};

void CvGstStreamOutput::SetStatus(const char* str){};

bool BuildPipelineString() { return false; };

}  // namespace common
}  // namespace zetton