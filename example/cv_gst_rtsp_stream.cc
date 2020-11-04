#include "zetton_common/stream/cv_gst_stream_source.h"
#include "zetton_common/stream/stream_options.h"
#include "zetton_common/stream/stream_uri.h"
#include "zetton_common/util/log.h"

int main(int argc, char** argv) {
  // prepare stream url
  std::string url =
      "rtsp://freja.hiof.no:1935/rtplive/_definst_/hessdalen02.stream";
  zetton::common::StreamOptions options;
  options.resource = url;
  options.platform = zetton::common::StreamPlatformType::PLATFORM_CPU;
  options.codec = zetton::common::StreamCodec::CODEC_H264;

  // init streamer
  std::shared_ptr<zetton::common::CvGstStreamSource> source;
  source = std::make_shared<zetton::common::CvGstStreamSource>();
  source->Init(options);

  // start capturing
  while (true) {
    cv::Mat frame;
    if (source->Capture(frame)) {
      ROS_INFO("User: %dx%d", frame.cols, frame.rows);
    }
    usleep(100000);
  }

  return 0;
}