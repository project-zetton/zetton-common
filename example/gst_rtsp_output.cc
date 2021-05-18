#include <ctime>

#include "zetton_common/stream/gst_rtsp_stream_output.h"
#include "zetton_common/stream/stream_options.h"
#include "zetton_common/stream/stream_uri.h"
#include "zetton_common/util/log.h"

int main(int argc, char** argv) {
  // prepare stream url
  std::string url = "rtsp://localhost:8554/test";
  zetton::common::StreamOptions options;
  options.resource = url;
  options.platform = zetton::common::StreamPlatformType::PLATFORM_CPU;
  options.codec = zetton::common::StreamCodec::CODEC_H264;
  options.pixel_format = zetton::common::StreamPixelFormat::PIXEL_FORMAT_BGR;
  options.async = false;
  options.width = 1280;
  options.height = 720;
  options.bit_rate = 4000;
  options.frame_rate = 30;

  // init streamer
  std::shared_ptr<zetton::common::GstRtspStreamOutput> output;
  output = std::make_shared<zetton::common::GstRtspStreamOutput>();
  output->Init(options);

  // start capturing
  cv::Mat frame = cv::Mat::zeros(options.height, options.width, CV_8UC3);
  std::cout << frame.empty() << std::endl;
  while (true) {
    cv::randu(frame, cv::Scalar(0, 0, 0), cv::Scalar(255, 255, 255));
    // draw current time
    auto t = std::time(nullptr);
    auto tm = *std::localtime(&t);
    std::ostringstream oss;
    oss << std::put_time(&tm, "%Y-%m-%d %H:%M:%S");
    auto time_str = oss.str();
    // draw on frame
    cv::putText(frame, time_str, cv::Point(50, 50), cv::FONT_HERSHEY_SIMPLEX, 1,
                cv::Scalar(0, 0, 0), 5);
    // push frame to stream
    if (output->Render(frame)) {
      ROS_WARN("Send frame: %dx%d", frame.cols, frame.rows);
    }
    usleep(100000);
  }

  return 0;
}