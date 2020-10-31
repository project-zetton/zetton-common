#include "zetton_common/stream/gst_rtsp_stream.h"

namespace zetton {
namespace common {

GstRtspStream::GstRtspStream(const std::string& url) : BaseStream(url) {
  Init(url, "omxh264dec", "", "");
}

GstRtspStream::GstRtspStream(const std::string& url, const std::string& decoder,
                             const std::string& user_id,
                             const std::string& user_pw)
    : BaseStream(url) {
  Init(url, decoder, user_id, user_pw);
}

bool GstRtspStream::Init(const std::string& url, const std::string& decoder,
                         const std::string& user_id,
                         const std::string& user_pw) {
  if (url.find("rtsp") == 0) {
    // input url is a RTSP stream
    pipeline_ = "rtspsrc latency=0 location=" + url;
    if (!user_id.empty() && !user_pw.empty()) {
      pipeline_ += " user-id=" + user_id + " user-pw=" + user_pw;
    }
    pipeline_ += " ! rtph264depay ! queue ! h264parse ! " + decoder +
                 " ! videoconvert ! " + "appsink";
    api_ = cv::CAP_GSTREAMER;
  } else {
    // input url is a file stream
    pipeline_ = url;
    api_ = cv::CAP_ANY;
  }
  std::cout << "Open pipeline: " << pipeline_ << std::endl;
  open();
}

bool GstRtspStream::open() {
  cap_.open(pipeline_, api_);
  return cap_.isOpened();
}

bool GstRtspStream::isOpened() { return cap_.isOpened(); }

void GstRtspStream::release() { cap_.release(); }

bool GstRtspStream::read(cv::Mat& frame) {
  auto ret = cap_.read(frame);
  if (frame.empty()) {
    std::cout << "empty" << std::endl;
    return false;
  }
  return true;
}

bool GstRtspStream::read(void* frame_data) {
  cv::Mat frame;
  auto ret = cap_.read(frame);
  frame_data = frame.data;
  return ret;
}

}  // namespace common
}  // namespace zetton