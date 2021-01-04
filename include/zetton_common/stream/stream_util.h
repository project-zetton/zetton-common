#pragma once

#include <cstring>
#include <sstream>
#include <string>

#include "zetton_common/stream/stream_options.h"
#include "zetton_common/util/log.h"

namespace zetton {
namespace common {

inline bool IsGstSupportedExtension(const char* ext, const char** supported) {
  if (!ext) return false;

  uint32_t ext_count = 0;
  while (true) {
    if (!supported[ext_count]) break;
    if (strcasecmp(supported[ext_count], ext) == 0) return true;
    ext_count++;
  }

  return false;
}

inline bool BuildGstCapturingPipelineString(
    const StreamUri& uri, StreamOptions& options, std::string& pipeline_string,
    const bool& use_custom_size = false, const bool& use_custom_rate = false) {
  std::ostringstream ss;

  // determine the source protocol
  if (uri.protocol == StreamProtocolType::PROTOCOL_FILE) {
    // build pipeline for file source
    options.device_type = StreamDeviceType::DEVICE_FILE;
    // source
    ss << "filesrc location=" << options.resource.location << " ! ";
    if (uri.extension == "mkv") {
      ss << "matroskademux ! ";
    } else if (uri.extension == "mp4" || uri.extension == "qt") {
      ss << "qtdemux ! ";
    } else if (uri.extension == "flv") {
      ss << "flvdemux ! ";
    } else if (uri.extension == "avi") {
      ss << "avidemux ! ";
    } else if (uri.extension != "h264" && uri.extension != "h265") {
      ROS_ERROR("Unsupported video file extension (%s)", uri.extension.c_str());
      return false;
    }
    // parser
    ss << "queue ! ";
    if (options.codec == StreamCodec::CODEC_H264) {
      ss << "h264parse ! ";
    } else if (options.codec == StreamCodec::CODEC_H265) {
      ss << "h265parse ! ";
    } else if (options.codec == StreamCodec::CODEC_MPEG2) {
      ss << "mpegvideoparse ! ";
    } else if (options.codec == StreamCodec::CODEC_MPEG4) {
      ss << "mpeg4videoparse ! ";
    }
  } else if (uri.protocol == StreamProtocolType::PROTOCOL_RTP) {
    // build pipeline for RTP source
    options.device_type = StreamDeviceType::DEVICE_IP;
    // check port number
    if (uri.port <= 0) {
      ROS_ERROR("Invalid RTP port (%i)", uri.port);
      return false;
    }
    // source
    ss << "udpsrc port=" << uri.port;
    ss << " multicast-group=" << uri.location << " auto-multicast=true";
    ss << " caps=\""
       << "application/"
          "x-rtp,media=(string)video,clock-rate=(int)90000,encoding-name=("
          "string)";
    // demux
    if (options.codec == StreamCodec::CODEC_H264) {
      ss << "H264\" ! rtph264depay ! h264parse ! ";
    } else if (options.codec == StreamCodec::CODEC_H265) {
      ss << "H265\" ! rtph265depay ! h265parse ! ";
    } else if (options.codec == StreamCodec::CODEC_VP8) {
      ss << "VP8\" ! rtpvp8depay ! ";
    } else if (options.codec == StreamCodec::CODEC_VP9) {
      ss << "VP9\" ! rtpvp9depay ! ";
    } else if (options.codec == StreamCodec::CODEC_MPEG2) {
      ss << "MP2T\" ! rtpmp2tdepay ! ";
    } else if (options.codec == StreamCodec::CODEC_MPEG4) {
      ss << "MP4V-ES\" ! rtpmp4vdepay ! ";
    } else if (options.codec == StreamCodec::CODEC_MJPEG) {
      ss << "JPEG\" ! rtpjpegdepay ! ";
    }
  } else if (uri.protocol == StreamProtocolType::PROTOCOL_RTSP) {
    // build pipeline for RTSP source
    options.device_type = StreamDeviceType::DEVICE_IP;
    // source
    ss << "rtspsrc location=" << uri.string;
    ss << " ! queue ! ";
    // demux
    if (options.codec == StreamCodec::CODEC_H264) {
      ss << "rtph264depay ! h264parse ! ";
    } else if (options.codec == StreamCodec::CODEC_H265) {
      ss << "rtph265depay ! h265parse ! ";
    } else if (options.codec == StreamCodec::CODEC_VP8) {
      ss << "rtpvp8depay ! ";
    } else if (options.codec == StreamCodec::CODEC_VP9) {
      ss << "rtpvp9depay ! ";
    } else if (options.codec == StreamCodec::CODEC_MPEG2) {
      ss << "rtpmp2tdepay ! ";
    } else if (options.codec == StreamCodec::CODEC_MPEG4) {
      ss << "rtpmp4vdepay ! ";
    } else if (options.codec == StreamCodec::CODEC_MJPEG) {
      ss << "rtpjpegdepay ! ";
    }
  } else {
    ROS_ERROR("Unsupported protocol (%s)",
              StreamProtocolTypeToStr(uri.protocol));
    return false;
  }

  // determine the decoder
  if (options.platform == StreamPlatformType::PLATFORM_CPU) {
    // use libav for decoding on CPU
    if (options.codec == StreamCodec::CODEC_H264) {
      ss << "avdec_h264 ! ";
    } else if (options.codec == StreamCodec::CODEC_H265) {
      ss << "avdec_h265 ! ";
    } else if (options.codec == StreamCodec::CODEC_VP8) {
      ss << "avdev_vp8 ! ";
    } else if (options.codec == StreamCodec::CODEC_VP9) {
      ss << "avdec_vp9 ! ";
    } else if (options.codec == StreamCodec::CODEC_MPEG2) {
      ss << "avdec_mpeg2video ! ";
    } else if (options.codec == StreamCodec::CODEC_MPEG4) {
      ss << "avdec_mpeg4 ! ";
    } else if (options.codec == StreamCodec::CODEC_MJPEG) {
      ss << "avdec_mjpeg ! ";
    } else {
      ROS_ERROR("Unsupported codec (%s)", StreamCodecToStr(options.codec));
      return false;
    }
  } else if (options.platform == StreamPlatformType::PLATFORM_GPU) {
    // use Video Codec SDK for GPU-accelerated decoding
    if (options.codec == StreamCodec::CODEC_H264 ||
        options.codec == StreamCodec::CODEC_H265 ||
        options.codec == StreamCodec::CODEC_MPEG2 ||
        options.codec == StreamCodec::CODEC_MPEG4 ||
        options.codec == StreamCodec::CODEC_MJPEG) {
      // FIXME gldownload doesn't work on headless mode
      ss << "nvdec ! gldownload ! ";
    } else {
      ROS_ERROR("Unsupported codec (%s)", StreamCodecToStr(options.codec));
      return false;
    }
  } else if (options.platform == StreamPlatformType::PLATFORM_JETSON) {
    // use omx for hardware-accelerated decoding on Jetson devices
    if (options.codec == StreamCodec::CODEC_H264) {
      ss << "omxh264dec ! ";
    } else if (options.codec == StreamCodec::CODEC_H265) {
      ss << "omxh265dec ! ";
    } else if (options.codec == StreamCodec::CODEC_VP8) {
      ss << "omxvp8dec ! ";
    } else if (options.codec == StreamCodec::CODEC_VP9) {
      ss << "omxvp9dec ! ";
    } else if (options.codec == StreamCodec::CODEC_MPEG2) {
      ss << "omxmpeg2videodec ! ";
    } else if (options.codec == StreamCodec::CODEC_MPEG4) {
      ss << "omxmpeg4videodec ! ";
    } else if (options.codec == StreamCodec::CODEC_MJPEG) {
      ss << "nvjpegdec ! ";
    } else {
      ROS_ERROR("Unsupported codec (%s)", StreamCodecToStr(options.codec));
      return false;
    }
  }

  // resize or flip if requested
  if (use_custom_size || options.flip_method != StreamFlipMethod::FLIP_NONE) {
    if (options.platform == StreamPlatformType::PLATFORM_CPU ||
        options.platform == StreamPlatformType::PLATFORM_GPU) {
      ss << "videoconvert";
      // flip on CPU
      if (options.flip_method != StreamFlipMethod::FLIP_NONE) {
        ss << " ! videoflip flip-method=(string)"
           << StreamFlipMethodToStr(options.flip_method);
      }
      // rescale
      if (options.width != 0 && options.height != 0) {
        ss << " ! videoscale ! video/x-raw";
        ss << ", width=(int)" << options.width << ", height=(int)"
           << options.height << ", format=(string)BGRx";
      }
      ss << " ! ";
    } else if (options.platform == StreamPlatformType::PLATFORM_JETSON) {
      // TODO check the validity on GPU
      ss << "nvvidconv";
      // flip on GPU
      if (options.flip_method != StreamFlipMethod::FLIP_NONE) {
        ss << " flip-method=" << (int)options.flip_method;
      }
      // rescale
      ss << " ! video/x-raw";
      if (options.width != 0 && options.height != 0) {
        ss << ", width=(int)" << options.width << ", height=(int)"
           << options.height << ", format=(string)BGRx";
      }
      ss << " ! ";
    }
  } else {
    ss << "videoconvert ! video/x-raw ! ";
  }

  // rate-limit if requested
  if (use_custom_rate)
    ss << "videorate drop-only=true max-rate=" << (int)options.frame_rate
       << " ! ";

  // add the app sink
  ss << "appsink";  // wait-on-eos=false;

  pipeline_string = ss.str();
  ROS_INFO("Built pipeline string: %s", pipeline_string.c_str());
  return true;
};

inline bool BuildGstOutputtingPipelineString(const StreamUri& uri,
                                             StreamOptions& options,
                                             std::string& pipeline_string) {
  std::ostringstream ss;

  // setup appsrc input element
  ss << "appsrc name=mysource is-live=true do-timestamp=true format=3 ! ";

  // set default bitrate (if needed)
  if (options.bit_rate == 0) {
    options.bit_rate = 4000000;
  }

  // determine the requested protocol to use
  if (options.codec == videoOptions::CODEC_H264) {
    ss << "omxh264enc bitrate=" << options.bit_rate << " ! video/x-h264 !  ";
  } else if (options.codec == StreamCodec::CODEC_H265) {
    ss << "omxh265enc bitrate=" << options.bit_rate << " ! video/x-h265 ! ";
  } else if (options.codec == StreamCodec::CODEC_VP8) {
    ss << "omxvp8enc bitrate=" << options.bit_rate << " ! video/x-vp8 ! ";
  } else if (options.codec == StreamCodec::CODEC_VP9) {
    ss << "omxvp9enc bitrate=" << options.bit_rate << " ! video/x-vp9 ! ";
  } else if (options.codec == StreamCodec::CODEC_MJPEG) {
    ss << "nvjpegenc ! image/jpeg ! ";
  } else {
    ROS_ERROR("Unsupported codec requested (%s)\n",
              StreamCodecToStr(options.codec));
    ROS_ERROR("supported encoder codecs are:\n");
    ROS_ERROR("  * h264\n");
    ROS_ERROR("  * h265\n");
    ROS_ERROR("  * vp8\n");
    ROS_ERROR("  * vp9\n");
    ROS_ERROR("  * mjpeg\n");
  }

  // to be continued

  return false;
};

}  // namespace common
}  // namespace zetton