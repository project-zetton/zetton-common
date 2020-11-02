#include "zetton_common/stream/stream_options.h"

#include <cstring>

namespace zetton {
namespace common {

const char* StreamIoTypeToStr(StreamIoType type) {
  switch (type) {
    case INPUT:
      return "input";
    case OUTPUT:
      return "output";
  }
}

StreamIoType StreamIoTypeFromStr(const char* str) {
  if (!str) return INPUT;
  for (int n = 0; n <= OUTPUT; n++) {
    const StreamIoType value = (StreamIoType)n;
    if (strcasecmp(str, StreamIoTypeToStr(value)) == 0) return value;
  }
  return INPUT;
}

const char* StreamDeviceTypeToStr(StreamDeviceType type) {
  switch (type) {
    case DEVICE_DEFAULT:
      return "default";
    case DEVICE_V4L2:
      return "v4l2";
    case DEVICE_CSI:
      return "csi";
    case DEVICE_IP:
      return "ip";
    case DEVICE_FILE:
      return "file";
    case DEVICE_DISPLAY:
      return "display";
  }
}

StreamDeviceType StreamDeviceTypeFromStr(const char* str) {
  if (!str) return DEVICE_DEFAULT;
  for (int n = 0; n <= DEVICE_DISPLAY; n++) {
    const StreamDeviceType value = (StreamDeviceType)n;
    if (strcasecmp(str, StreamDeviceTypeToStr(value)) == 0) return value;
  }
  return DEVICE_DEFAULT;
}

const char* StreamFlipMethodToStr(StreamFlipMethod flip) {
  switch (flip) {
    case FLIP_NONE:
      return "none";
    case FLIP_COUNTERCLOCKWISE:
      return "counterclockwise";
    case FLIP_ROTATE_180:
      return "rotate-180";
    case FLIP_CLOCKWISE:
      return "clockwise";
    case FLIP_HORIZONTAL:
      return "horizontal";
    case FLIP_UPPER_RIGHT_DIAGONAL:
      return "upper-right-diagonal";
    case FLIP_VERTICAL:
      return "vertical";
    case FLIP_UPPER_LEFT_DIAGONAL:
      return "upper-left-diagonal";
  }
}

StreamFlipMethod StreamFlipMethodFromStr(const char* str) {
  if (!str) return FLIP_DEFAULT;
  for (int n = 0; n <= FLIP_UPPER_LEFT_DIAGONAL; n++) {
    const StreamFlipMethod value = (StreamFlipMethod)n;
    if (strcasecmp(str, StreamFlipMethodToStr(value)) == 0) return value;
  }
  return FLIP_DEFAULT;
}

const char* StreamCodecToStr(StreamCodec codec) {
  switch (codec) {
    case CODEC_UNKNOWN:
      return "unknown";
    case CODEC_RAW:
      return "raw";
    case CODEC_H264:
      return "h264";
    case CODEC_H265:
      return "h265";
    case CODEC_VP8:
      return "vp8";
    case CODEC_VP9:
      return "vp9";
    case CODEC_MPEG2:
      return "mpeg2";
    case CODEC_MPEG4:
      return "mpeg4";
    case CODEC_MJPEG:
      return "mjpeg";
  }
}

StreamCodec StreamCodecFromStr(const char* str) {
  if (!str) return CODEC_UNKNOWN;
  for (int n = 0; n <= CODEC_MJPEG; n++) {
    const StreamCodec value = (StreamCodec)n;
    if (strcasecmp(str, StreamCodecToStr(value)) == 0) return value;
  }
  return CODEC_UNKNOWN;
}

StreamOptions::StreamOptions() {
  width = 0;
  height = 0;
  frame_rate = 0;
  bit_rate = 0;
  num_buffers = 4;
  loop = 0;
  zero_copy = true;
  io_type = StreamIoType::INPUT;
  device_type = StreamDeviceType::DEVICE_DEFAULT;
  flip_method = StreamFlipMethod::FLIP_DEFAULT;
  codec = StreamCodec::CODEC_UNKNOWN;
}

}  // namespace common
}  // namespace zetton