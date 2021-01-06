#pragma once

#include <cstdint>

#include "zetton_common/stream/stream_uri.h"

namespace zetton {
namespace common {

enum StreamDeviceType {
  DEVICE_DEFAULT = 0,
  DEVICE_V4L2,
  DEVICE_CSI,
  DEVICE_IP,
  DEVICE_FILE,
  DEVICE_DISPLAY,
  DEVICE_MAX_NUM
};

const char* StreamDeviceTypeToStr(StreamDeviceType type);
StreamDeviceType StreamDeviceTypeFromStr(const char* str);

enum StreamIoType { IO_INPUT = 0, IO_OUTPUT, IO_MAX_NUM };

const char* StreamIoTypeToStr(StreamIoType type);
StreamIoType StreamIoTypeFromStr(const char* str);

enum StreamFlipMethod {
  FLIP_NONE = 0,
  FLIP_COUNTERCLOCKWISE,
  FLIP_ROTATE_180,
  FLIP_CLOCKWISE,
  FLIP_HORIZONTAL,
  FLIP_UPPER_RIGHT_DIAGONAL,
  FLIP_VERTICAL,
  FLIP_UPPER_LEFT_DIAGONAL,
  FLIP_MAX_NUM
};

const char* StreamFlipMethodToStr(StreamFlipMethod flip);
StreamFlipMethod StreamFlipMethodFromStr(const char* str);

enum StreamCodec {
  CODEC_UNKNOWN = 0,
  CODEC_RAW,
  CODEC_H264,
  CODEC_H265,
  CODEC_VP8,
  CODEC_VP9,
  CODEC_MPEG2,
  CODEC_MPEG4,
  CODEC_MJPEG,
  CODEC_MAX_NUM
};

const char* StreamCodecToStr(StreamCodec codec);
StreamCodec StreamCodecFromStr(const char* str);

enum StreamPlatformType {
  PLATFORM_CPU = 0,
  PLATFORM_GPU,
  PLATFORM_JETSON,
  PLATFORM_MAX_NUM
};

const char* StreamPlatformTypeToStr(StreamPlatformType platform);
StreamPlatformType StreamPlatformTypeFromStr(const char* str);

struct StreamOptions {
 public:
  StreamUri resource;
  uint32_t width;
  uint32_t height;
  float frame_rate;
  uint32_t bit_rate;
  uint32_t num_buffers;
  bool zero_copy;
  int loop;
  bool async;

  StreamDeviceType device_type;
  StreamIoType io_type;
  StreamFlipMethod flip_method;
  StreamCodec codec;
  StreamPlatformType platform;

 public:
  StreamOptions();
};

}  // namespace common
}  // namespace zetton