#pragma once

#include <cstdint>

#include "zetton_common/stream/stream_uri.h"

namespace zetton {
namespace common {

enum StreamDeviceType {
  DEVICE_DEFAULT = 0, /**< Unknown interface type. */
  DEVICE_V4L2,        /**< V4L2 webcam (e.g. `/dev/video0`) */
  DEVICE_CSI,         /**< MIPI CSI camera */
  DEVICE_IP,          /**< IP-based network stream (e.g. RTP/RTSP) */
  DEVICE_FILE,   /**< Disk-based stream from a file or directory of files */
  DEVICE_DISPLAY /**< OpenGL output stream rendered to an attached display */
};

static const char* StreamDeviceTypeToStr(StreamDeviceType type);
static StreamDeviceType StreamDeviceTypeFromStr(const char* str);

enum StreamIoType {
  INPUT = 0, /**< Input stream (e.g. camera, video/image file, ect.) */
  OUTPUT,    /**< Output stream (e.g. display, video/image file, ect.) */
};

static const char* StreamIoTypeToStr(StreamIoType type);
static StreamIoType StreamIoTypeFromStr(const char* str);

enum StreamFlipMethod {
  FLIP_NONE = 0,             /**< Identity (no rotation) */
  FLIP_COUNTERCLOCKWISE,     /**< Rotate counter-clockwise 90 degrees */
  FLIP_ROTATE_180,           /**< Rotate 180 degrees */
  FLIP_CLOCKWISE,            /**< Rotate clockwise 90 degrees */
  FLIP_HORIZONTAL,           /**< Flip horizontally */
  FLIP_UPPER_RIGHT_DIAGONAL, /**< Flip across upper right/lower left diagonal
                              */
  FLIP_VERTICAL,             /**< Flip vertically */
  FLIP_UPPER_LEFT_DIAGONAL,  /**< Flip across upper left/lower right diagonal
                              */
  FLIP_DEFAULT = FLIP_NONE   /**< Default setting (none) */
};

static const char* StreamFlipMethodToStr(StreamFlipMethod flip);
static StreamFlipMethod StreamFlipMethodFromStr(const char* str);

enum StreamCodec {
  CODEC_UNKNOWN = 0, /**< Unknown/unsupported codec */
  CODEC_RAW,         /**< Uncompressed (e.g. RGB) */
  CODEC_H264,        /**< H.264 */
  CODEC_H265,        /**< H.265 */
  CODEC_VP8,         /**< VP8 */
  CODEC_VP9,         /**< VP9 */
  CODEC_MPEG2,       /**< MPEG2 (decode only) */
  CODEC_MPEG4,       /**< MPEG4 (decode only) */
  CODEC_MJPEG        /**< MJPEG */
};

static const char* StreamCodecToStr(StreamCodec codec);
static StreamCodec StreamCodecFromStr(const char* str);

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
  StreamDeviceType device_type;
  StreamIoType io_type;
  StreamFlipMethod flip_method;
  StreamCodec codec;

 public:
  StreamOptions();
};

}  // namespace common
}  // namespace zetton