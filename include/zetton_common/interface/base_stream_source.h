#pragma once

#include "zetton_common/stream/stream_options.h"

namespace zetton {
namespace common {

enum StreamSourceType {
  SOURCE_UNKNOWN = 0,
  SOURCE_CAMERA,
  SOURCE_VIDEO,
  SOURCE_IMAGE,
  SOURCE_MAX_NUM,
};

const char* StreamProtocolTypeToStr(StreamProtocolType type);
StreamProtocolType StreamProtocolTypeFromStr(const char* str);

class BaseStreamSource {
 public:
  BaseStreamSource() = default;
  explicit BaseStreamSource(const StreamOptions& options)
      : is_streaming_(false), options_(options){};
  virtual ~BaseStreamSource() = default;

  inline bool IsStreaming() const { return is_streaming_; }
  inline uint32_t GetWidth() const { return options_.width; }
  inline uint32_t GetHeight() const { return options_.height; }
  inline uint32_t GetFrameRate() const { return options_.frame_rate; }
  inline const StreamUri& GetResource() const { return options_.resource; }
  inline const StreamOptions& GetOptions() const { return options_; }
  virtual inline StreamSourceType GetType() const {
    return StreamSourceType::SOURCE_UNKNOWN;
  }

  virtual bool Open();
  virtual void Close();

 protected:
  bool is_streaming_;
  StreamOptions options_;
};

}  // namespace common
}  // namespace zetton