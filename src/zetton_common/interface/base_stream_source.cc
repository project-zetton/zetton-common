#include "zetton_common/interface/base_stream_source.h"

#include "zetton_common/util/log.h"

namespace zetton {
namespace common {

const char* StreamSourceTypeToStr(StreamSourceType type) {
  switch (type) {
    case StreamSourceType::SOURCE_UNKNOWN:
      return "unknown";
    case StreamSourceType::SOURCE_CAMERA:
      return "camera";
    case StreamSourceType::SOURCE_VIDEO:
      return "video";
    case StreamSourceType::SOURCE_IMAGE:
      return "image";
    default:
      return "unknown";
  }
}

StreamSourceType StreamSourceTypeFromStr(const char* str) {
  if (!str) return StreamSourceType::SOURCE_UNKNOWN;
  for (int n = 0; n < StreamSourceType::SOURCE_MAX_NUM; n++) {
    const StreamSourceType value = (StreamSourceType)n;
    if (strcasecmp(str, StreamSourceTypeToStr(value)) == 0) return value;
  }
  return StreamSourceType::SOURCE_UNKNOWN;
}

BaseStreamSource* BaseStreamSource::Create(const StreamOptions& options) {
  BaseStreamSource* src = nullptr;
  const StreamUri& uri = options.resource;

  // TODO(corenel): parse uri and create corresponding source

  if (!src) return nullptr;

  return src;
}

BaseStreamSource* BaseStreamSource::Create(const char* resource,
                                           const StreamOptions& options) {
  StreamOptions opt = options;
  opt.resource = resource;
  return Create(opt);
}

bool BaseStreamSource::Open() {
  is_streaming_ = true;
  return true;
}

void BaseStreamSource::Close() { is_streaming_ = false; }

}  // namespace common
}  // namespace zetton