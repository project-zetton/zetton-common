#pragma once

#include <string>

namespace zetton {
namespace common {

enum StreamProtocolType {
  PROTOCOL_DEFAULT = 0,
  PROTOCOL_V4L2,
  PROTOCOL_CSI,
  PROTOCOL_RTP,
  PROTOCOL_RTSP,
  PROTOCOL_FILE,
  PROTOCOL_DISPLAY,
  PROTOCOL_MAX_NUM,
};

static const char* StreamProtocolTypeToStr(StreamProtocolType type);
static StreamProtocolType StreamProtocolTypeFromStr(const char* str);

struct StreamUri {
  std::string string;
  StreamProtocolType protocol;
  std::string location;
  std::string extension;
  int port;

  StreamUri();
  StreamUri(const char* uri);
  bool Parse(const char* uri);
  void Print(const char* prefix = "") const;
  inline const char* c_str() const { return string.c_str(); }
  operator const char*() const { return string.c_str(); }
  operator std::string() const { return string; }
  inline void operator=(const char* uri) { Parse(uri); }
  inline void operator=(const std::string& uri) { Parse(uri.c_str()); }
};

}  // namespace common
}  // namespace zetton