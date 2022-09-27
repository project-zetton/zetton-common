#include <algorithm>
#include <string>

namespace zetton {
namespace common {

static std::string ToLower(const std::string& str) {
  std::string dst;
  dst.resize(str.size());
  std::transform(str.begin(), str.end(), dst.begin(), ::tolower);
  return dst;
}

}  // namespace common
}  // namespace zetton
