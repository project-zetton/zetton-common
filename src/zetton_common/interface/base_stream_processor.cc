#include "zetton_common/interface/base_stream_processor.h"

namespace zetton {
namespace common {

bool BaseStreamProcessor::Open() {
  is_streaming_ = true;
  return true;
}

void BaseStreamProcessor::Close() { is_streaming_ = false; }

}  // namespace common
}  // namespace zetton