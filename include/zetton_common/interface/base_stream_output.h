#pragma once

#include <vector>

#include "zetton_common/interface/base_stream_processor.h"
#include "zetton_common/stream/stream_options.h"

namespace zetton {
namespace common {

class BaseStreamOutput : public BaseStreamProcessor {
 public:
  template <typename T>
  bool Render(T* image, uint32_t width, uint32_t height) {
    return Render((void**)image, width, height);
  };
  virtual bool Render(void* image, uint32_t width, uint32_t height);

  inline void AddOutput(BaseStreamOutput* output) {
    if (output != NULL) outputs_.push_back(output);
  }
  inline uint32_t GetNumOutputs(BaseStreamOutput* output) const {
    return outputs_.size();
  }

  inline BaseStreamOutput* GetOutput(uint32_t index) const {
    return outputs_[index];
  }

  virtual void SetStatus(const char* str);

 protected:
  std::vector<BaseStreamOutput*> outputs_;
};

}  // namespace common
}  // namespace zetton