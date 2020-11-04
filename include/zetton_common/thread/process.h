#pragma once

#include <sys/types.h>
#include <unistd.h>

#include <string>

namespace zetton {
namespace common {

class Process {
 public:
  static pid_t GetID();
  static pid_t GetParentID();
  static std::string GetExecutablePath();
  static std::string GetExecutableDirectory();
  static std::string GetWorkingDirectory();

  static void Fork();
};

}  // namespace common
}  // namespace zetton
