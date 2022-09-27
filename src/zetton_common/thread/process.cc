#include "zetton_common/thread/process.h"

#include <cstring>

#include "zetton_common/util/filesystem.h"

namespace zetton {
namespace common {

pid_t Process::GetID() { return getpid(); }

pid_t Process::GetParentID() { return getppid(); }

std::string Process::GetExecutablePath() {
  char buf[512];
  std::memset(buf, 0, sizeof(buf));  // readlink() does not NULL-terminate
  const ssize_t size = readlink("/proc/self/exe", buf, sizeof(buf));
  if (size <= 0) return "";
  return std::string(buf);
}

std::string Process::GetExecutableDirectory() {
  const std::string path = GetExecutablePath();
  if (path.length() == 0) return "";
  return GetDirPath(path);
}

std::string Process::GetWorkingDirectory() {
  char buf[1024];
  char *str = getcwd(buf, sizeof(buf));
  if (!str) return "";
  return buf;
}

void Process::Fork() { fork(); }

}  // namespace common
}  // namespace zetton
