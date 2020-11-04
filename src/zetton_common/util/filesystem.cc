#include "zetton_common/util/filesystem.h"

#include <dirent.h>
#include <fcntl.h>
#include <glob.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <climits>
#include <cstddef>
#include <fstream>
#include <sstream>
#include <string>

#include "zetton_common/thread/process.h"
#include "zetton_common/util/log.h"

namespace zetton {
namespace common {

bool FileExists(const std::string& path, uint32_t mask) {
  return FileIsType(path, mask);
}

uint32_t GetFileType(const std::string& path) {
  if (path.size() == 0) return FILE_MISSING;

  struct stat file_stat;
  const int result = stat(path.c_str(), &file_stat);

  if (result == -1) {
    return FILE_MISSING;
  }

  if (S_ISREG(file_stat.st_mode))
    return FILE_REGULAR;
  else if (S_ISDIR(file_stat.st_mode))
    return FILE_DIR;
  else if (S_ISLNK(file_stat.st_mode))
    return FILE_LINK;
  else if (S_ISCHR(file_stat.st_mode))
    return FILE_CHAR;
  else if (S_ISBLK(file_stat.st_mode))
    return FILE_BLOCK;
  else if (S_ISFIFO(file_stat.st_mode))
    return FILE_FIFO;
  else if (S_ISSOCK(file_stat.st_mode))
    return FILE_SOCKET;

  return FILE_MISSING;
}

bool FileIsType(const std::string& path, uint32_t mask) {
  if (path.size() == 0) return false;
  const uint32_t type = GetFileType(path);
  if (type == FileTypes::FILE_MISSING) return false;
  if (mask == 0) return true;
  if ((type & mask) != type) return false;
  return true;
}

size_t GetFileSize(const std::string& path) {
  if (path.size() == 0) return 0;

  struct stat fileStat;

  const int result = stat(path.c_str(), &fileStat);

  if (result == -1) {
    ROS_ERROR("%s does not exist.", path.c_str());
    return 0;
  }

  return fileStat.st_size;
}

std::string GetFileExtension(const std::string& path) {
  const std::string::size_type dotIdx = path.find_last_of(".");

  if (dotIdx == std::string::npos) return "";

  std::string ext = path.substr(dotIdx + 1);
  std::transform(ext.begin(), ext.end(), ext.begin(), tolower);
  return ext;
}

bool FileHasExtension(const std::string& path, const std::string& extension) {
  std::vector<std::string> extensions;
  extensions.push_back(extension);
  return FileHasExtension(path, extensions);
}

bool FileHasExtension(const std::string& path, const char** extensions) {
  if (!extensions) return false;

  std::vector<std::string> extList;
  uint32_t extCount = 0;

  while (true) {
    if (!extensions[extCount]) break;
    extList.push_back(extensions[extCount]);
    extCount++;
  }

  return FileHasExtension(path, extList);
}

bool FileHasExtension(const std::string& path,
                      const std::vector<std::string>& extensions) {
  const std::string pathExtension = GetFileExtension(path);
  const size_t numExtensions = extensions.size();

  if (pathExtension.size() == 0) return false;
  if (numExtensions == 0) return false;
  for (size_t n = 0; n < numExtensions; n++) {
    if (extensions[n].size() == 0) continue;
    if (strcasecmp(pathExtension.c_str(), extensions[n].c_str()) == 0)
      return true;
  }

  return false;
}

std::string GetDirPath(const std::string& filename) {
  const std::string::size_type slashIdx = filename.find_last_of("/");
  if (slashIdx == std::string::npos || slashIdx == 0) return filename;
  return filename.substr(0, slashIdx + 1);
}

std::string JoinPath(const std::string& a, const std::string& b) {
  if (a.size() == 0) return b;
  if (b.size() == 0) return a;
  // check if there is already a path separator at the end
  const char lastChar = a[a.size() - 1];
  if (lastChar == '/' || lastChar == '\\') return a + b;
  return a + "/" + b;
}

std::string GetWorkingDirectory() { return Process::GetWorkingDirectory(); }

std::string GetAbsolutePath(const std::string& relative_path) {
  if (relative_path.size() != 0) {
    const char first_char = relative_path[0];

    if (first_char == '/' || first_char == '\\' || first_char == '~')
      return relative_path;
  }
  return JoinPath(GetWorkingDirectory(), relative_path);
}

}  // namespace common
}  // namespace zetton