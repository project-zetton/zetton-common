#pragma once

#include <string>
#include <vector>

namespace zetton {
namespace common {

enum FileTypes {
  FILE_MISSING = 0,
  FILE_REGULAR = (1 << 0),
  FILE_DIR = (1 << 1),
  FILE_LINK = (1 << 2),
  FILE_CHAR = (1 << 3),
  FILE_BLOCK = (1 << 4),
  FILE_FIFO = (1 << 5),
  FILE_SOCKET = (1 << 6)
};

bool FileExists(const std::string& path, uint32_t mask = 0);
uint32_t GetFileType(const std::string& path);
bool FileIsType(const std::string& path, uint32_t mask);
size_t GetFileSize(const std::string& path);

std::string GetFileExtension(const std::string& path);
bool FileHasExtension(const std::string& path, const std::string& extension);
bool FileHasExtension(const std::string& path,
                      const std::vector<std::string>& extensions);
bool FileHasExtension(const std::string& path, const char** extensions);

std::string JoinPath(const std::string& a, const std::string& b);
std::string GetDirPath(const std::string& path);
std::string GetWorkingDirectory();
std::string GetAbsolutePath(const std::string& relative_path);

}  // namespace common
}  // namespace zetton
