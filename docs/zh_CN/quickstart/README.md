---
title: 快速入门指南
---

- [快速入门指南](#快速入门指南)
  - [预置条件](#预置条件)
  - [获取并编译 zetton-common 代码](#获取并编译-zetton-common-代码)
    - [使用 colcon 构建](#使用-colcon-构建)
    - [使用纯 CMake 构建](#使用纯-cmake-构建)
  - [在其他项目中使用 zetton-common](#在其他项目中使用-zetton-common)
    - [其他的 colcon 项目](#其他的-colcon-项目)
    - [其他的纯 CMake 项目](#其他的纯-cmake-项目)

# 快速入门指南

这份文档旨在帮助您使用 CMake 搭建 zetton-common 开发环境。我们建议每个开始使用 zetton-common 代码进行开发的人都至少完成一遍这个快速教程。

## 预置条件

在本教程中，编译并运行 zetton-common 代码需要：

1. 操作系统：Ubuntu 18.04 或更高版本

2. C++ 编译器：支持 C++14 的 C++ 编译器，例如 g++ 的 `7.5.0` 或更高版本

3. [Git](https://git-scm.com)：用于从 GitHub 上下载 zetton-common 代码

4. [CMake](https://cmake.org)：用于构建 zetton-common 代码，要求 `3.5` 或更高版本

5. [abseil-cpp](https://github.com/abseil/abseil-cpp): 用于 zetton-common 代码中的一些工具类，建议使用 `lts_2021_11_02` 或更高版本

6. [fmt](https://github.com/fmtlib/fmt): 用于 zetton-common 代码中的格式化输出，建议使用 `9.0.0` 或更高版本

7. [colcon](https://colcon.readthedocs.io/en/released/): （可选）用于构建 zetton-common 代码，建议使用 `0.3.0` 或更高版本

上述所有依赖库，可以参考 [zetton-docker](https://github.com/project-zetton/zetton-docker) 中的 `Dockerfile` 进行安装。或者直接使用 zetton-docker 提供的 Docker 容器来运行 zetton-common 代码。

## 获取并编译 zetton-common 代码

### 使用 colcon 构建

```bash
# 创建工作空间
mkdir -p ~/zetton_ws/src
cd ~/zetton_ws

# 获取 zetton-common 代码
git clone https://github.com/project-zetton/zetton-common.git ~/zetton_ws/src/zetton-common

# 编译 zetton-common 代码
colcon build
```

编译完成后，将生成如下目录：

- `~/zetton_ws/install`: 编译生成的库文件和头文件
- `~/zetton_ws/build`: 编译生成的可执行文件
- `~/zetton_ws/log`: 编译过程中的日志文件。

关于 colcon 的更多用法，可以参考 [colcon 文档](https://colcon.readthedocs.io/en/released/)。

### 使用纯 CMake 构建

```bash
# 获取 zetton-common 代码
git clone https://github.com/project-zetton/zetton-common.git

# 编译 zetton-common 代码
mkdir -p zetton-common/build
cd zetton-common/build
cmake ..
make

# （可选）安装 zetton-common 代码
sudo make install
```

## 在其他项目中使用 zetton-common

### 其他的 colcon 项目

在其他的 colcon 项目中使用 zetton-common，需要：

1. 在 `package.xml` 中添加如下内容：

   ```xml
   <depend>zetton_common</depend>
   ```

2. 在 `CMakeLists.txt` 中添加如下内容：

   ```cmake
   find_package(zetton_common REQUIRED)

   target_link_libraries(${PROJECT_NAME}
     PRIVATE
       zetton_common::zetton_common
   )
   ```

3. 在代码中包含对应的头文件：

   ```cpp
   #include "zetton_common/util/log.h"

   void foo() {
     AINFO_F("hello world");
   }
   ```

在编译其他项目时，colcon 会自动检测到 zetton-common 的依赖关系，并自动编译 zetton-common 代码。因此，不需要手动编译 zetton-common 代码。

### 其他的纯 CMake 项目

若其他项目使用纯 CMake 进行构建，则在修改`CMakeLists.txt`文件前，需要提前进行如下步骤（任选其一）：

- 手动编译并安装 zetton-common 代码至系统目录，例如 `/usr/local`

- 手动编译 zetton-common 代码，并在其他项目的 `CMakeLists.txt` 中指定 zetton-common 的安装路径

   ```cmake
   set(ZETTON_COMMON_INSTALL_DIR "/path/to/zetton-common/build")
   find_package(zetton_common REQUIRED PATHS ${ZETTON_COMMON_INSTALL_DIR})

   target_link_libraries(${PROJECT_NAME}
     PRIVATE
       zetton_common::zetton_common
   )
   ```

- 手动编译 zetton-common 代码，并将 zetton-common 的安装路径添加到 `CMAKE_PREFIX_PATH` 环境变量中

   ```bash
   export CMAKE_PREFIX_PATH="/path/to/zetton-common/build:$CMAKE_PREFIX_PATH"
   ```

- 手动编译 zetton-common 代码，得到动态连接库文件 `libzetton_common.so` 与对应的头文件，将其链接到其他项目中：

   ```cmake
   include_directories("/path/to/zetton-common/include")
   target_link_libraries(${PROJECT_NAME}
     PRIVATE
       "/path/to/zetton-common/build/libzetton_common.so"
   )
   ```

- 将 zetton-common 代码作为子项目添加到其他项目中

   ```cmake
   add_subdirectory("/path/to/zetton-common")
   target_link_libraries(${PROJECT_NAME}
     PRIVATE
       zetton_common::zetton_common
   )
   ```
