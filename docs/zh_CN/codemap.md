---
title: 代码地图
---

- [代码地图](#代码地图)
  - [目录结构](#目录结构)
  - [模块介绍](#模块介绍)

# 代码地图

本文档旨在帮助您了解 zetton-common 代码的结构和功能。

## 目录结构

zetton-common 代码的目录结构如下：

```bash
tree -L 3
.
├── cmake/
│   ├── Config.cmake.in
│   └── util.cmake
├── CMakeLists.txt
├── docs/
│   ├── en/
│   └── zh_CN/
│       ├── quickstart/
│       └── specs/
├── include/
│   └── zetton_common/
│       ├── interface/
│       ├── log/
│       ├── thread/
│       ├── time/
│       └── util/
├── LICENSE
├── package.xml
├── README.md
├── src/
│   └── zetton_common/
│       ├── log/
│       ├── thread/
│       ├── time/
│       └── util/
├── test/
│   └── common/
└── tool/
    ├── githooks/
    │   └── commit-msg*
    └── install_git_hooks.sh*
```

其中：

- `cmake/` 与 `CMakeLists.txt`：CMake 构建相关的文件
- `docs/`：文档目录
- `include/`：头文件目录
- `src/`：源代码目录
- `test/`：测试代码目录
- `tool/`：工具脚本目录
- `LICENSE`：软件包许可证
- `README.md`：软件包说明文档
- `package.xml`：软件包描述文件，用于 colcon 构建

## 模块介绍

zetton-common 代码包含如下模块：

- `log`：日志模块，该模块基于 [fmt](https://github.com/fmtlib/fmt) 与 [loguru](https://github.com/emilk/loguru) 实现，提供了一套简单易用的日志接口，支持日志级别、日志文件、日志颜色等功能，并同时支持流式输出与格式化输出

- `thread`：线程模块，该模块提供了对于多线程、多进程的封装（待重构）

- `time`：时间模块，该模块提供了对于时间相关的封装，便于在不同平台上使用统一的时间接口

- `util`：实用工具模块

  - `filesystem`：文件系统模块，该模块提供了对于文件系统的封装，便于在不同平台上使用统一的文件系统接口

  - `log`: 日志模块，引用前述的日志模块，用于后向兼容

  - `macros`：宏定义模块，该模块提供了一些常用的宏定义，用于简化代码

  - `perf`：性能分析模块，该模块提供了对于性能分析的封装，支持运行时间的分析

  - `register`：注册模块，该模块提供了对于注册的封装，支持注册类、注册函数等

  - `string`：字符串模块，该模块提供了对于字符串的封装，支持字符串分割、字符串替换等
