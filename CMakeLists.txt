cmake_minimum_required(VERSION 3.5)

# ##############################################################################
# CMake Project ##
# ##############################################################################

# include useful macros
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/util.cmake)

# read metadata from package.xml and setup project
zetton_cc_metadata()

# set project name and version
project(
  ${extracted_name}
  VERSION ${extracted_version}
  LANGUAGES C CXX)

# =========#
# Settings #
# =========#

# introduce default building settings
zetton_cc_settings()

# =============#
# Dependencies #
# =============#

# ----------------------#
# Third-party libraries #
# ----------------------#

find_package(Threads REQUIRED)
# find_package(OpenCV 4 REQUIRED)
find_package(absl REQUIRED)
find_package(fmt REQUIRED)

# ##############################################################################
# Build #
# ##############################################################################

# ==============#
# Build targets #
# ==============#

include(GenerateExportHeader)
include(CMakePackageConfigHelpers)
include(GNUInstallDirs)

# ---------------#
# Library target #
# ---------------#

# find all source files
file(GLOB_RECURSE src_files ${PROJECT_SOURCE_DIR}/src/*.cc
     ${PROJECT_SOURCE_DIR}/src/*/*.cc)

# common library
zetton_cc_library(
  NAME
  ${PROJECT_NAME}
  SRCS
  ${src_files}
  INCLUDES
  ${OpenCV_INCLUDE_DIRS}
  DEPS
  Threads::Threads
  ${OpenCV_LIBS}
  fmt::fmt
  absl::time
  absl::flags
  absl::flags_parse)

# ------------------#
# Executable targets #
# ------------------#

zetton_cc_apps()
zetton_cc_examples()

# -------------#
# Test targets #
# -------------#

zetton_cc_tests("common")
