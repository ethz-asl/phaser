cmake_minimum_required(VERSION 3.1.0)
project(phaser_pre)

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++14" COMPILER_SUPPORTS_CXX14)
if(COMPILER_SUPPORTS_CXX14)
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_FLAGS         "-Wall -Wextra -msse -msse2 -msse3 -msse4 -O3 -DNDEBUG")
  set(CMAKE_CXX_FLAGS_DEBUG   "-O0 -g")
  set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")
else()
  message(FATAL_ERROR "The compiler does not support C++14.")
endif()

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

cs_add_library(${PROJECT_NAME}_lib
  src/cloud-pre-processor.cc

  src/common/pre-processor-gflags.cc
  
  src/command/voxel-grid-cmd.cc
)

#############
## TESTING ##
#############

cs_install()
cs_export()
