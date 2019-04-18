 cmake_minimum_required(VERSION 3.1.0)
 project(packlo)

 set(CMAKE_CXX_STANDARD 14)
 set(CMAKE_CXX_FLAGS         "-Wall -Wextra -msse -msse2 -msse3 -msse4")
 set(CMAKE_CXX_FLAGS_DEBUG   "-O0 -g")
 set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")


 find_package(catkin_simple REQUIRED)
 catkin_simple(ALL_DEPS_REQUIRED)

 cs_add_executable(${PROJECT_NAME}-node tools/packlo-node.cc)

 cs_install()
 cs_export()
