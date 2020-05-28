cmake_minimum_required(VERSION 3.1.0)
project(phaser_common)

find_package(OpenMP REQUIRED)

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

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(OpenMP_LIBS gomp)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

cs_add_library(${PROJECT_NAME}_lib
   src/model/point-cloud.cc
   src/model/ply-point-cloud.cc
   src/model/function-value.cc
   src/model/registration-result.cc
   src/model/gmm-parameters.cc
   src/model/state.cc

   src/common/data/datasource-ros.cc
   src/common/data/datasource-ply.cc
   src/common/data/file-system-helper.cc
   src/common/data/ply-helper.cc
   src/common/spherical-projection.cc
   src/common/spherical-sampler.cc
   src/common/rotation-utils.cc
   src/common/translation-utils.cc
   src/common/statistics-manager.cc
   src/common/metric-utils.cc
   src/common/point-cloud-utils.cc
   src/common/gflags-interface.cc
   src/common/math-utils.cc
   src/common/dual-quaternion.cc
   src/common/thread-pool.cc
   src/common/base-worker.cc
   src/common/test-worker.cc
   src/common/signal-utils.cc
   src/common/core-gflags.cc

   src/common/test/testing-entrypoint.cc

   src/distribution/gaussian.cc
   src/distribution/gaussian-mixture.cc
   src/distribution/bingham.cc
   src/distribution/bingham-mle.cc
   src/distribution/bingham-mixture.cc
   src/distribution/bingham-normalization-constant.cc
   src/distribution/bingham-objective.cc
   src/distribution/bingham-opt-mle.cc

   lib/tinyply/tinyply.cc
)

#############
## TESTING ##
#############

# Gaussian
catkin_add_gtest(test_gaussian test/distribution/gaussian-test.cc)
target_link_libraries(test_gaussian ${PROJECT_NAME}_lib)

# Gaussian Mixture
catkin_add_gtest(test_gaussian_mixture test/distribution/gaussian-mixture-test.cc)
target_link_libraries(test_gaussian_mixture ${PROJECT_NAME}_lib)

# Bingham
catkin_add_gtest(test_bingham test/distribution/bingham-test.cc)
target_link_libraries(test_bingham ${PROJECT_NAME}_lib)

# Bingham Mixture
catkin_add_gtest(test_bingham_mixture test/distribution/bingham-mixture-test.cc)
target_link_libraries(test_bingham_mixture ${PROJECT_NAME}_lib)

# Function value
catkin_add_gtest(test_function_value test/model/function-value-test.cc)
target_link_libraries(test_function_value ${PROJECT_NAME}_lib)

# Thread pool and test worker
catkin_add_gtest(test_thread_pool test/common/thread-pool-test.cc)
target_link_libraries(test_thread_pool ${PROJECT_NAME}_lib)

# Utils
catkin_add_gtest(test_signal_utils test/utils/signal-utils-test.cc)
target_link_libraries(test_signal_utils ${PROJECT_NAME}_lib)

cs_install()
cs_export()
