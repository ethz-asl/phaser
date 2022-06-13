cmake_minimum_required(VERSION 3.1.0)
project(phaser_core)

if(BUILD_TYPE MATCHES CUDA)
  set(CMAKE_CUDA_COMPILER  /usr/local/cuda/bin/nvcc)
  find_package(CUDA REQUIRED)
endif()

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
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

cs_add_library(${PROJECT_NAME}_lib
   lib/SOFT/s2_legendreTransforms.c
   lib/SOFT/makeweights.c
   lib/SOFT/s2_primitive.c
   lib/SOFT/makeWigner.c
   lib/SOFT/utils_so3.c
   lib/SOFT/s2_semi_memo.c
   lib/SOFT/utils_vec_cx.c
   lib/SOFT/so3_correlate_fftw.c
   lib/SOFT/s2_cospmls.c
   lib/SOFT/soft_fftw.c
   lib/SOFT/wignerTransforms_fftw.c

   src/controller/cloud-controller.cc
   src/backend/registration/base-registration.cc
   src/backend/registration/sph-registration.cc
   src/backend/registration/sph-opt-registration.cc
   src/backend/registration/mock/sph-registration-mock-rotated.cc
   src/backend/registration/mock/sph-registration-mock-cutted.cc
   src/backend/registration/mock/sph-registration-mock-translated.cc
   src/backend/registration/mock/sph-registration-mock-transformed.cc

   src/backend/correlation/spherical-correlation.cc
   src/backend/correlation/spherical-correlation-low-pass.cc
   src/backend/correlation/spherical-correlation-laplace.cc
   src/backend/correlation/spatial-correlation.cc
   src/backend/correlation/spatial-correlation-low-pass.cc
   src/backend/correlation/spatial-correlation-laplace.cc

   src/backend/correlation/spherical-intensity-worker.cc
   src/backend/correlation/spherical-range-worker.cc
   src/backend/correlation/spherical-combined-worker.cc

   src/backend/uncertainty/base-eval.cc
   src/backend/uncertainty/z-score-eval.cc
   src/backend/uncertainty/z-score-peak-extraction.cc
   src/backend/uncertainty/gaussian-peak-based-eval.cc
   src/backend/uncertainty/gmm-peak-based-eval.cc
   src/backend/uncertainty/bingham-peak-based-eval.cc
   src/backend/uncertainty/bmm-peak-based-eval.cc
   src/backend/uncertainty/peak-based-eval.cc
   src/backend/uncertainty/phase-correlation-eval.cc
   src/backend/uncertainty/signal-analysis.cc

   src/backend/alignment/phase-aligner.cc

   src/backend/fusion/laplace-pyramid.cc
   src/backend/fusion/pyramid-struct.cc
)

if(BUILD_TYPE MATCHES CUDA)
  cuda_add_library(${PROJECT_NAME}_cuda_lib
   src/backend/correlation/spatial-correlation-cuda.cu
  )
endif()

#############
## TESTING ##
#############

if(BUILD_TYPE MATCHES CUDA)
  cuda_add_executable(${PROJECT_NAME}_driver test/driver.cc)
  cuda_add_cufft_to_target(${PROJECT_NAME}_driver)
  target_link_libraries(${PROJECT_NAME}_driver
    ${PROJECT_NAME}_lib ${PROJECT_NAME}_cuda_lib
    ${CUDA_LIBRARIES} ${CUDA_CUFFT_LIBRARIES} ${OpenMP_LIBS})
else()
  cs_add_executable(${PROJECT_NAME}_driver test/driver.cc)
  target_link_libraries(${PROJECT_NAME}_driver
    ${PROJECT_NAME}_lib ${OpenMP_LIBS})
endif()

# Sanity check.
catkin_add_gtest(test_sanity_check test/sanity-check/sanity-check-test.cc)
target_link_libraries(test_sanity_check
  ${PROJECT_NAME}_lib )

catkin_add_gtest(test_laplace_pyramid
  test/fusion/laplace-pyramid-test.cc)
target_link_libraries(test_laplace_pyramid ${PROJECT_NAME}_lib)

catkin_add_gtest(test_pyramid_struct
  test/fusion/pyramid-struct-test.cc)
target_link_libraries(test_pyramid_struct ${PROJECT_NAME}_lib)

catkin_add_gtest(test_spatial_correlation
  test/translation-alignment/spatial-correlation-test.cc)
target_link_libraries(test_spatial_correlation ${PROJECT_NAME}_lib)

cs_install()
cs_export()
