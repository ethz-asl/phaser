cmake_minimum_required(VERSION 3.1.0)
project(phaser_core)

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

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

cs_add_library(${PROJECT_NAME}_lib
   lib/SOFT/csecond.c
   lib/SOFT/s2_legendreTransforms.c
   lib/SOFT/soft_fftw_nt.c
   lib/SOFT/wrap_s2_rotate_fftw.c
   lib/SOFT/makeweights.c
   lib/SOFT/s2_primitive.c
   lib/SOFT/soft_fftw_pc.c
   lib/SOFT/wrap_soft_fftw.c
   lib/SOFT/makeWigner.c
   lib/SOFT/s2_semi_fly.c
   lib/SOFT/utils_so3.c
   lib/SOFT/wrap_soft_fftw_cor2.c
   lib/SOFT/rotate_so3_fftw.c
   lib/SOFT/s2_semi_memo.c
   lib/SOFT/utils_vec_cx.c
   lib/SOFT/rotate_so3_utils.c
   lib/SOFT/so3_correlate_fftw.c
   lib/SOFT/wignerTransforms.c
   lib/SOFT/s2_cospmls.c
   lib/SOFT/soft_fftw.c
   lib/SOFT/wignerTransforms_fftw.c

   src/controller/distributor.cc
   src/backend/registration/base-registration.cc
   src/backend/registration/sph-registration.cc
   src/backend/registration/mock/sph-registration-mock-rotated.cc
   src/backend/registration/mock/sph-registration-mock-cutted.cc
   src/backend/registration/mock/sph-registration-mock-translated.cc
   src/backend/registration/mock/sph-registration-mock-transformed.cc
   src/backend/correlation/spherical-correlation.cc
   src/backend/correlation/z-score-eval.cc
   src/backend/alignment/range-based-aligner.cc
   src/backend/alignment/phase-aligner.cc
   src/backend/alignment/feature-aligner.cc
)

#############
## TESTING ##
#############

# Sanity check.
catkin_add_gtest(test_sanity_check test/sanity-check/sanity-check-test.cc)
target_link_libraries(test_sanity_check
  ${PROJECT_NAME}_lib)

# Rotation estimation.
catkin_add_gtest(test_rotation_alignment
  test/rotation-alignment/rotation-alignment-test.cc)
target_link_libraries(test_rotation_alignment
  ${PROJECT_NAME}_lib)
phaser_import_test_maps(test_rotation_alignment)

# Translation estimation.
catkin_add_gtest(test_translation_alignment
  test/translation-alignment/translation-alignment-test.cc)
target_link_libraries(test_translation_alignment
  ${PROJECT_NAME}_lib)
phaser_import_test_maps(test_translation_alignment)

# Transformation estimation.
catkin_add_gtest(test_transformation_alignment
  test/transformation-alignment/transformation-alignment-test.cc)
target_link_libraries(test_transformation_alignment
  ${PROJECT_NAME}_lib)
phaser_import_test_maps(test_transformation_alignment)

# Voxgraph transformation estimation.
catkin_add_gtest(test_voxgraph_alignment
  test/transformation-alignment/voxgraph-alignment-test.cc)
target_link_libraries(test_voxgraph_alignment
  ${PROJECT_NAME}_lib)
phaser_import_test_maps(test_voxgraph_alignment)

cs_install()
cs_export()
