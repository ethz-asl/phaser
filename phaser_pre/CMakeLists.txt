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
  src/common/sse-mathfun.cc
  src/common/sse-mathfun-extension.cc

  src/model/projection-result.cc
  src/model/cluster-result.cc
  src/model/segmentation-result.cc
  src/model/ground-removal-result.cc
  src/model/feature-extraction-result.cc
  src/model/smoothness-result.cc
  src/model/occlusion-result.cc
  src/model/gradient-result.cc

  src/commands/voxel-grid-cmd.cc
  src/commands/pass-through-gnd-filter-cmd.cc
  src/commands/segment-cloud-cmd.cc

  src/algorithm/image-projection.cc
  src/algorithm/angle-based-ground-removal.cc
  src/algorithm/cluster-points.cc
  src/algorithm/cloud-segmentation.cc
  src/algorithm/extract-loam-features.cc
  src/algorithm/calc-smoothness.cc
  src/algorithm/mark-occluded.cc
  src/algorithm/image-gradient.cc
)

#############
## TESTING ##
#############

catkin_add_gtest(test_cloud_pre_processor test/cloud-pre-processor-test.cc)
target_link_libraries(test_cloud_pre_processor ${PROJECT_NAME}_lib)

catkin_add_gtest(test_image_projection test/image-projection-test.cc)
target_link_libraries(test_image_projection ${PROJECT_NAME}_lib)
phaser_import_test_maps(test_image_projection)

catkin_add_gtest(test_angle_based_ground_removal test/angle-based-ground-removal-test.cc)
target_link_libraries(test_angle_based_ground_removal ${PROJECT_NAME}_lib)
phaser_import_test_maps(test_angle_based_ground_removal)

catkin_add_gtest(test_cluster_points test/cluster-points-test.cc)
target_link_libraries(test_cluster_points ${PROJECT_NAME}_lib)
phaser_import_test_maps(test_cluster_points)

catkin_add_gtest(test_cloud_segmentation test/cloud-segmentation-test.cc)
target_link_libraries(test_cloud_segmentation ${PROJECT_NAME}_lib)
phaser_import_test_maps(test_cloud_segmentation)

catkin_add_gtest(test_mark_occluded test/mark-occluded-test.cc)
target_link_libraries(test_mark_occluded ${PROJECT_NAME}_lib)
phaser_import_test_maps(test_mark_occluded)

catkin_add_gtest(test_loam_features test/loam-feature-extraction-test.cc)
target_link_libraries(test_loam_features ${PROJECT_NAME}_lib)
phaser_import_test_maps(test_loam_features)

catkin_add_gtest(test_image_gradient test/image-gradient-test.cc)
target_link_libraries(test_image_gradient ${PROJECT_NAME}_lib)
phaser_import_test_maps(test_image_gradient)

cs_install()
cs_export()
