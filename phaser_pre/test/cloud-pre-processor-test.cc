#include "phaser_pre/cloud-pre-processor.h"

#include <chrono>
#include <cmath>
#include <gtest/gtest.h>
#include <memory>
#include <random>

#include "phaser/common/test/testing-entrypoint.h"
#include "phaser/common/test/testing-predicates.h"
#include "phaser/model/point-cloud.h"

namespace preproc {

class CloudPreProcessorTest : public ::testing::Test {
 public:
  CloudPreProcessorTest()
      : generator_(std::chrono::system_clock::now().time_since_epoch().count()),
        distribution_(0, M_PI) {}

  model::PointCloudPtr getRandomCloud(const uint32_t width = 100) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(
        new pcl::PointCloud<pcl::PointXYZI>);
    cloud->width = width;
    cloud->height = 1;
    cloud->is_dense = false;

    const uint32_t n_points = cloud->width * cloud->height;
    cloud->points.resize(n_points);
    for (uint32_t i = 0u; i < n_points; ++i) {
      cloud->points[i].x = distribution_(generator_);
      cloud->points[i].y = distribution_(generator_);
      cloud->points[i].z = distribution_(generator_);
      cloud->points[i].intensity = distribution_(generator_);
    }
    return std::make_shared<model::PointCloud>(cloud);
  }

 private:
  std::default_random_engine generator_;
  std::uniform_real_distribution<float> distribution_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr target_;
};

TEST_F(CloudPreProcessorTest, CreationSanityCheck) {
  CloudPreProcessorSettings settings;
  settings.enable_voxel_grid_downsampling = true;
  settings.enable_pass_through_gnd_filtering = true;

  auto pre_processor = std::make_unique<CloudPreProcessor>(settings);
  EXPECT_NE(nullptr, pre_processor);
}

TEST_F(CloudPreProcessorTest, VoxelGridCmdTest) {
  CloudPreProcessorSettings settings;
  settings.enable_voxel_grid_downsampling = true;

  CloudPreProcessor pre_processor(settings);

  const uint32_t n_points = 500;
  model::PointCloudPtr cloud = getRandomCloud(n_points);
  pre_processor.process(cloud);

  EXPECT_LT(cloud->size(), n_points);
}

TEST_F(CloudPreProcessorTest, PassThroughGndFilterTest) {
  CloudPreProcessorSettings settings;
  settings.enable_pass_through_gnd_filtering = true;

  CloudPreProcessor pre_processor(settings);

  const uint32_t n_points = 500;
  model::PointCloudPtr cloud = getRandomCloud(n_points);
  pre_processor.process(cloud);

  EXPECT_LT(cloud->size(), n_points);
}

}  // namespace preproc

MAPLAB_UNITTEST_ENTRYPOINT
