#include <chrono>
#include <cmath>
#include <gtest/gtest.h>
#include <memory>
#include <random>

#include <opencv2/highgui/highgui.hpp>

#include "phaser/common/data/datasource-ply.h"
#include "phaser/common/test/testing-entrypoint.h"
#include "phaser/common/test/testing-predicates.h"
#include "phaser/model/point-cloud.h"
#include "phaser_pre/algorithm/image-projection.h"

namespace preproc {

class ImageProjectionTest : public ::testing::Test {
 public:
  ImageProjectionTest()
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

 protected:
  virtual void SetUp() {
    ds_ = std::make_unique<data::DatasourcePly>();
    CHECK_NOTNULL(ds_);
    ds_->setDatasetFolder("./test_clouds/arche/sigma-level-1/");
  }

  data::DatasourcePlyPtr ds_;
  std::default_random_engine generator_;
  std::uniform_real_distribution<float> distribution_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr target_;
};

TEST_F(ImageProjectionTest, ProjectionSeqTest) {
  CHECK(ds_);
  ImageProjection proj;
  ds_->subscribeToPointClouds([&](const model::PointCloudPtr& cloud) {
    CHECK(cloud);
    ProjectionResult result = proj.projectPointCloudSequential(cloud);
    cv::imshow("range wnd", result.getSignalMat());
    cvWaitKey(0);
  });
  ds_->startStreaming(1);
}

}  // namespace preproc

MAPLAB_UNITTEST_ENTRYPOINT
