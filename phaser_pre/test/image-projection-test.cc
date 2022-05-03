#include "phaser_pre/algorithm/image-projection.h"

#include <Eigen/Dense>
#include <chrono>
#include <cmath>
#include <eigen-checks/gtest.h>
#include <gtest/gtest.h>
#include <memory>
#include <opencv2/core/eigen.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <random>

#include "phaser/common/data/datasource-ply.h"
#include "phaser/common/test/testing-entrypoint.h"
#include "phaser/common/test/testing-predicates.h"
#include "phaser/model/point-cloud.h"

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
    ds_->setDatasetFolder("./test_clouds/os0/");
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
    const ProjectionResult result = proj.projectPointCloudSequential(cloud);

    Eigen::MatrixXf range, signals;
    cv::cv2eigen(result.getRangeMat(), range);
    cv::cv2eigen(result.getSignalMat(), signals);
    EXPECT_TRUE((range.array() > 0).any());
    EXPECT_TRUE((signals.array() > 0).any());
  });
  ds_->startStreaming(1);
}

TEST_F(ImageProjectionTest, ProjectionVecTest) {
  CHECK(ds_);
  ImageProjection proj;
  ds_->subscribeToPointClouds([&](const model::PointCloudPtr& cloud) {
    CHECK(cloud);
    const ProjectionResult result = proj.projectPointCloud(cloud);

    Eigen::MatrixXf range, signals;
    cv::cv2eigen(result.getRangeMat(), range);
    cv::cv2eigen(result.getSignalMat(), signals);
    EXPECT_TRUE((range.array() > 0).any());
    EXPECT_TRUE((signals.array() > 0).any());
  });
  ds_->startStreaming(1);
}

TEST_F(ImageProjectionTest, FullCloudTest) {
  CHECK(ds_);
  ImageProjection proj;
  ds_->subscribeToPointClouds([&](const model::PointCloudPtr& cloud) {
    CHECK(cloud);
    const ProjectionResult result = proj.projectPointCloud(cloud);

    common::PointCloud_tPtr full_cloud = result.getFullCloud();
    common::PointCloud_tPtr full_info_cloud = result.getFullInfoCloud();
    EXPECT_NE(full_cloud, nullptr);
    EXPECT_NE(full_info_cloud, nullptr);
    EXPECT_GT(full_cloud->size(), 0);
    EXPECT_GT(full_info_cloud->size(), 0);
  });
  ds_->startStreaming(1);
}

TEST_F(ImageProjectionTest, ProjectionSeqVecEqualResultTest) {
  CHECK(ds_);
  ImageProjection proj;
  const float precision = 0.1f;
  ds_->subscribeToPointClouds([&](const model::PointCloudPtr& cloud) {
    CHECK(cloud);
    const ProjectionResult resultVec = proj.projectPointCloud(cloud);
    const ProjectionResult resultSeq = proj.projectPointCloudSequential(cloud);

    Eigen::MatrixXf rangeVec, signalsVec, rangeSeq, signalsSeq;
    cv::cv2eigen(resultVec.getRangeMat(), rangeVec);
    cv::cv2eigen(resultVec.getSignalMat(), signalsVec);
    cv::cv2eigen(resultSeq.getRangeMat(), rangeSeq);
    cv::cv2eigen(resultSeq.getSignalMat(), signalsSeq);

    // cv::imshow("signal wnd seq", resultSeq.getSignalMat());
    // cv::imshow("signal wnd vec", resultVec.getSignalMat());
    // cv::waitKey(0);
    EXPECT_TRUE((rangeVec.array() > 0).any());
    EXPECT_TRUE((signalsVec.array() > 0).any());
    EXPECT_TRUE((rangeSeq.array() > 0).any());
    EXPECT_TRUE((signalsSeq.array() > 0).any());
    EXPECT_NEAR(rangeVec.nonZeros(), rangeSeq.nonZeros(), 5);
    EXPECT_NEAR(signalsVec.nonZeros(), signalsSeq.nonZeros(), 5);
  });
  ds_->startStreaming(1);
}

}  // namespace preproc

MAPLAB_UNITTEST_ENTRYPOINT
