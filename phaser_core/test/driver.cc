#include <gflags/gflags.h>
#include <glog/logging.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <ros/ros.h>

#include "phaser/backend/registration/sph-opt-registration.h"
#include "phaser/controller/cloud-controller.h"

DEFINE_string(target_cloud, "", "Defines the path to the target cloud.");
DEFINE_string(source_cloud, "", "Defines the path to the source cloud.");
DEFINE_string(reg_cloud, "", "Defines the path to the registered cloud.");

static model::PointCloudPtr readPointCloud(
    const std::string& cloud) {
  CHECK(!cloud.empty());
  LOG(INFO) << "Reading point cloud from " << cloud;
  pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud(
      new pcl::PointCloud<pcl::PointXYZI>);
  if (pcl::io::loadPLYFile<pcl::PointXYZI>(cloud, *input_cloud) == -1) {
    LOG(FATAL) << "Unable to load point cloud.";
    return nullptr;
  }
  LOG(INFO) << "Point cloud loaded with size " << input_cloud->size();
  return std::make_shared<model::PointCloud>(input_cloud);
}

static void writePointCloud(
    const std::string& reg_file, model::PointCloudPtr cloud) {
  CHECK(!reg_file.empty());
  CHECK_NOTNULL(cloud);
  pcl::io::savePLYFileASCII(reg_file, *cloud->getRawCloud());
  LOG(INFO) << "Wrote registered cloud to " << reg_file;
}

static void registerCloud(
    const std::string& target, const std::string& source,
    const std::string& reg_cloud) {
  model::PointCloudPtr target_cloud = readPointCloud(target);
  model::PointCloudPtr source_cloud = readPointCloud(source);
  CHECK_NOTNULL(target_cloud);
  CHECK_NOTNULL(source_cloud);
  CHECK(!reg_cloud.empty());

  auto ctrl = std::make_unique<controller::CloudController>("sph");
  model::RegistrationResult result =
    ctrl->registerPointCloud(target_cloud, source_cloud);

  LOG(INFO) << "Registration result: " << result.getStateAsVec().transpose();
  writePointCloud(reg_cloud, result.getRegisteredCloud());
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "phaser_core_driver");
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();

  VLOG(1) << "=== PHASER CORE DRIVER =====================";
  registerCloud(
      FLAGS_target_cloud, FLAGS_source_cloud, FLAGS_reg_cloud);

  return 0;
}
