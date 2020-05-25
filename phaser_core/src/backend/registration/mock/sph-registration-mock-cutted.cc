#include "phaser/backend/registration/mock/sph-registration-mock-cutted.h"
#include "phaser/common/rotation-utils.h"

#include <pcl/filters/passthrough.h>
#include <pcl/common/common.h>
#include <glog/logging.h>

namespace phaser_core {

model::RegistrationResult SphRegistrationMockCutted::registerPointCloud(
    model::PointCloudPtr cloud_prev, model::PointCloudPtr cloud_cur) {
  CHECK(cloud_prev);
  CHECK(cloud_cur);
  VLOG(1) << "=== Registering point cloud (mock cutted) ======================";
  VLOG(1) << "Cloud1: " << cloud_prev->getPlyReadDirectory();
  VLOG(1) << "Cloud2: " << cloud_cur->getPlyReadDirectory();
  common::Point_t min_pt, max_pt;
  common::PointCloud_tPtr raw_cloud = cloud_prev->getRawCloud();
  common::PointCloud_tPtr cur_raw_cloud = cloud_cur->getRawCloud();

  pcl::getMinMax3D(*raw_cloud, min_pt, max_pt);
  float dist = std::sqrt((min_pt.y - max_pt.y) * (min_pt.y - max_pt.y));
  VLOG(3) << "min pt dist: " << dist;
  model::PointCloudPtr point_cloud =
      cutPointCloud(raw_cloud, min_pt.y + 1, min_pt.y + 0.7 * dist, "y");
  model::PointCloudPtr syn_cloud =
      cutPointCloud(raw_cloud, min_pt.y + 0.55 * dist, min_pt.y + dist, "y");

  point_cloud->writeToFile("./");
  common::RotationUtils::RotateAroundXYZ(syn_cloud, 0.2, 0.55, 1.23);
  syn_cloud->writeToFile("./");


  point_cloud->initialize_kd_tree();
  syn_cloud->initialize_kd_tree();
  model::RegistrationResult result = estimateRotation(point_cloud, syn_cloud);

  model::PointCloudPtr reg_cloud = result.getRegisteredCloud();

  reg_cloud->writeToFile("./");

  return result;
}

model::PointCloudPtr SphRegistrationMockCutted::cutPointCloud(
    common::PointCloud_tPtr& cloud, double min, double max, std::string&& dim) {
  VLOG(3) << "pass using " << min << " and " << max;
  common::PointCloud_tPtr mod_cloud (new common::PointCloud_t);
  pcl::PassThrough<common::Point_t> pass;
  pass.setInputCloud (cloud);
  pass.setFilterFieldName (dim);
  pass.setFilterLimits (min,max);
  pass.filter (*mod_cloud);
  return std::make_shared<model::PointCloud>(mod_cloud);
}

}  // namespace phaser_core
