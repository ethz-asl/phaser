#include "packlo/backend/registration/mock/sph-registration-mock-cutted.h"
#include "packlo/common/rotation-utils.h"

#include <pcl/filters/passthrough.h>
#include <pcl/common/common.h>
#include <glog/logging.h>

namespace registration {

void SphRegistrationMockCutted::registerPointCloud(
    model::PointCloudPtr cloud_prev, 
    model::PointCloudPtr) {
  common::Point_t min_pt, max_pt;
  common::PointCloud_tPtr raw_cloud = cloud_prev->getRawCloud();

  pcl::getMinMax3D(*raw_cloud, min_pt, max_pt);
  //model::PointCloud_tPtr input_cloud2 (new model::PointCloud_t);
  //pcl::copyPointCloud(*raw_cloud, *input_cloud2);

  float dist = std::sqrt((min_pt.y - max_pt.y) * (min_pt.y - max_pt.y));
  VLOG(1) << "min pt dist: " << dist;
  model::PointCloud point_cloud = cutPointCloud(raw_cloud,
      min_pt.y + 0.2 * dist, min_pt.y + 0.60 * dist, "y");
  model::PointCloud syn_cloud = cutPointCloud(raw_cloud,
      min_pt.y + 0.50 * dist, min_pt.y + dist, "y");


  point_cloud.initialize_kd_tree();
  syn_cloud.initialize_kd_tree();
  std::array<double, 3> zyz;
  correlatePointcloud(point_cloud, syn_cloud, &zyz);
  
  model::PointCloud reg_cloud = common::RotationUtils::RotateAroundZYZCopy(
      syn_cloud, zyz[2], zyz[1], zyz[0]);
}

model::PointCloud SphRegistrationMockCutted::cutPointCloud(
    common::PointCloud_tPtr& cloud, 
    double min, double max, std::string&& dim) {
  VLOG(1) << "pass using " << min << " and " << max;
  common::PointCloud_tPtr mod_cloud (new common::PointCloud_t);
  pcl::PassThrough<common::Point_t> pass;
  pass.setInputCloud (cloud);
  pass.setFilterFieldName (dim);
  pass.setFilterLimits (min,max);
  pass.filter (*mod_cloud);
  return model::PointCloud{mod_cloud};
}

} // namespace registration
