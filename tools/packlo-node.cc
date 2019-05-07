#include <packlo/packlo-node.h>
#include <packlo/controller/distributor.h>

#include <ros/ros.h>
#include <glog/logging.h>

namespace packlo {
  PackloNode::PackloNode(const ros::NodeHandle& nh) 
    : node_handle_(nh) {
    LOG(INFO) << "running packlo";
    dist_ = std::make_unique<controller::Distributor>(ds_);
  }
} // namespace packlo

int main(int argc, char** argv) {
  ros::init(argc, argv, "packlo");
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();
  
  ros::NodeHandle nh;
  packlo::PackloNode node(nh);

  ros::spin();
  return 0;
}
