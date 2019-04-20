#pragma once

#include <ouster_ros/PacketMsg.h>

namespace controller {

class PacketDistributor {
  public: 

  void packetCloudCallback(const ouster_ros::PacketMsgConstPtr& packet);

  private:
    void distribute(const ouster_ros::PacketMsgConstPtr& packet);

};

}
