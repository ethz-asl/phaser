#include <packlo/controller/packet-distributor.h>
#include <packlo/common/packet-converter.h>

namespace controller {
  
void PacketDistributor::packetCloudCallback(
    const ouster_ros::PacketMsgConstPtr& packet) {
}

void PacketDistributor::distribute(
    const ouster_ros::PacketMsgConstPtr& ouster_packet) {
  model::Packet packet = common::PacketConverter::convert(ouster_packet);
}



}
