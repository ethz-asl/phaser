#pragma once

#include <packlo/model/packet.h>

#include <ouster_ros/PacketMsg.h>

namespace common {
class PacketConverter {
  public:
    PacketConverter(const PacketConverter &conv) = delete;

    static model::Packet convert(const ouster_ros::PacketMsgConstPtr& packet);

  private:
    PacketConverter();
};
} // namespace common
