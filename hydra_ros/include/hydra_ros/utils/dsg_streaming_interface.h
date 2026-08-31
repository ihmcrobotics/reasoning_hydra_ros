/* -----------------------------------------------------------------------------
 * Copyright 2022 Massachusetts Institute of Technology.
 * All Rights Reserved
 * -------------------------------------------------------------------------- */
#pragma once

#include <hydra/common/dsg_types.h>
#include <hydra_msgs/msg/dsg_update.hpp>
#include <ianvs/node_handle.h>

#include <optional>
#include <string>

namespace hydra {

//! Publishes complete binary-serialized scene-graph snapshots over ROS 2.
class DsgSender {
 public:
  struct Config {
    std::string frame_id;
    std::string timer_name = "publish_dsg";
    bool serialize_dsg_mesh = true;
    double min_dsg_separation_s = 0.0;

    Config withName(const std::string& name) const;
    Config withFrame(const std::string& frame) const;
  } const config;

  DsgSender(const Config& config, ianvs::NodeHandle nh);

  void sendGraph(const DynamicSceneGraph& graph, const rclcpp::Time& stamp) const;

 private:
  rclcpp::Publisher<hydra_msgs::msg::DsgUpdate>::SharedPtr publisher_;
  mutable std::optional<uint64_t> last_publish_time_ns_;
  mutable int64_t sequence_number_ = 0;
};

void declare_config(DsgSender::Config& config);

}  // namespace hydra
