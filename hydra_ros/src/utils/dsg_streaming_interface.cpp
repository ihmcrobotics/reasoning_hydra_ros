/* -----------------------------------------------------------------------------
 * Copyright 2022 Massachusetts Institute of Technology.
 * All Rights Reserved
 * -------------------------------------------------------------------------- */
#include "hydra_ros/utils/dsg_streaming_interface.h"

#include <config_utilities/config.h>
#include <config_utilities/validation.h>
#include <spark_dsg/serialization/graph_binary_serialization.h>

#include <chrono>
#include <memory>

namespace hydra {

void declare_config(DsgSender::Config& config) {
  using namespace config;
  name("DsgSender::Config");
  field(config.frame_id, "frame_id");
  field(config.timer_name, "timer_name");
  field(config.serialize_dsg_mesh, "serialize_dsg_mesh");
  field(config.min_dsg_separation_s, "min_dsg_separation_s");
  checkCondition(!config.frame_id.empty(), "frame_id is empty");
}

DsgSender::Config DsgSender::Config::withName(const std::string& name) const {
  auto result = *this;
  result.timer_name = name;
  return result;
}

DsgSender::Config DsgSender::Config::withFrame(const std::string& frame) const {
  auto result = *this;
  result.frame_id = frame;
  return result;
}

DsgSender::DsgSender(const Config& config, ianvs::NodeHandle nh)
    : config(config::checkValid(config)) {
  const auto qos = rclcpp::QoS(1).reliable().transient_local();
  publisher_ = nh.create_publisher<hydra_msgs::msg::DsgUpdate>("dsg", qos);
}

void DsgSender::sendGraph(const DynamicSceneGraph& graph,
                          const rclcpp::Time& stamp) const {
  const auto timestamp_ns = static_cast<uint64_t>(stamp.nanoseconds());
  if (last_publish_time_ns_) {
    const auto elapsed = std::chrono::duration<double>(
        std::chrono::nanoseconds(timestamp_ns - *last_publish_time_ns_));
    if (elapsed.count() < config.min_dsg_separation_s) {
      return;
    }
  }

  last_publish_time_ns_ = timestamp_ns;
  auto message = std::make_unique<hydra_msgs::msg::DsgUpdate>();
  message->header.stamp = stamp;
  message->header.frame_id = config.frame_id;
  spark_dsg::io::binary::writeGraph(
      graph, message->layer_contents, config.serialize_dsg_mesh);
  message->full_update = true;
  message->sequence_number = sequence_number_++;
  publisher_->publish(std::move(message));
}

}  // namespace hydra
