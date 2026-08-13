// Copyright (c) 2026, IHMC Robotics Lab.
// All rights reserved.

#include "hydra_ros/backend/ros_backend_publisher.h"

#include <hydra/common/global_info.h>
#include <vision_msgs/msg/bounding_box3_d.hpp>

#include <Eigen/Geometry>
#include <unordered_map>

namespace hydra {

Ros2BackendPublisher::Ros2BackendPublisher(
    ianvs::NodeHandle nh,
    const std::string& active_edges_topic)
    : active_edges_pub_(
          nh.create_publisher<hydra_msgs::msg::ActiveObjectRelationships>(
              active_edges_topic, rclcpp::QoS(10))) {}

void Ros2BackendPublisher::call(
    uint64_t timestamp_ns,
    const DynamicSceneGraph& graph,
    const kimera_pgmo::DeformationGraph& /*dgraph*/) {
  publishActiveObjectEdges(graph, timestamp_ns);
}

void Ros2BackendPublisher::publishActiveObjectEdges(
    const DynamicSceneGraph& graph,
    uint64_t timestamp_ns) const {
  if (active_edges_pub_->get_subscription_count() == 0) {
    return;
  }

  hydra_msgs::msg::ActiveObjectRelationships message;
  message.header.stamp = rclcpp::Time(timestamp_ns);
  message.header.frame_id = GlobalInfo::instance().getFrames().odom;

  const auto& objects = graph.getLayer(DsgLayers::OBJECTS);
  std::unordered_map<NodeId, uint32_t> object_indices;

  auto add_object = [&](NodeId id, const ObjectNodeAttributes& attributes) {
    const auto existing = object_indices.find(id);
    if (existing != object_indices.end()) {
      return existing->second;
    }

    vision_msgs::msg::BoundingBox3D box;
    box.center.position.x = attributes.bounding_box.world_P_center.x();
    box.center.position.y = attributes.bounding_box.world_P_center.y();
    box.center.position.z = attributes.bounding_box.world_P_center.z();

    const Eigen::Quaternionf orientation(
        attributes.bounding_box.world_R_center.matrix());
    box.center.orientation.x = orientation.x();
    box.center.orientation.y = orientation.y();
    box.center.orientation.z = orientation.z();
    box.center.orientation.w = orientation.w();
    box.size.x = attributes.bounding_box.dimensions.x();
    box.size.y = attributes.bounding_box.dimensions.y();
    box.size.z = attributes.bounding_box.dimensions.z();

    const auto index = static_cast<uint32_t>(message.object_boxes.size());
    object_indices.emplace(id, index);
    message.object_boxes.push_back(std::move(box));
    return index;
  };

  for (const auto& [key, edge] : objects.edges()) {
    (void)key;
    if (!graph.hasNode(edge.source) || !graph.hasNode(edge.target)) {
      continue;
    }

    const auto& source = graph.getNode(edge.source);
    const auto& target = graph.getNode(edge.target);
    const auto& source_attributes = source.attributes<ObjectNodeAttributes>();
    const auto& target_attributes = target.attributes<ObjectNodeAttributes>();
    if (!source_attributes.bounding_box.isValid() ||
        !target_attributes.bounding_box.isValid()) {
      continue;
    }

    message.object_ids.push_back(add_object(edge.source, source_attributes));
    message.object_ids.push_back(add_object(edge.target, target_attributes));
  }

  if (!message.object_boxes.empty()) {
    active_edges_pub_->publish(message);
  }
}

}  // namespace hydra
