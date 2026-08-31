// Copyright (c) 2026, IHMC Robotics Lab.
// All rights reserved.

#include "hydra_ros/backend/ros_backend_publisher.h"

#include <hydra/common/global_info.h>
#include <kimera_pgmo_ros/visualization_functions.h>
#include <pose_graph_tools/pose_graph.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <vision_msgs/msg/bounding_box3_d.hpp>

#include <Eigen/Geometry>
#include <map>
#include <unordered_map>

namespace hydra {

Ros2BackendPublisher::Ros2BackendPublisher(
    ianvs::NodeHandle nh,
    const std::string& active_edges_topic)
    : nh_(ianvs::NodeHandle::this_node("/hydra_ros_node/backend")),
      dsg_sender_(std::make_unique<DsgSender>(
          DsgSender::Config{GlobalInfo::instance().getFrames().map,
                            "backend_dsg",
                            true,
                            0.0},
          nh_)),
      pose_graph_pub_(nh_.create_publisher<
                      pose_graph_tools::PoseGraphTypeAdapter>(
          "pose_graph", rclcpp::QoS(10).reliable().transient_local())),
      mesh_mesh_edges_pub_(
          nh_.create_publisher<visualization_msgs::msg::Marker>(
              "deformation_graph_mesh_mesh", 10)),
      pose_mesh_edges_pub_(
          nh_.create_publisher<visualization_msgs::msg::Marker>(
              "deformation_graph_pose_mesh", 10)),
      tf_broadcaster_(nh_.node()),
      active_edges_pub_(
          nh.create_publisher<hydra_msgs::msg::ActiveObjectRelationships>(
              active_edges_topic, rclcpp::QoS(10))) {}

void Ros2BackendPublisher::call(
    uint64_t timestamp_ns,
    const DynamicSceneGraph& graph,
    const kimera_pgmo::DeformationGraph& dgraph) {
  dsg_sender_->sendGraph(graph, rclcpp::Time(timestamp_ns));
  publishPoseGraph(graph, dgraph, timestamp_ns);
  publishDeformationGraph(dgraph, timestamp_ns);
  publishBackendTf(graph, dgraph);
  publishActiveObjectEdges(graph, timestamp_ns);
}

void Ros2BackendPublisher::publishPoseGraph(
    const DynamicSceneGraph& graph,
    const kimera_pgmo::DeformationGraph& dgraph,
    uint64_t timestamp_ns) const {
  const auto& prefix = GlobalInfo::instance().getRobotPrefix();
  if (!graph.hasLayer(DsgLayers::AGENTS, prefix.key)) {
    return;
  }
  const auto& agents = graph.getLayer(DsgLayers::AGENTS, prefix.key);
  if (agents.numNodes() == 0) {
    return;
  }

  std::map<size_t, std::vector<size_t>> timestamps;
  auto& robot_timestamps = timestamps[prefix.id];
  for (const auto& node : agents.nodes()) {
    robot_timestamps.push_back(node->timestamp.value_or(
        std::chrono::nanoseconds(0)).count());
  }

  auto pose_graph = dgraph.getPoseGraph(timestamps);
  if (!pose_graph) {
    return;
  }
  pose_graph->stamp_ns = timestamp_ns;
  pose_graph->frame_id = GlobalInfo::instance().getFrames().map;
  pose_graph_pub_->publish(*pose_graph);
}

void Ros2BackendPublisher::publishDeformationGraph(
    const kimera_pgmo::DeformationGraph& dgraph,
    uint64_t timestamp_ns) const {
  if (!mesh_mesh_edges_pub_->get_subscription_count() &&
      !pose_mesh_edges_pub_->get_subscription_count()) {
    return;
  }

  visualization_msgs::msg::Marker mesh_mesh;
  visualization_msgs::msg::Marker pose_mesh;
  kimera_pgmo::fillDeformationGraphMarkers(
      dgraph,
      rclcpp::Time(timestamp_ns),
      mesh_mesh,
      pose_mesh,
      GlobalInfo::instance().getFrames().map);
  if (!mesh_mesh.points.empty()) {
    mesh_mesh_edges_pub_->publish(mesh_mesh);
  }
  if (!pose_mesh.points.empty()) {
    pose_mesh_edges_pub_->publish(pose_mesh);
  }
}

void Ros2BackendPublisher::publishBackendTf(
    const DynamicSceneGraph& graph,
    const kimera_pgmo::DeformationGraph& dgraph) const {
  const auto& frames = GlobalInfo::instance().getFrames();
  if (frames.map == frames.odom) {
    return;
  }

  const auto& prefix = GlobalInfo::instance().getRobotPrefix();
  if (!graph.hasLayer(DsgLayers::AGENTS, prefix.key)) {
    return;
  }
  const auto& agents = graph.getLayer(DsgLayers::AGENTS, prefix.key);
  if (agents.numNodes() == 0) {
    return;
  }

  const auto pose_index = agents.numNodes() - 1;
  const NodeSymbol key(prefix.key, pose_index);
  if (!graph.hasNode(key)) {
    return;
  }

  const auto& attributes =
      graph.getNode(key).attributes<AgentNodeAttributes>();
  const auto odom_T_body = dgraph.getInitialPose(prefix.key, pose_index);
  const gtsam::Pose3 map_T_body(gtsam::Rot3(attributes.world_R_body),
                               attributes.position);
  const auto map_T_odom = map_T_body * odom_T_body.inverse();

  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = nh_.now();
  transform.header.frame_id = frames.map;
  transform.child_frame_id = frames.odom;
  tf2::convert(map_T_odom.rotation().toQuaternion(),
               transform.transform.rotation);
  tf2::toMsg(map_T_odom.translation(), transform.transform.translation);
  tf_broadcaster_.sendTransform(transform);
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
