/* -----------------------------------------------------------------------------
 * Copyright 2022 Massachusetts Institute of Technology.
 * All Rights Reserved
 * -------------------------------------------------------------------------- */
#include "hydra_ros/frontend/ros_frontend_publisher.h"

#include <hydra/common/global_info.h>

#include <algorithm>
#include <limits>

namespace hydra {

namespace {

kimera_pgmo_msgs::msg::KimeraPgmoMeshDelta toMeshDeltaMessage(
    const kimera_pgmo::MeshDelta& delta,
    uint64_t timestamp_ns,
    const std::string& frame_id) {
  kimera_pgmo_msgs::msg::KimeraPgmoMeshDelta message;
  message.header.stamp = rclcpp::Time(timestamp_ns);
  message.header.frame_id = frame_id;
  message.vertex_start = delta.vertex_start;
  message.face_start = delta.face_start;

  const auto& vertices = *delta.vertex_updates;
  message.vertex_updates.resize(vertices.size());
  message.vertex_updates_colors.resize(vertices.size());
  constexpr float color_scale =
      1.0f / std::numeric_limits<uint8_t>::max();
  for (size_t i = 0; i < vertices.size(); ++i) {
    auto& point = message.vertex_updates[i];
    point.x = vertices[i].x;
    point.y = vertices[i].y;
    point.z = vertices[i].z;
    auto& color = message.vertex_updates_colors[i];
    color.r = color_scale * vertices[i].r;
    color.g = color_scale * vertices[i].g;
    color.b = color_scale * vertices[i].b;
    color.a = color_scale * vertices[i].a;
  }

  message.stamp_updates = delta.stamp_updates;
  if (delta.hasSemantics()) {
    message.semantic_updates = delta.semantic_updates;
  }
  message.deleted_indices.assign(delta.deleted_indices.begin(),
                                 delta.deleted_indices.end());

  auto convert_faces = [](const auto& source, auto& destination) {
    destination.resize(source.size());
    for (size_t i = 0; i < source.size(); ++i) {
      destination[i].vertex_indices = {
          static_cast<uint32_t>(source[i].v1),
          static_cast<uint32_t>(source[i].v2),
          static_cast<uint32_t>(source[i].v3)};
    }
  };
  convert_faces(delta.face_updates, message.face_updates);
  convert_faces(delta.face_archive_updates, message.face_archive_updates);

  for (const auto& [previous, current] : delta.prev_to_curr) {
    message.prev_indices.push_back(previous);
    message.curr_indices.push_back(current);
  }
  return message;
}

}  // namespace

RosFrontendPublisher::RosFrontendPublisher(ianvs::NodeHandle nh) {
  const auto frame = GlobalInfo::instance().getFrames().odom;
  dsg_sender_ = std::make_unique<DsgSender>(
      DsgSender::Config{frame, "frontend_dsg", true, 0.0}, nh);

  const auto qos = rclcpp::QoS(100).reliable().transient_local();
  mesh_graph_pub_ =
      nh.create_publisher<pose_graph_tools::PoseGraphTypeAdapter>(
          "mesh_graph_incremental", qos);
  mesh_update_pub_ =
      nh.create_publisher<kimera_pgmo_msgs::msg::KimeraPgmoMeshDelta>(
          "full_mesh_update", qos);
}

void RosFrontendPublisher::call(uint64_t timestamp_ns,
                                const DynamicSceneGraph& graph,
                                const BackendInput& backend_input) {
  dsg_sender_->sendGraph(graph, rclcpp::Time(timestamp_ns));

  if (backend_input.deformation_graph) {
    auto pose_graph = *backend_input.deformation_graph;
    pose_graph.stamp_ns = timestamp_ns;
    pose_graph.frame_id = GlobalInfo::instance().getFrames().odom;
    mesh_graph_pub_->publish(pose_graph);
  }

  if (backend_input.mesh_update) {
    auto message = toMeshDeltaMessage(*backend_input.mesh_update,
                                      timestamp_ns,
                                      GlobalInfo::instance().getFrames().odom);
    mesh_update_pub_->publish(message);
  }
}

}  // namespace hydra
