// Copyright (c) 2026, IHMC Robotics Lab.
// All rights reserved.

#pragma once

#include <hydra/backend/backend_module.h>
#include <hydra_msgs/msg/active_object_relationships.hpp>
#include <ianvs/node_handle.h>
#include <pose_graph_tools_ros/conversions.h>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/msg/marker.hpp>

#include <string>

#include "hydra_ros/utils/dsg_streaming_interface.h"

namespace hydra {

class Ros2BackendPublisher : public BackendModule::Sink {
 public:
  Ros2BackendPublisher(ianvs::NodeHandle nh, const std::string& active_edges_topic);

  void call(uint64_t timestamp_ns,
            const DynamicSceneGraph& graph,
            const kimera_pgmo::DeformationGraph& dgraph) override;

 private:
  void publishPoseGraph(const DynamicSceneGraph& graph,
                        const kimera_pgmo::DeformationGraph& dgraph,
                        uint64_t timestamp_ns) const;
  void publishDeformationGraph(
      const kimera_pgmo::DeformationGraph& dgraph,
      uint64_t timestamp_ns) const;
  void publishBackendTf(const DynamicSceneGraph& graph,
                        const kimera_pgmo::DeformationGraph& dgraph) const;
  void publishActiveObjectEdges(const DynamicSceneGraph& graph,
                                uint64_t timestamp_ns) const;

  ianvs::NodeHandle nh_;
  std::unique_ptr<DsgSender> dsg_sender_;
  pose_graph_tools::PoseGraphPublisher pose_graph_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr
      mesh_mesh_edges_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr
      pose_mesh_edges_pub_;
  mutable tf2_ros::TransformBroadcaster tf_broadcaster_;
  ianvs::NodeHandle::Publisher<hydra_msgs::msg::ActiveObjectRelationships>
      active_edges_pub_;
};

}  // namespace hydra
