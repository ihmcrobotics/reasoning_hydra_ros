/* -----------------------------------------------------------------------------
 * Copyright 2022 Massachusetts Institute of Technology.
 * All Rights Reserved
 * -------------------------------------------------------------------------- */
#pragma once

#include <hydra/frontend/frontend_module.h>
#include <ianvs/node_handle.h>
#include <kimera_pgmo_msgs/msg/kimera_pgmo_mesh_delta.hpp>
#include <pose_graph_tools_ros/conversions.h>

#include "hydra_ros/utils/dsg_streaming_interface.h"

namespace hydra {

class RosFrontendPublisher : public FrontendModule::Sink {
 public:
  explicit RosFrontendPublisher(ianvs::NodeHandle nh);

  void call(uint64_t timestamp_ns,
            const DynamicSceneGraph& graph,
            const BackendInput& backend_input) override;

  std::string printInfo() const override { return "RosFrontendPublisher"; }

 private:
  std::unique_ptr<DsgSender> dsg_sender_;
  pose_graph_tools::PoseGraphPublisher mesh_graph_pub_;
  rclcpp::Publisher<kimera_pgmo_msgs::msg::KimeraPgmoMeshDelta>::SharedPtr
      mesh_update_pub_;
};

}  // namespace hydra
