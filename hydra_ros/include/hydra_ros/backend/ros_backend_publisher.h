// Copyright (c) 2026, IHMC Robotics Lab.
// All rights reserved.

#pragma once

#include <hydra/backend/backend_module.h>
#include <hydra_msgs/msg/active_object_relationships.hpp>
#include <ianvs/node_handle.h>

#include <string>

namespace hydra {

class Ros2BackendPublisher : public BackendModule::Sink {
 public:
  Ros2BackendPublisher(ianvs::NodeHandle nh, const std::string& active_edges_topic);

  void call(uint64_t timestamp_ns,
            const DynamicSceneGraph& graph,
            const kimera_pgmo::DeformationGraph& dgraph) override;

 private:
  void publishActiveObjectEdges(const DynamicSceneGraph& graph,
                                uint64_t timestamp_ns) const;

  ianvs::NodeHandle::Publisher<hydra_msgs::msg::ActiveObjectRelationships>
      active_edges_pub_;
};

}  // namespace hydra
