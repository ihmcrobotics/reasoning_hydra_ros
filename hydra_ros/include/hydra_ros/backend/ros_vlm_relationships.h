// Copyright (c) 2026, IHMC Robotics Lab.
// All rights reserved.

#pragma once

#include <hydra/backend/backend_module.h>
#include <hydra_msgs/msg/labeled_relationships.hpp>
#include <hydra_msgs/msg/visual_relationships_encodings.hpp>
#include <hydra_msgs/srv/vlm_relationship.hpp>
#include <ianvs/node_handle.h>

#include <string>

namespace hydra {

class Ros2VLMRelationships : public BackendModule::Sink {
 public:
  struct Config {
    std::string default_prompt = "What is the relationship between";
    size_t queue_size = 10;
    std::string service = "/hydra_ros_node/backend/vlm_relationships";
    std::string encodings_topic =
        "/hydra_ros_node/backend/vlm_relationships/visual_relationships_encodings";
    std::string labels_topic = "/semantic_inference/labeled_relationships";
  };

  Ros2VLMRelationships(
      ianvs::NodeHandle nh,
      Config config,
      const InputQueue<BackendVLMLabelsInput::Ptr>::Ptr& labels_queue);

  void call(uint64_t timestamp_ns,
            const DynamicSceneGraph& graph,
            const kimera_pgmo::DeformationGraph& dgraph) override;

 private:
  struct Request {
    std::string prompt;
    int room_id;
  };

  void handleRequest(
      const std::shared_ptr<hydra_msgs::srv::VLMRelationship::Request> request,
      std::shared_ptr<hydra_msgs::srv::VLMRelationship::Response> response);
  void handleLabels(
      hydra_msgs::msg::LabeledRelationships::ConstSharedPtr relationships);
  void resetLabels(const DynamicSceneGraph& graph, uint64_t timestamp_ns);

  const Config config_;
  InputQueue<BackendVLMLabelsInput::Ptr>::Ptr labels_queue_;
  InputQueue<Request> request_queue_;
  ianvs::NodeHandle::Service<hydra_msgs::srv::VLMRelationship> service_;
  ianvs::NodeHandle::Subscription<hydra_msgs::msg::LabeledRelationships>
      labels_sub_;
  ianvs::NodeHandle::Publisher<hydra_msgs::msg::VisualRelationshipsEncodings>
      encodings_pub_;
};

}  // namespace hydra
