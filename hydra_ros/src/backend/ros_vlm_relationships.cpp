// Copyright (c) 2026, IHMC Robotics Lab.
// All rights reserved.

#include "hydra_ros/backend/ros_vlm_relationships.h"

#include <geometry_msgs/msg/pose.hpp>
#include <semantic_inference_msgs/msg/feature_vector.hpp>
#include <vision_msgs/msg/bounding_box3_d.hpp>

#include <Eigen/Geometry>
#include <cctype>

namespace hydra {
namespace {

int extractNumber(const std::string& input) {
  std::string digits;
  for (const char value : input) {
    if (std::isdigit(static_cast<unsigned char>(value))) {
      digits.push_back(value);
    } else if (!digits.empty()) {
      break;
    }
  }

  if (digits.empty()) {
    throw std::runtime_error("No number found in room name: " + input);
  }
  return std::stoi(digits);
}

vision_msgs::msg::BoundingBox3D makeBoundingBox(
    const SemanticNodeAttributes& attributes) {
  vision_msgs::msg::BoundingBox3D box;
  box.center.position.x = attributes.bounding_box.world_P_center.x();
  box.center.position.y = attributes.bounding_box.world_P_center.y();
  box.center.position.z = attributes.bounding_box.world_P_center.z();
  box.center.orientation.w = 1.0;
  if (attributes.bounding_box.hasRotation()) {
    const Eigen::Quaternionf orientation(
        attributes.bounding_box.world_R_center);
    box.center.orientation.x = orientation.x();
    box.center.orientation.y = orientation.y();
    box.center.orientation.z = orientation.z();
    box.center.orientation.w = orientation.w();
  }
  box.size.x = attributes.bounding_box.dimensions.x();
  box.size.y = attributes.bounding_box.dimensions.y();
  box.size.z = attributes.bounding_box.dimensions.z();
  return box;
}

geometry_msgs::msg::Pose makePose(const SemanticNodeAttributes& attributes) {
  geometry_msgs::msg::Pose pose;
  pose.position.x = attributes.position.x();
  pose.position.y = attributes.position.y();
  pose.position.z = attributes.position.z();
  pose.orientation.w = 1.0;
  return pose;
}

semantic_inference_msgs::msg::FeatureVector makeFeature(
    const Eigen::MatrixXf& feature) {
  semantic_inference_msgs::msg::FeatureVector message;
  message.rows = feature.rows();
  message.cols = feature.cols();
  message.data.reserve(feature.size());
  for (int row = 0; row < feature.rows(); ++row) {
    for (int col = 0; col < feature.cols(); ++col) {
      message.data.push_back(feature(row, col));
    }
  }
  return message;
}

bool belongsToRoom(const DynamicSceneGraph& graph,
                   const SceneGraphNode& node,
                   int requested_room) {
  if (requested_room < 0) {
    return true;
  }
  if (node.parents().empty()) {
    return false;
  }
  const auto place_id = *node.parents().begin();
  if (!graph.hasNode(place_id)) {
    return false;
  }
  const auto room_id = graph.getNode(place_id).getParent();
  if (!room_id || !graph.hasNode(*room_id)) {
    return false;
  }
  try {
    return extractNumber(
               graph.getNode(*room_id).attributes<RoomNodeAttributes>().name) ==
           requested_room;
  } catch (const std::runtime_error&) {
    return false;
  }
}

}  // namespace

Ros2VLMRelationships::Ros2VLMRelationships(
    ianvs::NodeHandle nh,
    Config config,
    const InputQueue<BackendVLMLabelsInput::Ptr>::Ptr& labels_queue)
    : config_(std::move(config)),
      labels_queue_(labels_queue),
      request_queue_(config_.queue_size),
      service_(nh.create_service<hydra_msgs::srv::VLMRelationship>(
          config_.service, &Ros2VLMRelationships::handleRequest, this)),
      labels_sub_(nh.create_subscription<hydra_msgs::msg::LabeledRelationships>(
          config_.labels_topic,
          rclcpp::QoS(config_.queue_size),
          &Ros2VLMRelationships::handleLabels,
          this)),
      encodings_pub_(
          nh.create_publisher<hydra_msgs::msg::VisualRelationshipsEncodings>(
              config_.encodings_topic, rclcpp::QoS(config_.queue_size))) {}

void Ros2VLMRelationships::handleRequest(
    const std::shared_ptr<hydra_msgs::srv::VLMRelationship::Request> request,
    std::shared_ptr<hydra_msgs::srv::VLMRelationship::Response> response) {
  if (encodings_pub_->get_subscription_count() == 0) {
    response->success = false;
    return;
  }
  response->success = request_queue_.push(
      {request->prompt.empty() ? config_.default_prompt : request->prompt,
       request->room});
  if (!response->success) {
    LOG(WARNING) << "VLM relationship request queue is full";
  }
}

void Ros2VLMRelationships::call(
    uint64_t timestamp_ns,
    const DynamicSceneGraph& graph,
    const kimera_pgmo::DeformationGraph& /*dgraph*/) {
  if (encodings_pub_->get_subscription_count() == 0 ||
      !request_queue_.poll(0)) {
    return;
  }

  const auto request = request_queue_.pop();
  if (request.prompt == "reset" || request.prompt == "RESET") {
    resetLabels(graph, timestamp_ns);
    return;
  }

  hydra_msgs::msg::VisualRelationshipsEncodings message;
  const auto& objects = graph.getLayer(DsgLayers::OBJECTS);
  for (const auto& [key, edge] : objects.edges()) {
    (void)key;
    const auto& source = objects.getNode(edge.source);
    const auto& target = objects.getNode(edge.target);
    if (!belongsToRoom(graph, source, request.room_id) &&
        !belongsToRoom(graph, target, request.room_id)) {
      continue;
    }

    const auto& source_feature = edge.info->feature(edge.source);
    const auto& target_feature = edge.info->feature(edge.target);
    if (source_feature.size() == 0 || target_feature.size() == 0) {
      continue;
    }

    message.features.feature.push_back(makeFeature(source_feature));
    message.features.feature.push_back(makeFeature(target_feature));
    message.features.ids.insert(message.features.ids.end(),
                                {edge.source, edge.target, edge.target, edge.source});

    const auto& source_attributes =
        source.attributes<SemanticNodeAttributes>();
    const auto& target_attributes =
        target.attributes<SemanticNodeAttributes>();
    const auto source_label = source_attributes.name.empty()
                                  ? NodeSymbol(source.id).getLabel()
                                  : source_attributes.name;
    const auto target_label = target_attributes.name.empty()
                                  ? NodeSymbol(target.id).getLabel()
                                  : target_attributes.name;
    message.object_classes.insert(message.object_classes.end(),
                                  {source_label, target_label,
                                   target_label, source_label});

    const auto source_pose = makePose(source_attributes);
    const auto target_pose = makePose(target_attributes);
    message.object_poses.insert(message.object_poses.end(),
                                {source_pose, target_pose,
                                 target_pose, source_pose});

    const auto source_box = makeBoundingBox(source_attributes);
    const auto target_box = makeBoundingBox(target_attributes);
    auto& boxes = message.object_bounding_boxes.boxes;
    boxes.insert(boxes.end(),
                 {source_box, target_box, target_box, source_box});
  }

  if (message.features.ids.empty()) {
    return;
  }
  message.prompt = request.prompt;
  message.features.header.stamp = rclcpp::Time(timestamp_ns);
  encodings_pub_->publish(message);
}

void Ros2VLMRelationships::handleLabels(
    hydra_msgs::msg::LabeledRelationships::ConstSharedPtr relationships) {
  if (relationships->node_ids.size() < 2 * relationships->labels.size()) {
    LOG(ERROR) << "Ignoring malformed labeled relationships message";
    return;
  }

  auto labels = std::make_shared<VLMLabels>();
  for (size_t index = 0; index < relationships->labels.size(); ++index) {
    labels->labels.push_back(relationships->labels[index]);
    labels->edge_ids.emplace_back(relationships->node_ids[2 * index],
                                  relationships->node_ids[2 * index + 1]);
  }
  if (labels->labels.empty()) {
    return;
  }

  auto input = std::make_shared<BackendVLMLabelsInput>();
  input->vlm_labels = labels;
  input->timestamp_ns = rclcpp::Time(relationships->header.stamp).nanoseconds();
  if (!labels_queue_->push(input)) {
    LOG(WARNING) << "VLM labels queue is full";
  }
}

void Ros2VLMRelationships::resetLabels(const DynamicSceneGraph& graph,
                                       uint64_t timestamp_ns) {
  auto labels = std::make_shared<VLMLabels>();
  for (const auto& [key, edge] : graph.getLayer(DsgLayers::OBJECTS).edges()) {
    (void)key;
    labels->labels.insert(labels->labels.end(), {"", ""});
    labels->edge_ids.emplace_back(edge.source, edge.target);
    labels->edge_ids.emplace_back(edge.target, edge.source);
  }

  auto input = std::make_shared<BackendVLMLabelsInput>();
  input->vlm_labels = labels;
  input->timestamp_ns = timestamp_ns;
  if (!labels_queue_->push(input)) {
    LOG(WARNING) << "VLM labels queue is full";
  }
}

}  // namespace hydra
