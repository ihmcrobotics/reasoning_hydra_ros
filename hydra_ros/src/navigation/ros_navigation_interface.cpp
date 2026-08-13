// Copyright (c) 2026, IHMC Robotics Lab.
// All rights reserved.

#include "hydra_ros/navigation/ros_navigation_interface.h"

#include <geometry_msgs/msg/point.hpp>
#include <hydra/common/global_info.h>
#include <hydra_msgs/msg/navigation_path.hpp>
#include <hydra_msgs/msg/object_search_pair.hpp>
#include <semantic_inference_msgs/msg/feature_vector.hpp>

#include <chrono>
#include <unordered_map>

namespace hydra {
namespace {

Eigen::MatrixXf toMatrix(const semantic_inference_msgs::msg::FeatureVector& msg) {
  if (msg.rows <= 0 || msg.cols <= 0 ||
      msg.data.size() != static_cast<size_t>(msg.rows * msg.cols)) {
    return {};
  }
  return Eigen::Map<const Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>>(
      msg.data.data(), msg.rows, msg.cols);
}

semantic_inference_msgs::msg::FeatureVector toMessage(
    const Eigen::MatrixXf& matrix) {
  semantic_inference_msgs::msg::FeatureVector msg;
  msg.rows = matrix.rows();
  msg.cols = matrix.cols();
  msg.data.reserve(matrix.size());
  for (int row = 0; row < matrix.rows(); ++row) {
    for (int col = 0; col < matrix.cols(); ++col) {
      msg.data.push_back(matrix(row, col));
    }
  }
  return msg;
}

}  // namespace

Ros2NavigationInterface::Ros2NavigationInterface(
    ianvs::NodeHandle nh,
    Config config,
    SharedDsgInfo::Ptr backend_graph,
    NavigationModule::Ptr navigation,
    ObjectSearchModule::Ptr object_search)
    : config_(std::move(config)),
      backend_graph_(std::move(backend_graph)),
      navigation_(std::move(navigation)),
      object_search_(std::move(object_search)),
      find_paths_service_(nh.create_service<hydra_msgs::srv::GetNavigation>(
          config_.find_paths_service,
          &Ros2NavigationInterface::handleFindPaths,
          this)),
      reset_service_(nh.create_service<std_srvs::srv::Empty>(
          config_.reset_service, &Ros2NavigationInterface::handleReset, this)),
      prompts_sub_(nh.create_subscription<
                   semantic_inference_msgs::msg::NavigationPromptsEmbeddings>(
          config_.prompt_embeddings_topic,
          rclcpp::QoS(10),
          &Ros2NavigationInterface::handlePromptEmbeddings,
          this)),
      navigation_pub_(nh.create_publisher<hydra_msgs::msg::Navigation>(
          config_.navigation_output_topic, rclcpp::QoS(10))),
      object_search_pub_(nh.create_publisher<hydra_msgs::msg::ObjectSearch>(
          config_.object_search_output_topic, rclcpp::QoS(10))),
      output_thread_(&Ros2NavigationInterface::spinOutputs, this) {}

Ros2NavigationInterface::~Ros2NavigationInterface() {
  should_stop_ = true;
  if (output_thread_.joinable()) {
    output_thread_.join();
  }
}

void Ros2NavigationInterface::updateGraphs() {
  std::lock_guard<std::mutex> lock(backend_graph_->mutex);
  const auto graph = backend_graph_->graph->clone();
  navigation_->setGraph(graph);
  object_search_->setGraph(graph->clone());
}

void Ros2NavigationInterface::handleFindPaths(
    const std::shared_ptr<hydra_msgs::srv::GetNavigation::Request> request,
    std::shared_ptr<hydra_msgs::srv::GetNavigation::Response> response) {
  if (request->object_ids.empty()) {
    response->success = false;
    response->message = "No object ids provided";
    return;
  }
  if (request->object_ids.size() % 2 != 0) {
    response->success = false;
    response->message = "Object ids must be provided in pairs";
    return;
  }
  if (request->explanation.size() != request->object_ids.size() / 2) {
    response->success = false;
    response->message = "Explanation count does not match object-id pairs";
    return;
  }

  updateGraphs();
  auto input = std::make_shared<NavigationInput>();
  input->method = request->method;
  input->explanation = request->explanation;
  for (size_t index = 0; index < request->explanation.size(); ++index) {
    input->object_ids.emplace_back(request->object_ids[2 * index],
                                   request->object_ids[2 * index + 1]);
  }
  response->success = navigation_->inputQueue()->push(input);
  response->message = response->success ? "Navigation request queued"
                                        : "Navigation request queue is full";
}

void Ros2NavigationInterface::handleReset(
    const std::shared_ptr<std_srvs::srv::Empty::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Empty::Response> /*response*/) {
  while (!navigation_->outputQueue()->empty()) {
    navigation_->outputQueue()->pop();
  }
  while (!object_search_->outputQueue()->empty()) {
    object_search_->outputQueue()->pop();
  }
}

void Ros2NavigationInterface::handlePromptEmbeddings(
    semantic_inference_msgs::msg::NavigationPromptsEmbeddings::ConstSharedPtr msg) {
  updateGraphs();
  auto input = std::make_shared<ObjectSearchInput>();
  input->room = msg->room;
  input->prompt = msg->prompt;
  input->object_search = msg->object_search;
  input->text_room_embedding.data = toMatrix(msg->text_room_embedding.feature);
  input->text_room_embedding.rows = msg->text_room_embedding.feature.rows;
  input->text_room_embedding.cols = msg->text_room_embedding.feature.cols;

  for (const auto& feature : msg->text_object_embedding) {
    ObjectSearchInput::ObjectFeature output;
    output.data = toMatrix(feature.feature);
    output.rows = feature.feature.rows;
    output.cols = feature.feature.cols;
    input->text_object_embedding.push_back(std::move(output));
  }

  std::unordered_map<std::string, size_t> label_indices;
  for (size_t index = 0; index < msg->objects.size(); ++index) {
    label_indices[msg->objects[index]] = index;
  }
  for (const auto& pair : msg->objects_prompt_pairs) {
    const auto object = label_indices.find(pair.object);
    const auto subject = label_indices.find(pair.subject);
    if (object == label_indices.end() || subject == label_indices.end()) {
      LOG(WARNING) << "Ignoring navigation prompt pair with unknown labels";
      continue;
    }
    ObjectSearchInput::ObjectsPtomptsPair output;
    output.object_label_index = object->second;
    output.subject_label_index = subject->second;
    output.prompt = pair.prompt;
    input->objects_prompt_pairs.push_back(std::move(output));
  }

  if (!object_search_->inputQueue()->push(input)) {
    LOG(WARNING) << "Object-search request queue is full";
  }
}

void Ros2NavigationInterface::spinOutputs() {
  while (!should_stop_) {
    if (navigation_->outputQueue()->poll(10000)) {
      publishNavigation(navigation_->outputQueue()->pop());
    }
    if (object_search_->outputQueue()->poll(10000)) {
      publishObjectSearch(object_search_->outputQueue()->pop());
    }
  }
}

void Ros2NavigationInterface::publishNavigation(const NavigationOutput& output) {
  hydra_msgs::msg::Navigation msg;
  msg.header.stamp = rclcpp::Clock().now();
  msg.header.frame_id = GlobalInfo::instance().getFrames().odom;
  for (const auto& path : output.paths) {
    hydra_msgs::msg::NavigationPath path_msg;
    path_msg.object_id = path.object_id;
    path_msg.target_id = path.target_id;
    path_msg.object_label = path.object_label;
    path_msg.target_label = path.target_label;
    path_msg.explanation = path.explanation;
    for (const auto& point : path.agent_to_target) {
      geometry_msgs::msg::Point point_msg;
      point_msg.x = point.x();
      point_msg.y = point.y();
      point_msg.z = point.z();
      path_msg.agent_to_target.push_back(point_msg);
    }
    for (const auto& point : path.target_to_object) {
      geometry_msgs::msg::Point point_msg;
      point_msg.x = point.x();
      point_msg.y = point.y();
      point_msg.z = point.z();
      path_msg.target_to_object.push_back(point_msg);
    }
    msg.navigation_paths.push_back(std::move(path_msg));
  }
  if (!msg.navigation_paths.empty()) {
    navigation_pub_->publish(msg);
  }
}

void Ros2NavigationInterface::publishObjectSearch(
    const ObjectSearchOutput::Ptr& output) {
  if (output->object_search) {
    auto input = std::make_shared<NavigationInput>();
    input->method = "dijkstra";
    input->object_search = true;
    for (const auto& object : output->objects) {
      input->object_ids.emplace_back(object.id, object.id);
    }
    navigation_->inputQueue()->push(input);
    return;
  }

  hydra_msgs::msg::ObjectSearch msg;
  msg.header.stamp = rclcpp::Clock().now();
  msg.general_prompt = output->general_prompt;
  for (const auto& object : output->objects) {
    msg.object_ids.push_back(object.id);
    hydra_msgs::msg::ObjectSearchPair pair_msg;
    for (const auto& relationship : object.relationships) {
      pair_msg.feature.push_back(toMessage(relationship.feature));
      pair_msg.ids.insert(pair_msg.ids.end(),
                          {relationship.object1, relationship.object2});
      pair_msg.labels.insert(pair_msg.labels.end(),
                             {relationship.object1_label,
                              relationship.object2_label});
      pair_msg.prompt.push_back(relationship.prompt);
      pair_msg.num_observations.push_back(relationship.num_observations);
    }
    msg.features.push_back(std::move(pair_msg));
  }
  object_search_pub_->publish(msg);
}

}  // namespace hydra
