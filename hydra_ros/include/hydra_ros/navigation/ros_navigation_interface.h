// Copyright (c) 2026, IHMC Robotics Lab.
// All rights reserved.

#pragma once

#include <hydra/navigation/navigation_module.h>
#include <hydra/navigation/object_search_module.h>
#include <hydra_msgs/msg/navigation.hpp>
#include <hydra_msgs/msg/object_search.hpp>
#include <hydra_msgs/srv/get_navigation.hpp>
#include <ianvs/node_handle.h>
#include <semantic_inference_msgs/msg/navigation_prompts_embeddings.hpp>
#include <std_srvs/srv/empty.hpp>

#include <atomic>
#include <thread>

namespace hydra {

class Ros2NavigationInterface {
 public:
  struct Config {
    std::string find_paths_service = "/hydra_ros_node/navigation/find_paths";
    std::string reset_service =
        "/hydra_dsg_visualizer/reset_object_search_coloring";
    std::string prompt_embeddings_topic =
        "/semantic_inference/navigation_embeddings";
    std::string navigation_output_topic =
        "/hydra_ros_node/navigation/navigation_output";
    std::string object_search_output_topic =
        "/hydra_ros_node/navigation/object_search_output";
  };

  Ros2NavigationInterface(ianvs::NodeHandle nh,
                          Config config,
                          SharedDsgInfo::Ptr backend_graph,
                          NavigationModule::Ptr navigation,
                          ObjectSearchModule::Ptr object_search);
  ~Ros2NavigationInterface();

 private:
  void handleFindPaths(
      const std::shared_ptr<hydra_msgs::srv::GetNavigation::Request> request,
      std::shared_ptr<hydra_msgs::srv::GetNavigation::Response> response);
  void handleReset(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
                   std::shared_ptr<std_srvs::srv::Empty::Response> response);
  void handlePromptEmbeddings(
      semantic_inference_msgs::msg::NavigationPromptsEmbeddings::ConstSharedPtr msg);
  void updateGraphs();
  void spinOutputs();
  void publishNavigation(const NavigationOutput& output);
  void publishObjectSearch(const ObjectSearchOutput::Ptr& output);

  const Config config_;
  SharedDsgInfo::Ptr backend_graph_;
  NavigationModule::Ptr navigation_;
  ObjectSearchModule::Ptr object_search_;
  std::atomic<bool> should_stop_{false};

  ianvs::NodeHandle::Service<hydra_msgs::srv::GetNavigation> find_paths_service_;
  ianvs::NodeHandle::Service<std_srvs::srv::Empty> reset_service_;
  ianvs::NodeHandle::Subscription<
      semantic_inference_msgs::msg::NavigationPromptsEmbeddings> prompts_sub_;
  ianvs::NodeHandle::Publisher<hydra_msgs::msg::Navigation> navigation_pub_;
  ianvs::NodeHandle::Publisher<hydra_msgs::msg::ObjectSearch> object_search_pub_;
  std::thread output_thread_;
};

}  // namespace hydra
