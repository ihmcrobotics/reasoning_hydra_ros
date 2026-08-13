#include "hydra_ui.h"

#include <QHBoxLayout>
#include <QLabel>
#include <QVBoxLayout>

#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>

namespace hydra_ui {

HydraPanel::HydraPanel(QWidget* parent) : rviz_common::Panel(parent) {
  auto* layout = new QVBoxLayout;
  button_find_next_object = new QPushButton("Find Next Object");
  button_publish_waypoints = new QPushButton("Publish Waypoints");
  button_clear_navigation = new QPushButton("Clear Navigation");
  button_vlm_process_next = new QPushButton("VLM Process Next");
  button_vlm_publish_current = new QPushButton("VLM Publish Current");
  button_reset_mesh = new QPushButton("Reset Mesh");
  button_toggle_processing = new QPushButton("Toggle Processing");
  button_repeat_last_vlm = new QPushButton("Repeat Last VLM");
  button_stop_vlm = new QPushButton("Stop VLM");
  prompt_input = new QLineEdit;
  room_input = new QLineEdit;
  button_send_task = new QPushButton("Send Task");

  prompt_input->setPlaceholderText("Enter prompt...");
  room_input->setPlaceholderText("Enter room...");
  layout->addWidget(button_find_next_object);
  layout->addWidget(button_publish_waypoints);
  layout->addWidget(button_clear_navigation);
  layout->addWidget(button_vlm_publish_current);
  layout->addWidget(button_vlm_process_next);
  layout->addWidget(button_repeat_last_vlm);
  layout->addWidget(button_stop_vlm);
  layout->addWidget(button_reset_mesh);
  layout->addWidget(button_toggle_processing);
  layout->addWidget(button_send_task);
  layout->addWidget(new QLabel("Task:"));
  layout->addWidget(prompt_input);
  layout->addWidget(new QLabel("Room:"));
  layout->addWidget(room_input);
  setLayout(layout);

  connect(button_find_next_object, &QPushButton::clicked, this,
          &HydraPanel::onFindNextObjectClick);
  connect(button_publish_waypoints, &QPushButton::clicked, this,
          &HydraPanel::onPublishWaypointsClick);
  connect(button_clear_navigation, &QPushButton::clicked, this,
          &HydraPanel::onClearNavigationClick);
  connect(button_vlm_process_next, &QPushButton::clicked, this,
          &HydraPanel::onVLMProcessNextClick);
  connect(button_vlm_publish_current, &QPushButton::clicked, this,
          &HydraPanel::onVLMPublishCurrentClick);
  connect(button_reset_mesh, &QPushButton::clicked, this,
          &HydraPanel::onResetMeshClick);
  connect(button_toggle_processing, &QPushButton::clicked, this,
          &HydraPanel::onToggleProcessingClick);
  connect(button_repeat_last_vlm, &QPushButton::clicked, this,
          &HydraPanel::onRepeatLastVLMClick);
  connect(button_stop_vlm, &QPushButton::clicked, this,
          &HydraPanel::onStopVLMClick);
  connect(button_send_task, &QPushButton::clicked, this,
          &HydraPanel::onSendTaskClick);
}

void HydraPanel::onInitialize() {
  auto abstraction = getDisplayContext()->getRosNodeAbstraction().lock();
  if (!abstraction) {
    return;
  }
  node_ = abstraction->get_raw_node();
  hydra_client_find_next_object =
      node_->create_client<Empty>("/hydra_ros_node/navigation/find_next");
  hydra_client_publish_waypoints =
      node_->create_client<Empty>("/hydra_ros_node/navigation/publish_waypoints");
  hydra_client_clear_navigation =
      node_->create_client<Empty>("/hydra_ros_node/navigation/clear_navigation");
  hydra_client_vlm_process_next = node_->create_client<Empty>(
      "/semantic_inference/vlm_for_navigation_node/process_next");
  hydra_client_vlm_publish_current = node_->create_client<Empty>(
      "/semantic_inference/vlm_for_navigation_node/publish_current");
  hydra_client_reset_mesh = node_->create_client<Empty>(
      "/hydra_dsg_visualizer/reset_object_search_coloring");
  hydra_client_toggle_processing =
      node_->create_client<Trigger>("/hydra_ros_node/toggle_image_processing");
  hydra_client_repeat_last_vlm = node_->create_client<Empty>(
      "/semantic_inference/vlm_for_navigation_node/repeat_processing");
  hydra_client_stop_vlm = node_->create_client<Empty>(
      "/semantic_inference/vlm_for_navigation_node/stop_processing");
  hydra_client_send_task = node_->create_client<NavigationPrompt>(
      "/semantic_inference/navigation_prompt_service/navigation_prompt");
}

void HydraPanel::callEmpty(const rclcpp::Client<Empty>::SharedPtr& client,
                           const char* service_name) {
  if (!client) {
    RCLCPP_ERROR(rclcpp::get_logger("hydra_ui"),
                 "RViz panel has not finished initializing");
    return;
  }
  if (!client->service_is_ready()) {
    RCLCPP_ERROR(rclcpp::get_logger("hydra_ui"),
                 "Service is not available: %s", service_name);
    return;
  }
  client->async_send_request(std::make_shared<Empty::Request>());
}

void HydraPanel::onFindNextObjectClick() {
  callEmpty(hydra_client_find_next_object, "/hydra_ros_node/navigation/find_next");
}
void HydraPanel::onPublishWaypointsClick() {
  callEmpty(hydra_client_publish_waypoints,
            "/hydra_ros_node/navigation/publish_waypoints");
}
void HydraPanel::onClearNavigationClick() {
  callEmpty(hydra_client_clear_navigation,
            "/hydra_ros_node/navigation/clear_navigation");
}
void HydraPanel::onVLMProcessNextClick() {
  callEmpty(hydra_client_vlm_process_next,
            "/semantic_inference/vlm_for_navigation_node/process_next");
}
void HydraPanel::onVLMPublishCurrentClick() {
  callEmpty(hydra_client_vlm_publish_current,
            "/semantic_inference/vlm_for_navigation_node/publish_current");
}
void HydraPanel::onResetMeshClick() {
  callEmpty(hydra_client_reset_mesh,
            "/hydra_dsg_visualizer/reset_object_search_coloring");
}
void HydraPanel::onRepeatLastVLMClick() {
  callEmpty(hydra_client_repeat_last_vlm,
            "/semantic_inference/vlm_for_navigation_node/repeat_processing");
}
void HydraPanel::onStopVLMClick() {
  callEmpty(hydra_client_stop_vlm,
            "/semantic_inference/vlm_for_navigation_node/stop_processing");
}

void HydraPanel::onToggleProcessingClick() {
  if (!hydra_client_toggle_processing ||
      !hydra_client_toggle_processing->service_is_ready()) {
    RCLCPP_ERROR(rclcpp::get_logger("hydra_ui"),
                 "Service is not available: /hydra_ros_node/toggle_image_processing");
    return;
  }
  hydra_client_toggle_processing->async_send_request(
      std::make_shared<Trigger::Request>(),
      [](rclcpp::Client<Trigger>::SharedFuture future) {
        const auto response = future.get();
        if (response->success) {
          RCLCPP_INFO(rclcpp::get_logger("hydra_ui"),
                      "Image processing toggled successfully");
        } else {
          RCLCPP_WARN(rclcpp::get_logger("hydra_ui"), "%s",
                      response->message.c_str());
        }
      });
}

void HydraPanel::onSendTaskClick() {
  auto request = std::make_shared<NavigationPrompt::Request>();
  request->prompt = prompt_input->text().toStdString();
  request->room = room_input->text().toStdString();
  if (request->prompt.empty() || request->room.empty()) {
    RCLCPP_ERROR(rclcpp::get_logger("hydra_ui"),
                 "Prompt and room must not be empty");
    return;
  }
  if (!hydra_client_send_task || !hydra_client_send_task->service_is_ready()) {
    RCLCPP_ERROR(rclcpp::get_logger("hydra_ui"),
                 "Navigation prompt service is not available");
    return;
  }
  hydra_client_send_task->async_send_request(
      request, [](rclcpp::Client<NavigationPrompt>::SharedFuture future) {
        const auto response = future.get();
        if (!response->error.empty()) {
          RCLCPP_ERROR(rclcpp::get_logger("hydra_ui"), "%s",
                       response->error.c_str());
          return;
        }
        std::string output = "Task sent successfully. Objects to find:";
        for (const auto& object : response->response.objects) {
          output += " " + object;
        }
        RCLCPP_INFO(rclcpp::get_logger("hydra_ui"), "%s", output.c_str());
      });
}

void HydraPanel::save(rviz_common::Config config) const {
  rviz_common::Panel::save(config);
}
void HydraPanel::load(const rviz_common::Config& config) {
  rviz_common::Panel::load(config);
}

}  // namespace hydra_ui

PLUGINLIB_EXPORT_CLASS(hydra_ui::HydraPanel, rviz_common::Panel)
