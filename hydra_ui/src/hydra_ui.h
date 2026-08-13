#pragma once

#include <memory>

#include <QLineEdit>
#include <QPushButton>

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/config.hpp>
#include <rviz_common/panel.hpp>
#include <semantic_inference_msgs/srv/navigation_prompt.hpp>
#include <std_srvs/srv/empty.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace hydra_ui {

class HydraPanel : public rviz_common::Panel {
  Q_OBJECT

 public:
  explicit HydraPanel(QWidget* parent = nullptr);
  void onInitialize() override;
  void load(const rviz_common::Config& config) override;
  void save(rviz_common::Config config) const override;

 public Q_SLOTS:
  void onFindNextObjectClick();
  void onPublishWaypointsClick();
  void onClearNavigationClick();
  void onVLMProcessNextClick();
  void onVLMPublishCurrentClick();
  void onResetMeshClick();
  void onToggleProcessingClick();
  void onRepeatLastVLMClick();
  void onStopVLMClick();
  void onSendTaskClick();

 private:
  using Empty = std_srvs::srv::Empty;
  using Trigger = std_srvs::srv::Trigger;
  using NavigationPrompt = semantic_inference_msgs::srv::NavigationPrompt;

  void callEmpty(const rclcpp::Client<Empty>::SharedPtr& client,
                 const char* service_name);

  QPushButton* button_find_next_object;
  QPushButton* button_publish_waypoints;
  QPushButton* button_clear_navigation;
  QPushButton* button_vlm_process_next;
  QPushButton* button_vlm_publish_current;
  QPushButton* button_reset_mesh;
  QPushButton* button_toggle_processing;
  QPushButton* button_repeat_last_vlm;
  QPushButton* button_stop_vlm;
  QLineEdit* prompt_input;
  QLineEdit* room_input;
  QPushButton* button_send_task;

  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<Empty>::SharedPtr hydra_client_find_next_object;
  rclcpp::Client<Empty>::SharedPtr hydra_client_publish_waypoints;
  rclcpp::Client<Empty>::SharedPtr hydra_client_clear_navigation;
  rclcpp::Client<Empty>::SharedPtr hydra_client_vlm_process_next;
  rclcpp::Client<Empty>::SharedPtr hydra_client_vlm_publish_current;
  rclcpp::Client<Empty>::SharedPtr hydra_client_reset_mesh;
  rclcpp::Client<Trigger>::SharedPtr hydra_client_toggle_processing;
  rclcpp::Client<Empty>::SharedPtr hydra_client_repeat_last_vlm;
  rclcpp::Client<Empty>::SharedPtr hydra_client_stop_vlm;
  rclcpp::Client<NavigationPrompt>::SharedPtr hydra_client_send_task;
};

}  // namespace hydra_ui
