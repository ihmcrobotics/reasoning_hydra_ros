#include <config_utilities/config.h>
#include <config_utilities/parsing/context.h>
#include <config_utilities/printing.h>
#include <config_utilities/settings.h>
#include <glog/logging.h>
#include <hydra/common/global_info.h>
#include <hydra_msgs/srv/load.hpp>
#include <ianvs/node_init.h>
#include <ianvs/spin_functions.h>
#include <std_srvs/srv/trigger.hpp>

#include <atomic>
#include <exception>
#include <memory>

#include "hydra_ros/hydra_ros_pipeline.h"

namespace hydra {

struct RunSettings {
  int robot_id = 0;
  int config_verbosity = 1;
  bool exit_after_clock = false;
};

void declare_config(RunSettings& config) {
  using namespace config;
  name("RunSettings");
  field(config.robot_id, "robot_id");
  field(config.config_verbosity, "config_verbosity");
  field(config.exit_after_clock, "exit_after_clock");
}

class NodeWrapper {
 public:
  NodeWrapper(ianvs::NodeHandle nh, const RunSettings& settings)
      : nh_(nh),
        pipeline_(std::make_unique<HydraRosPipeline>(settings.robot_id,
                                                    settings.config_verbosity)) {
    save_service_ = nh_.create_service<std_srvs::srv::Trigger>(
        "save_graph", &NodeWrapper::saveGraph, this);
    load_service_ = nh_.create_service<hydra_msgs::srv::Load>(
        "load_graph", &NodeWrapper::loadGraph, this);
    pipeline_->init();
  }

  ~NodeWrapper() = default;

  void start() {
    pipeline_->start();
    running_ = true;
  }

  void stop() {
    if (!running_.exchange(false)) {
      return;
    }
    pipeline_->stop();
  }

  void save() { pipeline_->save(); }

 private:
  void saveGraph(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    try {
      save();
      response->success = true;
    } catch (const std::exception& error) {
      response->success = false;
      response->message = "Failed to save graph: " + std::string(error.what());
      LOG(ERROR) << response->message;
    }
  }

  void loadGraph(const std::shared_ptr<hydra_msgs::srv::Load::Request> request,
                 std::shared_ptr<hydra_msgs::srv::Load::Response> response) {
    if (running_) {
      response->success = false;
      response->message = "Cannot load a graph while the pipeline is running";
      return;
    }

    try {
      pipeline_->loadGraph(request->filepath);
      response->success = true;
    } catch (const std::exception& error) {
      response->success = false;
      response->message = "Failed to load graph: " + std::string(error.what());
      LOG(ERROR) << response->message;
    }
  }

  ianvs::NodeHandle nh_;
  std::unique_ptr<HydraRosPipeline> pipeline_;
  ianvs::NodeHandle::Service<std_srvs::srv::Trigger> save_service_;
  ianvs::NodeHandle::Service<hydra_msgs::srv::Load> load_service_;
  std::atomic<bool> running_{false};
};

}  // namespace hydra

int main(int argc, char* argv[]) {
  config::initContext(argc, argv, true);
  config::setConfigSettingsFromContext();
  const auto settings = config::fromContext<hydra::RunSettings>();

  FLAGS_logtostderr = 1;
  FLAGS_colorlogtostderr = 1;
  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();
  config::Settings().setLogger("glog");

  [[maybe_unused]] const auto node_guard =
      ianvs::init_node(argc, argv, "hydra_ros_node");
  auto nh = ianvs::NodeHandle::this_node("~");

  int exit_code = 0;
  try {
    // Ensure modules are destroyed before GlobalInfo and the ROS node.
    hydra::NodeWrapper wrapper(nh, settings);
    wrapper.start();
    ianvs::spinAndWait(nh, settings.exit_after_clock);
    wrapper.stop();
    wrapper.save();
  } catch (const std::exception& error) {
    if (rclcpp::ok()) {
      LOG(ERROR) << "Hydra initialization failed: " << error.what();
      exit_code = 1;
    } else {
      LOG(INFO) << "Hydra initialization canceled during ROS shutdown";
    }
  }

  hydra::GlobalInfo::exit();
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return exit_code;
}
