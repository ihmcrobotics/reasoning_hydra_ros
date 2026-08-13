#include "hydra_ros/input/ros_sensors.h"

#include <config_utilities/config.h>
#include <config_utilities/factory.h>
#include <config_utilities/printing.h>
#include <config_utilities/types/path.h>
#include <config_utilities/validation.h>
#include <glog/logging.h>
#include <hydra/common/global_info.h>
#include <sensor_msgs/msg/camera_info.hpp>

#include <future>
#include <stdexcept>

#include "hydra_ros/utils/lookup_tf.h"
#include <ianvs/node_handle.h>

namespace hydra {
namespace {

using CameraInfo = sensor_msgs::msg::CameraInfo;

CameraInfo::ConstSharedPtr waitForCameraInfo(const std::string& topic) {
  auto nh = ianvs::NodeHandle::this_node();
  auto promise = std::make_shared<std::promise<CameraInfo::ConstSharedPtr>>();
  auto future = promise->get_future();
  auto received = std::make_shared<std::atomic<bool>>(false);
  auto subscription = nh.create_subscription<CameraInfo>(
      topic,
      rclcpp::SensorDataQoS().keep_last(1),
      [promise, received](const CameraInfo::ConstSharedPtr msg) {
        if (!received->exchange(true)) {
          promise->set_value(msg);
        }
      });

  LOG(INFO) << "Waiting for CameraInfo on " << topic
            << " to initialize sensor model";
  while (rclcpp::ok()) {
    if (future.wait_for(std::chrono::milliseconds(100)) ==
        std::future_status::ready) {
      return future.get();
    }
  }
  return nullptr;
}

void fillConfigFromInfo(const CameraInfo& msg, Camera::Config& config) {
  config.width = msg.width;
  config.height = msg.height;
  config.fx = msg.k[0];
  config.fy = msg.k[4];
  config.cx = msg.k[2];
  config.cy = msg.k[5];
}

void fillConfigFromInfo(const CameraInfo& msg,
                        CameraLidarFusion::Config& config) {
  config.width = msg.width;
  config.height = msg.height;
  config.fx = msg.k[0];
  config.fy = msg.k[4];
  config.cx = msg.k[2];
  config.cy = msg.k[5];
  CHECK_GE(msg.d.size(), 4u) << "CameraInfo requires four distortion coefficients";
  config.k1 = msg.d[0];
  config.k2 = msg.d[1];
  config.k3 = msg.d[2];
  config.k4 = msg.d[3];
  const auto pose = lookupTransform(
      GlobalInfo::instance().getFrames().robot, msg.header.frame_id);
  CHECK(pose.is_valid) << "Could not look up camera extrinsics from ROS 2 TF";
  config.cam2body_rotation = pose.target_R_source;
  config.cam2body_translation = pose.target_p_source;
}

}  // namespace

RosSensorExtrinsics::RosSensorExtrinsics(const Config& config)
    : SensorExtrinsics() {
  config::checkValid(config);
  const auto pose = lookupTransform(
      GlobalInfo::instance().getFrames().robot, config.sensor_frame);
  CHECK(pose.is_valid) << "Could not look up sensor extrinsics from ROS 2 TF";
  body_R_sensor = pose.target_R_source;
  body_p_sensor = pose.target_p_source;
}

RosbagExtrinsics::RosbagExtrinsics(const Config& config) : SensorExtrinsics() {
  config::checkValid(config);
  throw std::runtime_error(
      "The 'rosbag' extrinsics factory uses the ROS 1 bag API and is disabled "
      "in the ROS 2 build. Publish the bag's /tf_static data and use type: ros.");
}

RosCameraIntrinsics::RosCameraIntrinsics(const Config& config)
    : Camera(makeCameraConfig(YAML::Node(), config)) {}

RosCameraLidarIntrinsics::RosCameraLidarIntrinsics(const Config& config)
    : CameraLidarFusion(makeCameraConfig(YAML::Node(), config)) {}

RosbagCameraIntrinsics::RosbagCameraIntrinsics(const Config& config)
    : Camera(makeCameraConfig(YAML::Node(), config)) {}

Camera::Config RosCameraIntrinsics::makeCameraConfig(const YAML::Node&,
                                                     const Config& config) {
  const auto msg = waitForCameraInfo(config.topic);
  if (!msg) {
    throw std::runtime_error("ROS shutdown while waiting for CameraInfo on " +
                             config.topic);
  }
  Camera::Config output;
  static_cast<Sensor::Config&>(output) = static_cast<const Sensor::Config&>(config);
  fillConfigFromInfo(*msg, output);
  LOG(INFO) << "Initialized camera as\n" << config::toString(output);
  return output;
}

CameraLidarFusion::Config RosCameraLidarIntrinsics::makeCameraConfig(
    const YAML::Node&, const Config& config) {
  const auto msg = waitForCameraInfo(config.topic);
  if (!msg) {
    throw std::runtime_error("ROS shutdown while waiting for CameraInfo on " +
                             config.topic);
  }
  CameraLidarFusion::Config output;
  static_cast<Sensor::Config&>(output) = static_cast<const Sensor::Config&>(config);
  fillConfigFromInfo(*msg, output);
  LOG(INFO) << "Initialized camera-lidar fusion sensor as\n"
            << config::toString(output);
  return output;
}

Camera::Config RosbagCameraIntrinsics::makeCameraConfig(
    const YAML::Node&, const Config&) {
  throw std::runtime_error(
      "The ROS 1 rosbag camera-info reader is disabled in the ROS 2 build");
}

void declare_config(RosSensorExtrinsics::Config& config) {
  using namespace config;
  name("RosSensorExtrinsics::Config");
  field(config.sensor_frame, "sensor_frame");
  checkCondition(!config.sensor_frame.empty(), "sensor frame required");
}

void declare_config(RosbagExtrinsics::Config& config) {
  using namespace config;
  name("RosbagExtrinsics::Config");
  field(config.sensor_frame, "sensor_frame");
  field<Path>(config.bag_path, "bag_path");
  checkCondition(!config.sensor_frame.empty(), "sensor frame required");
}

void declare_config(RosCameraIntrinsics::Config& config) {
  using namespace config;
  name("RosCameraIntrinsics::Config");
  base<Sensor::Config>(config);
  field(config.topic, "camera_info_topic");
  checkCondition(!config.topic.empty(), "camera info topic required");
}

void declare_config(RosCameraLidarIntrinsics::Config& config) {
  using namespace config;
  name("RosCameraLidarIntrinsics::Config");
  base<Sensor::Config>(config);
  field(config.topic, "camera_info_topic");
  checkCondition(!config.topic.empty(), "camera info topic required");
}

void declare_config(RosbagCameraIntrinsics::Config& config) {
  using namespace config;
  name("RosbagCameraIntrinsics::Config");
  base<Sensor::Config>(config);
  field(config.topic, "camera_info_topic");
  field<Path>(config.bag_path, "bag_path");
  checkCondition(!config.topic.empty(), "camera info topic required");
}

}  // namespace hydra
