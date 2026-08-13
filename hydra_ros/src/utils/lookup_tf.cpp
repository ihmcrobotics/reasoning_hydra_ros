#include "hydra_ros/utils/lookup_tf.h"

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <glog/logging.h>
#include <rclcpp/rclcpp.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/transform_listener.h>

#include <ianvs/node_handle.h>

namespace hydra {

PoseStatus lookupTransform(const tf2_ros::Buffer& buffer,
                           const std::optional<rclcpp::Time>& stamp,
                           const std::string& target,
                           const std::string& source,
                           std::optional<size_t> max_tries,
                           double wait_duration_s,
                           int verbosity) {
  rclcpp::WallRate wait_rate(1.0 / wait_duration_s);
  const auto lookup_time = stamp.value_or(rclcpp::Time());
  std::string error;
  size_t attempt = 0;
  while (rclcpp::ok()) {
    if (max_tries && attempt >= *max_tries) {
      break;
    }
    if (buffer.canTransform(target,
                            source,
                            lookup_time,
                            rclcpp::Duration::from_seconds(0.0),
                            &error)) {
      try {
        const auto transform = buffer.lookupTransform(target, source, lookup_time);
        PoseStatus result;
        result.is_valid = true;
        tf2::fromMsg(transform.transform.translation, result.target_p_source);
        tf2::fromMsg(transform.transform.rotation, result.target_R_source);
        result.target_R_source.normalize();
        return result;
      } catch (const tf2::TransformException& ex) {
        error = ex.what();
      }
    }
    ++attempt;
    wait_rate.sleep();
  }

  VLOG(verbosity) << "Failed to find " << target << "_T_" << source
                  << ": " << error;
  return PoseStatus(false);
}

PoseStatus lookupTransform(const std::string& target,
                           const std::string& source,
                           double wait_duration_s,
                           int verbosity,
                           bool nonblocking) {
  auto nh = ianvs::NodeHandle::this_node();
  tf2_ros::Buffer buffer(nh.clock());
  tf2_ros::TransformListener listener(buffer);
  const std::optional<size_t> tries = nonblocking ? std::optional<size_t>(1)
                                                  : std::nullopt;
  return lookupTransform(
      buffer, std::nullopt, target, source, tries, wait_duration_s, verbosity);
}

}  // namespace hydra
