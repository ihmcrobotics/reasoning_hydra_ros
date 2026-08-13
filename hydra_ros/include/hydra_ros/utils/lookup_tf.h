#pragma once

#include <hydra/input/input_module.h>
#include <rclcpp/time.hpp>
#include <tf2_ros/buffer.h>

#include <optional>
#include <string>

namespace hydra {

PoseStatus lookupTransform(const tf2_ros::Buffer& buffer,
                           const std::optional<rclcpp::Time>& stamp,
                           const std::string& target,
                           const std::string& source,
                           std::optional<size_t> max_tries = std::nullopt,
                           double wait_duration_s = 0.1,
                           int verbosity = 10);

PoseStatus lookupTransform(const std::string& target,
                           const std::string& source,
                           double wait_duration_s = 0.1,
                           int verbosity = 10,
                           bool nonblocking = false);

}  // namespace hydra
