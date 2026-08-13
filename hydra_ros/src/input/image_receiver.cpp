// Portions of the following code and their modifications are originally from
// https://github.com/MIT-SPARK/Hydra/tree/main and are licensed under the following
// license:
/* -----------------------------------------------------------------------------
 * Copyright 2022 Massachusetts Institute of Technology.
 * All Rights Reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  1. Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *
 *  2. Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Research was sponsored by the United States Air Force Research Laboratory and
 * the United States Air Force Artificial Intelligence Accelerator and was
 * accomplished under Cooperative Agreement Number FA8750-19-2-1000. The views
 * and conclusions contained in this document are those of the authors and should
 * not be interpreted as representing the official policies, either expressed or
 * implied, of the United States Air Force or the U.S. Government. The U.S.
 * Government is authorized to reproduce and distribute reprints for Government
 * purposes notwithstanding any copyright notation herein.
 * -------------------------------------------------------------------------- */

// Copyright (c) 2025, Autonomous Robots Lab, Norwegian University of Science and
// Technology All rights reserved.
//
// Copyright (c) 2026, IHMC Robotics Lab.
// All rights reserved.

// This source code is licensed under the BSD-style license found in the
// LICENSE file in the root directory of this source tree.
#include "hydra_ros/input/image_receiver.h"

#include <config_utilities/config.h>
#include <cv_bridge/cv_bridge.hpp>
#include <glog/logging.h>
#include <rclcpp/time.hpp>
#include <sensor_msgs/image_encodings.hpp>

#include <ianvs/node_handle.h>
#include <ianvs/node_init.h>

namespace hydra {

void declare_config(ImageReceiver::Config& config) {
  using namespace config;
  name("ImageReceiver::Config");
  base<DataReceiver::Config>(config);
  field(config.ns, "ns");
  field(config.queue_size, "queue_size");
}

ImageReceiver::ImageReceiver(const Config& config, size_t sensor_id)
    : DataReceiver(config, sensor_id), config(config) {}

bool ImageReceiver::initImpl() {
  // TODO(nathan) subscribe to image subsets
  // color_sub_ = ImageSubscriber(nh_, "rgb");
  // depth_sub_ = ImageSubscriber(nh_, "depth_registered", "image_rect");

  auto nh = ianvs::NodeHandle::this_node(config.ns);
  auto* node = ianvs::CurrentNode::get();
  CHECK(node) << "ianvs current node is not initialized";
  enable_processing_service_ = nh.create_service<std_srvs::srv::Trigger>(
      "toggle_image_processing",
      std::bind(&ImageReceiver::toggleProcessingService,
                this,
                std::placeholders::_1,
                std::placeholders::_2));

  camera_info_sub_ = nh.create_subscription<sensor_msgs::msg::CameraInfo>(
      "rgb/camera_info",
      rclcpp::SensorDataQoS().keep_last(config.queue_size),
      std::bind(&ImageReceiver::callbackCameraInfo, this, std::placeholders::_1));
  color_sub_.subscribe(node, nh.resolve_name("rgb/image_raw", false), rmw_qos_profile_sensor_data);
  depth_sub_.subscribe(node, nh.resolve_name("depth_registered/image_rect", false), rmw_qos_profile_sensor_data);
  panoptic_sub_.subscribe(node, nh.resolve_name("panoptic/image_raw", false), rmw_qos_profile_sensor_data);
  feature_sub_.subscribe(node, nh.resolve_name("image_feature", false), rmw_qos_profile_sensor_data);
  label_sub_.subscribe(node, nh.resolve_name("semantic", false), rmw_qos_profile_sensor_data);
  relations_sub_.subscribe(node, nh.resolve_name("relations", false), rmw_qos_profile_sensor_data);
  synchronizer_.reset(new Synchronizer(SyncPolicy(config.queue_size),
                                       color_sub_,
                                       depth_sub_,
                                       panoptic_sub_,
                                       feature_sub_,
                                       label_sub_,
                                       relations_sub_));
  synchronizer_->registerCallback(std::bind(&ImageReceiver::callback,
                                             this,
                                             std::placeholders::_1,
                                             std::placeholders::_2,
                                             std::placeholders::_3,
                                             std::placeholders::_4,
                                             std::placeholders::_5,
                                             std::placeholders::_6));

  return true;
}

ImageReceiver::~ImageReceiver() {}

void ImageReceiver::toggleProcessingService(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
  processing_enabled_ = !processing_enabled_;
  res->success = true;
  res->message = processing_enabled_ ? "Processing enabled" : "Processing disabled";
  LOG(INFO) << "[image_receiver] " << res->message;
}

std::string showImageDim(const sensor_msgs::msg::Image::ConstSharedPtr& image) {
  std::stringstream ss;
  ss << "[" << image->width << ", " << image->height << "]";
  return ss.str();
}

void ImageReceiver::callbackCameraInfo(
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr& msg) {
  if (msg->distortion_model == "none") {
    LOG(WARNING) << "Camera info has no distortion model. Skipping initialization.";
    maps_.init = true;  // No distortion, so we can consider it initialized.
    camera_info_sub_.reset();
    return;
  }
  cv::Mat camera_matrix(3, 3, CV_64F, const_cast<double*>(msg->k.data()));
  cv::Mat dist_coeffs = cv::Mat(msg->d).clone();
  cv::Size image_size(msg->width, msg->height);
  cv::initUndistortRectifyMap(camera_matrix,
                              dist_coeffs,
                              cv::Mat(),
                              camera_matrix,  // or newCameraMatrix
                              image_size,
                              CV_16SC2,
                              maps_.map1,
                              maps_.map2);
  maps_.init = true;
  LOG(INFO) << "[image_receiver] Initialized camera rectification maps";
  camera_info_sub_.reset();
}

void ImageReceiver::processLabels(
    const cv::Mat& panoptic_ids,
    const semantic_inference_msgs::msg::FeatureImage::ConstSharedPtr& labels,
    ImageInputPacket::Ptr& packet,
    const semantic_inference_msgs::msg::FeatureVectorsStamped::ConstSharedPtr& relations) const {
  // Initialize the features mask
  packet->features_mask = cv::Mat::zeros(panoptic_ids.size(), CV_16UC1);
  panoptic_ids.convertTo(packet->features_mask.value(), CV_16UC1);

  // Reserve space for the semantic_features map
  packet->semantic_features = std::unordered_map<uint16_t, Eigen::VectorXf>();
  packet->semantic_features.value().reserve(labels->mask_ids.size());
  for (size_t i = 0; i < labels->mask_ids.size(); ++i) {
    const auto& mask_id = labels->mask_ids[i];
    const auto& feature_data = labels->features[i].data;
    packet->semantic_features.value()[mask_id] =
        Eigen::Map<const Eigen::VectorXf>(feature_data.data(), feature_data.size());
  }

  if (!relations->feature.empty()) {
    packet->relations = PairHashMap();
  }

  for (size_t i = 0; i < static_cast<size_t>(relations->choice.size() / 2); ++i) {
    const auto& relation =
        relations->feature.size() > 1 ? relations->feature[i] : relations->feature[0];
    try {
      packet->relations.value()[std::make_pair(
          static_cast<uint16_t>(relations->choice[2 * i]),
          static_cast<uint16_t>(relations->choice[2 * i + 1]))] =
          Eigen::Map<const Eigen::MatrixXf>(
              relation.data.data(), relation.rows, relation.cols);
    } catch (const std::exception& e) {
      LOG(ERROR) << "unable to read relations from ros: " << e.what();
    }
  }
}

void ImageReceiver::callback(
    const sensor_msgs::msg::Image::ConstSharedPtr& color,
    const sensor_msgs::msg::Image::ConstSharedPtr& depth,
    const sensor_msgs::msg::Image::ConstSharedPtr& panoptic_ids,
    const semantic_inference_msgs::msg::FeatureVectorStamped::ConstSharedPtr& features,
    const semantic_inference_msgs::msg::FeatureImage::ConstSharedPtr& labels,
    const semantic_inference_msgs::msg::FeatureVectorsStamped::ConstSharedPtr& relations) {
  if (!maps_.init) {
    LOG(ERROR) << "Camera maps are not initialized. Cannot process images.";
    return;
  }
  if (!processing_enabled_) {
    auto nh = ianvs::NodeHandle::this_node();
    RCLCPP_WARN_THROTTLE(nh.logger(),
                         *nh.clock(),
                         5000,
                         "Image processing paused via service toggle");
    return;
  }

  if (color && (color->width != depth->width || color->height != depth->height)) {
    LOG(ERROR) << "color dimensions do not match depth dimensions: "
               << showImageDim(color) << " != " << showImageDim(depth);
    return;
  }
  const auto labels_image =
      std::make_shared<sensor_msgs::msg::Image>(labels->image);
  if (labels_image &&
      (labels_image->width != depth->width || labels_image->height != depth->height)) {
    LOG(ERROR) << "label dimensions do not match depth dimensions: "
               << showImageDim(labels_image) << " != " << showImageDim(depth);
    return;
  }

  if (!checkInputTimestamp(rclcpp::Time(depth->header.stamp).nanoseconds())) {
    return;
  }

  auto packet =
      std::make_shared<ImageInputPacket>(
          rclcpp::Time(color->header.stamp).nanoseconds(), sensor_id_);
  try {
    const auto cv_depth = cv_bridge::toCvShare(depth);
    packet->depth = cv_depth->image.clone();
    // Convert depth to meters if it is in millimeters.
    if (cv_depth->image.type() == CV_16UC1) {
      packet->depth.convertTo(packet->depth, CV_32FC1, 0.001);
    } else if (cv_depth->image.type() != CV_32FC1) {
      LOG(ERROR) << "Unsupported depth image type: " << cv_depth->image.type();
      return;
    }
    if (color && color->encoding == sensor_msgs::image_encodings::RGB8) {
      auto cv_color = cv_bridge::toCvShare(color);
      packet->color = cv_color->image.clone();
    } else if (color) {
      auto cv_color = cv_bridge::toCvCopy(color, sensor_msgs::image_encodings::RGB8);
      packet->color = cv_color->image;
    }
    if (!maps_.map1.empty() && !maps_.map2.empty()) {
      // Apply the camera maps for undistortion and rectification.
      cv::remap(packet->depth, packet->depth, maps_.map1, maps_.map2, cv::INTER_LINEAR);
      cv::remap(packet->color, packet->color, maps_.map1, maps_.map2, cv::INTER_LINEAR);
    }

    if (labels_image) {
      auto cv_labels = cv_bridge::toCvShare(labels_image);
      packet->labels = cv_labels->image.clone();
    }
  } catch (const cv_bridge::Exception& e) {
    LOG(ERROR) << "unable to read images from ros: " << e.what();
  }

  cv::Mat label_image;
  if (!hydra::conversions::colorToLabels(label_image, packet->labels)) {
    return;
  }
  if (!labels->features.empty() || !relations->feature.empty()) {
    auto cv_panoptic = cv_bridge::toCvShare(panoptic_ids);
    processLabels(cv_panoptic->image, labels, packet, relations);
  }

  if (!features->feature.data.empty()) {
    // Map vector of floats to Eigen vector.
    packet->image_feature = Eigen::Map<const Eigen::VectorXf>(
        features->feature.data.data(), features->feature.data.size());
  }
  queue.push(packet);
}
}  // namespace hydra
