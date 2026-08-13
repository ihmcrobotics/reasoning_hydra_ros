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
#pragma once

#include <config_utilities/factory.h>
#include <hydra/input/data_receiver.h>
#include <hydra/input/input_conversion.h>
#include <image_transport/image_transport.hpp>
#include <image_transport/subscriber_filter.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <rclcpp/rclcpp.hpp>
#include <semantic_inference_msgs/msg/feature_image.hpp>
#include <semantic_inference_msgs/msg/feature_vector_stamped.hpp>
#include <semantic_inference_msgs/msg/feature_vectors_stamped.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <memory>
#include <opencv2/opencv.hpp>
#include <string>
#include <unordered_map>

namespace hydra {

class ImageReceiver : public DataReceiver {
 public:
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<
      sensor_msgs::msg::Image,
      sensor_msgs::msg::Image,
      sensor_msgs::msg::Image,
      semantic_inference_msgs::msg::FeatureVectorStamped,
      semantic_inference_msgs::msg::FeatureImage,
      semantic_inference_msgs::msg::FeatureVectorsStamped>;
  using Synchronizer = message_filters::Synchronizer<SyncPolicy>;

  struct Config : DataReceiver::Config {
    std::string ns = "~";
    size_t queue_size = 100;
  };

  ImageReceiver(const Config& config, size_t sensor_id);

  virtual ~ImageReceiver();

 public:
  const Config config;

 protected:
  bool initImpl() override;

 private:
  void toggleProcessingService(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
      std::shared_ptr<std_srvs::srv::Trigger::Response> res);
  void callback(
      const sensor_msgs::msg::Image::ConstSharedPtr& color,
      const sensor_msgs::msg::Image::ConstSharedPtr& depth,
      const sensor_msgs::msg::Image::ConstSharedPtr& panoptic_ids,
      const semantic_inference_msgs::msg::FeatureVectorStamped::ConstSharedPtr& features,
      const semantic_inference_msgs::msg::FeatureImage::ConstSharedPtr& labels,
      const semantic_inference_msgs::msg::FeatureVectorsStamped::ConstSharedPtr& relations);
  void callbackCameraInfo(
      const sensor_msgs::msg::CameraInfo::ConstSharedPtr& msg);
  void processLabels(
      const cv::Mat& panoptic_ids,
      const semantic_inference_msgs::msg::FeatureImage::ConstSharedPtr& labels,
      ImageInputPacket::Ptr& packet,
      const semantic_inference_msgs::msg::FeatureVectorsStamped::ConstSharedPtr& relations) const;

  message_filters::Subscriber<sensor_msgs::msg::Image> color_sub_;
  message_filters::Subscriber<sensor_msgs::msg::Image> depth_sub_;
  message_filters::Subscriber<sensor_msgs::msg::Image> panoptic_sub_;
  message_filters::Subscriber<semantic_inference_msgs::msg::FeatureVectorStamped>
      feature_sub_;
  message_filters::Subscriber<semantic_inference_msgs::msg::FeatureImage> label_sub_;
  message_filters::Subscriber<semantic_inference_msgs::msg::FeatureVectorsStamped>
      relations_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  std::unique_ptr<Synchronizer> synchronizer_;
  std::atomic<bool> processing_enabled_{true};
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr enable_processing_service_;

  inline static const auto registration_ =
      config::RegistrationWithConfig<DataReceiver,
                                     ImageReceiver,
                                     ImageReceiver::Config,
                                     size_t>("ImageReceiver");
  struct Maps {
    cv::Mat map1;
    cv::Mat map2;
    bool init = false;
  } maps_;
};

void declare_config(ImageReceiver::Config& config);

}  // namespace hydra
