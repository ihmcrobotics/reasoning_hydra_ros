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
#include <glog/logging.h>
#include <hydra/input/sensor_input_packet.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>

#include <functional>
#include <ostream>

namespace sensor_msgs::msg {

inline std::ostream& operator<<(std::ostream& out, const PointField& field) {
  return out << "PointField(name='" << field.name << "', offset=" << field.offset
             << ", datatype=" << static_cast<int>(field.datatype) << ")";
}

}  // namespace sensor_msgs::msg

namespace hydra {

class PointcloudAdaptor {
 public:
  explicit PointcloudAdaptor(const sensor_msgs::msg::PointCloud2& cloud);

  bool valid() const;

  bool hasLabels() const;

  cv::Vec3f position(const uint8_t* point_ptr) const;

  cv::Vec3b color(const uint8_t* point_ptr) const;

  uint32_t label(const uint8_t* point_ptr) const;

 protected:
  std::function<double(const uint8_t*)> x_parser_;
  std::function<double(const uint8_t*)> y_parser_;
  std::function<double(const uint8_t*)> z_parser_;
  std::function<uint32_t(const uint8_t*)> label_parser_;
  std::function<cv::Vec3b(const uint8_t*)> color_parser_;
};

std::function<double(const uint8_t*)> initFloatParser(
    const sensor_msgs::msg::PointField& field);

std::function<double(const uint8_t*)> initIntParser(
    const sensor_msgs::msg::PointField& field);

std::function<cv::Vec3b(const uint8_t*)> initColorParser(
    const sensor_msgs::msg::PointField& field);

bool fillPointcloudPacket(const sensor_msgs::msg::PointCloud2& msg,
                          CloudInputPacket& packet,
                          bool labels_required);

}  // namespace hydra
