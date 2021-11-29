/* Copyright (c) 2021, United States Government, as represented by the
 * Administrator of the National Aeronautics and Space Administration.
 *
 * All rights reserved.
 *
 * The "ISAAC - Integrated System for Autonomous and Adaptive Caretaking
 * platform" software is licensed under the Apache License, Version 2.0
 * (the "License"); you may not use this file except in compliance with the
 * License. You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations
 * under the License.
 */

#ifndef VOLUMETRIC_MAPPER_VOLUMETRIC_MAPPER_H_
#define VOLUMETRIC_MAPPER_VOLUMETRIC_MAPPER_H_

// Standard ROS includes
#include <ros/ros.h>

// TF2 support
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// Shared project includes
#include <volumetric_mapper/volumetric_mapper.h>

// Messages
#include <visualization_msgs/MarkerArray.h>

// Volumetric Mapper dependencies
#include <gp/gp.h>
#include <gp/gp_utils.h>
#include <gp/rprop.h>

// C++ headers
#include <string>
#include <mutex>


namespace volumetric_mapper {
// Volumetric Mapper Class

class VolumetricMapper{
 public:
  // Constructor
  VolumetricMapper(ros::NodeHandle* nh, std::string topic, double resolution,
                  double min_intensity, double max_intensity, double transparency,
                  double offset);
  ~VolumetricMapper();

  // Add new point to the mapping iterpolation
  void AddMapData(double value, geometry_msgs::TransformStamped tf);
  // Publishes the map into a marker vector for visualization
  void PubMapData(double x_min, double x_max, double y_min, double y_max,
                                              double z_min, double z_max);

 private:
  // initialize Gaussian process for 3-D input using the squared exponential
  // covariance function with additive white noise
  libgp::GaussianProcess * gp_;
  // libgp::GaussianProcess gp_ = libgp::GaussianProcess(3, "CovSum ( CovSEiso, CovNoise)");
  libgp::RProp rprop_;
  ros::Publisher publisher_;
  visualization_msgs::MarkerArray wifi_trace_;
  std::mutex * mtx_;
  // Parameters
  double resolution_;
  double min_intensity_;
  double max_intensity_;
  double transparency_;
  double offset_;
};

// Trace Mapper Class
class TraceMapper{
 public:
  // Constructor
  TraceMapper(ros::NodeHandle* nh, std::string topic, double resolution,
              double min_intensity, double max_intensity, double transparency);

  ~TraceMapper();
  void AddTraceData(double value, geometry_msgs::TransformStamped tf);
  void PubTraceData();

 private:
  //
  ros::Publisher pub_map_trace_;
  visualization_msgs::MarkerArray map_trace_;

  // Parameters
  double resolution_;
  double min_intensity_;
  double max_intensity_;
  double transparency_;
};

// Round value to expected resolution
double roundPartial(double value, double resolution) {
  return round(value / resolution) * resolution;
}

// extracted from https://github.com/OctoMap/octomap_mapping
std_msgs::ColorRGBA intensityMapColor(const double &height,
                                              const double &alpha) {
  std_msgs::ColorRGBA color;
  color.a = alpha;
  // blend over HSV-values (more colors)
  double s = 1.0;
  double v = 1.0;

  double height_mod = height;
  height_mod -= floor(height_mod);
  height_mod *= 6;
  int i;
  double m, n, f;

  i = floor(height_mod);
  f = height_mod - i;
  if (!(i & 1))
    f = 1 - f;  // if i is even
  m = v * (1 - s);
  n = v * (1 - s * f);

  switch (i) {
  case 6:
  case 0:
    color.r = v; color.g = n; color.b = m;
    break;
  case 1:
    color.r = n; color.g = v; color.b = m;
    break;
  case 2:
    color.r = m; color.g = v; color.b = n;
    break;
  case 3:
    color.r = m; color.g = n; color.b = v;
    break;
  case 4:
    color.r = n; color.g = m; color.b = v;
    break;
  case 5:
    color.r = v; color.g = m; color.b = n;
    break;
  default:
    color.r = 1; color.g = 0.5; color.b = 0.5;
    break;
  }
  return color;
}
}  // namespace volumetric_mapper
#endif  // VOLUMETRIC_MAPPER_VOLUMETRIC_MAPPER_H_

