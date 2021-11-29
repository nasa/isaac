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


// Shared project includes
#include <volumetric_mapper/volumetric_mapper.h>

namespace volumetric_mapper {


  VolumetricMapper::VolumetricMapper(ros::NodeHandle* nh, std::string topic, double resolution,
                  double min_intensity, double max_intensity, double transparency, double offset) {
    // Initialize mutex
    mtx_ = new std::mutex();
    // Define the map printing resolution
    resolution_ = resolution;
    min_intensity_ = min_intensity;
    max_intensity_ = max_intensity;
    transparency_ = transparency;
    offset_ = offset;

    // libgp::GaussianProcess gp(3, "CovSum ( CovSEiso, CovNoise)");
    gp_ = new libgp::GaussianProcess(3, "CovSum ( CovSEiso, CovNoise)");
    // initialize hyper parameter vector
    Eigen::VectorXd params(gp_->covf().get_param_dim());
    params << 0, 0, -1.6;


    // Set parameters of covariance function
    gp_->covf().set_loghyper(params);

    // Hyperparameter optimizer
    rprop_.init();

    // Add publisher
    publisher_ = nh->advertise<visualization_msgs::Marker>(
                      topic, 1, true);
  }

  VolumetricMapper::~VolumetricMapper() {}

  // Add new point to the mapping iterpolation
  void VolumetricMapper::AddMapData(double value, geometry_msgs::TransformStamped tf) {
    // Convert from tf to vector to feed into gp regression
    double x[] = {tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z};
    // Lock this operation to make interpolation effective
    mtx_->lock();
    gp_->add_pattern(x, static_cast<double>(value) + offset_);
    mtx_->unlock();
  }

  // Publishes the map into a marker vector for visualization
  void VolumetricMapper::PubMapData(double x_min, double x_max,
            double y_min, double y_max, double z_min, double z_max) {
    // Round the dimentions to fit the map resolution
    x_min = roundPartial(x_min, resolution_);
    x_max = roundPartial(x_max, resolution_);
    y_min = roundPartial(y_min, resolution_);
    y_max = roundPartial(y_max, resolution_);
    z_min = roundPartial(z_min, resolution_);
    z_max = roundPartial(z_max, resolution_);

    // Position to interpolate map data
    double x[] = {0, 0, 0};

    // Initialize marker message
    visualization_msgs::Marker marker;

    // Fill in marker properties
    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time::now();
    marker.ns = "";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::CUBE_LIST;
    marker.scale.x = resolution_;
    marker.scale.y = resolution_;
    marker.scale.z = resolution_;
    marker.pose.orientation.w = 1.0;
    marker.action = visualization_msgs::Marker::ADD;
    // Faster if we allocate the exact necessary size from beginning instead of appending
    const int map_dim =  (std::floor((x_max-x_min)/resolution_) + 1)
                       * (std::floor((y_max-y_min)/resolution_) + 1)
                       * (std::floor((z_max-z_min)/resolution_) + 1);
    marker.colors.resize(map_dim);
    marker.points.resize(map_dim);

    // Lock such that the map does not change and computation is quicker
    // Interpolation will only run once if no more data is added
    int index = 0;
    mtx_->lock();
    // Maximize hyperparameters
    // rprop_.maximize(gp_, 100, 1);
    // ROS_ERROR_STREAM("0: " << gp_->covf().get_loghyper()(0) << " 1: " << gp_->covf().get_loghyper()(1)
    //                   << " 2: " << gp_->covf().get_loghyper()(2));
    // Assert if the optimization maes sense (enough measures)
    // if (gp_->covf().get_loghyper()(0) > 0 && gp_->covf().get_loghyper()(0) < 5 &&
    //     gp_->covf().get_loghyper()(1) > 1 && gp_->covf().get_loghyper()(1) < 5)
    //   return;

    for (double i = x_min; i < x_max; i += resolution_) {
      for (double j = y_min; j < y_max; j += resolution_) {
        for (double k = z_min; k < z_max; k += resolution_) {
          // New marker point
          marker.points[index].x = i;
          marker.points[index].y = j;
          marker.points[index].z = k;

          // Calculate intensity
          x[0] = i; x[1] = j; x[2] = k;
          const double f = gp_->f(x) - offset_;
          if (f < min_intensity_)
            continue;

          // Define Marker Color
          const double h = (1.0 - std::min(std::max((f - min_intensity_)/
                                (max_intensity_ - min_intensity_), 0.0), 1.0));
          marker.colors[index] = intensityMapColor(h, 0.02);

          // Increment counter
          index++;
    // ROS_ERROR_STREAM(topic_ << " index " << index);
        }
      }
    }
    mtx_->unlock();

    // Publish
    publisher_.publish(marker);
  }

}  // namespace volumetric_mapper
