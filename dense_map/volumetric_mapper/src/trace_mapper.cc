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

// Volumetric Mapper dependencies
#include <volumetric_mapper/volumetric_mapper.h>

namespace volumetric_mapper {

  TraceMapper::TraceMapper(ros::NodeHandle* nh, std::string topic, double resolution,
              double min_intensity, double max_intensity, double transparency) {
    // Define the map printing resolution
    resolution_ = resolution;
    min_intensity_ = min_intensity;
    max_intensity_ = max_intensity;
    transparency_ = transparency;

    // Publisher for the robot trace map
    pub_map_trace_ = nh->advertise<visualization_msgs::MarkerArray>(
                      topic, 1, true);
  }

  TraceMapper::~TraceMapper() {}

  void TraceMapper::AddTraceData(double value, geometry_msgs::TransformStamped tf) {
    // Set position within the resolution
    static geometry_msgs::Point point_center;
    point_center.x = roundPartial(tf.transform.translation.x, resolution_);
    point_center.y = roundPartial(tf.transform.translation.y, resolution_);
    point_center.z = roundPartial(tf.transform.translation.z, resolution_);

    // Check if the position has been scoped
    for (int i = 0; i < map_trace_.markers.size(); ++i) {
      if (map_trace_.markers[i].points[0].x == point_center.x &&
          map_trace_.markers[i].points[0].y == point_center.y &&
          map_trace_.markers[i].points[0].z == point_center.z) {
        // Average out all the measurements
        // The number of measurements for a given point is hidden in header.seq
        // The unconverted to rgb intensity is hidden in color.r
        const double tot_intensity = (map_trace_.markers[i].color.r * map_trace_.markers[i].header.seq
                                   + value)
                                   / (++map_trace_.markers[i].header.seq);
        map_trace_.markers[i].color.r = tot_intensity;

        // Define Marker Color
        const double h = (1.0 - std::min(std::max((tot_intensity - min_intensity_)/
                                    (max_intensity_ - min_intensity_), 0.0), 1.0));
        map_trace_.markers[i].colors[0] = intensityMapColor(h, transparency_);
        return;
      }
    }
    // In case this is a new position
    // Initialize new marker message
    visualization_msgs::Marker marker;

    // Fill in marker properties
    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time::now();
    marker.ns = "";
    marker.id = map_trace_.markers.size();
    marker.type = visualization_msgs::Marker::CUBE_LIST;
    marker.scale.x = resolution_;
    marker.scale.y = resolution_;
    marker.scale.z = resolution_;
    marker.pose.orientation.w = 1.0;
    marker.action = visualization_msgs::Marker::ADD;

    // Fill in center position
    marker.points.push_back(point_center);

    // Set up the initial hidden information
    marker.color.r = value;
    marker.header.seq = 1;

    // Define Marker Color
    const double h = (1.0 - std::min(std::max((value - min_intensity_)/
                                (max_intensity_ - min_intensity_), 0.0), 1.0));
    marker.colors.push_back(intensityMapColor(h, transparency_));

    // Add to the marker message
    map_trace_.markers.push_back(marker);
  }

  void TraceMapper::PubTraceData() {
    // Publish
    pub_map_trace_.publish(map_trace_);
  }
}  // namespace volumetric_mapper
