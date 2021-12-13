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

// ROS includes
#include <ros/ros.h>

// Wifi Sensor library
#include <gazebo/sensors/WirelessReceiver.hh>
#include <gazebo/sensors/WirelessTransmitter.hh>

// Sensor plugin interface
#include <astrobee_gazebo/astrobee_gazebo.h>

// FSW includes
#include <isaac_util/isaac_names.h>

// Wifi Sensor message
#include <visualization_msgs/MarkerArray.h>

// STL includes
#include <string>

#define MAX_SIGNAL_STRENGTH 0
#define MIN_SIGNAL_STRENGTH -90

namespace gazebo {

class GazeboSensorPluginWifiTransmitter : public FreeFlyerSensorPlugin {
 public:
  GazeboSensorPluginWifiTransmitter() :
    FreeFlyerSensorPlugin("wifi_transmitter", "", true) {}

  ~GazeboSensorPluginWifiTransmitter() {}

 protected:
  // Called when plugin is loaded into gazebo
  void LoadCallback(ros::NodeHandle* nh,
    sensors::SensorPtr sensor, sdf::ElementPtr sdf) {
    // Get a link to the sensor
    sensor_ = std::dynamic_pointer_cast<sensors::WirelessTransmitter>(sensor);
    if (!sensor_) {
      gzerr << "GazeboSensorPluginWifi requires a valid sensor.\n";
      return;
    }

    // Offer wifi transmitter propagation for visualization
    pub_visual_ = nh->advertise<visualization_msgs::MarkerArray>(
                                            TOPIC_WIFI_TRANSMITTER_MAP, 1);
    // Get node handle
    nh_gz_ = transport::NodePtr(new transport::Node());
    // Initialise with default namespace (typically /gazebo/default/).
    nh_gz_->Init();
    // Connect to the sensor update event.
    update_ = nh_gz_->Subscribe(sensor_->Topic(),
                    &GazeboSensorPluginWifiTransmitter::UpdateCallback, this);
  }

  // Turn wifi on or off based on topic subscription
  void ToggleCallback() {
  }

  // Called on each sensor update event
  void UpdateCallback(ConstPropagationGridPtr& msg) {
    // Initialize marker message
    visualization_msgs::MarkerArray msg_visual;
    msg_visual.markers.resize(msg->particle_size());

    for (int i = 0; i < msg->particle_size(); ++i) {
      // Fill in marker properties
      msg_visual.markers[i].header.frame_id = sensor_->ESSID() + std::string("/body");
      msg_visual.markers[i].header.stamp = ros::Time::now();
      msg_visual.markers[i].ns = "wifiMap";
      msg_visual.markers[i].id = i;
      msg_visual.markers[i].type = visualization_msgs::Marker::CUBE_LIST;
      msg_visual.markers[i].scale.x = 1;
      msg_visual.markers[i].scale.y = 1;
      msg_visual.markers[i].scale.z = 0.1;
      msg_visual.markers[i].pose.orientation.w = 1.0;

      // Filter weak signal zones
      if (msg->particle(i).signal_level() < MIN_SIGNAL_STRENGTH) {
        msg_visual.markers[i].action = visualization_msgs::Marker::DELETE;
        continue;
      }
      msg_visual.markers[i].action = visualization_msgs::Marker::ADD;

      // Set position
      static geometry_msgs::Point point_center;
      point_center.x = msg->particle(i).x();
      point_center.y = msg->particle(i).y();
      point_center.z = 0;
      msg_visual.markers[i].points.push_back(point_center);

      // Set Colour
      const double h = (1.0 - std::min(std::max((msg->particle(i).signal_level()-MIN_SIGNAL_STRENGTH)/
                                  (MAX_SIGNAL_STRENGTH - MIN_SIGNAL_STRENGTH), 0.0), 1.0));
      msg_visual.markers[i].colors.push_back(HeightMapColor(h, 1.0));
    }
    // Publish message
    pub_visual_.publish(msg_visual);
  }

  // extracted from https://github.com/OctoMap/octomap_mapping
  std_msgs::ColorRGBA HeightMapColor(const double &height,
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

 private:
  // ROS variables
  ros::Publisher pub_visual_;
  // Gazebo variables
  gazebo::transport::NodePtr nh_gz_;
  sensors::WirelessTransmitterPtr sensor_;
  physics::EntityPtr parent_;
  gazebo::transport::SubscriberPtr update_;
  int id_ = 0;
};

GZ_REGISTER_SENSOR_PLUGIN(GazeboSensorPluginWifiTransmitter)

}   // namespace gazebo
