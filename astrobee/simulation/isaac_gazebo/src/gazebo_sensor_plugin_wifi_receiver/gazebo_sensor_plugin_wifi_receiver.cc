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

// Tf2 includes
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>

// FSW includes
#include <config_reader/config_reader.h>
#include <isaac_util/isaac_names.h>

// Wifi Sensor message
#include <isaac_hw_msgs/WifiSignal.h>
#include <isaac_hw_msgs/WifiSignals.h>

// STL includes
#include <string>

namespace gazebo {

class GazeboSensorPluginWifiReceiver : public FreeFlyerSensorPlugin {
 public:
  GazeboSensorPluginWifiReceiver() :
    FreeFlyerSensorPlugin("wifi_receiver", "", false) {
  }

  ~GazeboSensorPluginWifiReceiver() {}

 protected:
  // Called when plugin is loaded into gazebo
  void LoadCallback(ros::NodeHandle* nh,
    sensors::SensorPtr sensor, sdf::ElementPtr sdf) {
    // Get a link to the parent sensor
    sensor_ = std::dynamic_pointer_cast<sensors::WirelessReceiver>(sensor);
    if (!sensor_) {
      gzerr << "GazeboSensorPluginWifi requires a valid sensor.\n";
      return;
    }

    // Read the sdf
    if (sdf->HasElement("connected_essid"))
      connected_essid_ = sdf->Get<std::string>("connected_essid");

    // Read the config file
    config_reader::ConfigReader config_params;
    // Set the config path to ISAAC
    char *path;
    if ((path = getenv("ISAAC_CONFIG_DIR")) == NULL) {
      NODELET_FATAL("Could not find config path");
      return;
    }

    config_params.SetPath(path);

    config_params.AddFile("hw/wifi.config");
    if (!config_params.ReadFiles()) NODELET_FATAL("Failed to read config files.");

    // Get time in between measurements for Scan Station
    if (!config_params.GetReal("time_scan_station", &time_scan_station_))
      NODELET_FATAL("Could not find row 'time_data' in table");

    // Get time in between measurements for Scan All
    if (!config_params.GetReal("time_scan_all", &time_scan_all_))
      NODELET_FATAL("Could not find row 'time_scan_all' in table");

    if (!config_params.GetInt("max_networks", &max_networks_))
      NODELET_FATAL("Could not find row 'max_networks' in table");

    // Publish static transform
    geometry_msgs::TransformStamped msg;
    static tf2_ros::StaticTransformBroadcaster br;
    msg.header.frame_id = "body";
    msg.child_frame_id = sensor_->Name();
    msg.header.stamp = ros::Time::now();
    msg.transform.translation.x = sensor_->Pose().Pos().X();
    msg.transform.translation.y = sensor_->Pose().Pos().Y();
    msg.transform.translation.z = sensor_->Pose().Pos().Z();
    msg.transform.rotation.x = sensor_->Pose().Rot().X();
    msg.transform.rotation.y = sensor_->Pose().Rot().Y();
    msg.transform.rotation.z = sensor_->Pose().Rot().Z();
    msg.transform.rotation.w = sensor_->Pose().Rot().W();
    br.sendTransform(msg);

    // Get node handle.
    nh_gz_ = transport::NodePtr(new transport::Node());
    // Initialise with default namespace (typically /gazebo/default/).
    nh_gz_->Init();

    // Connect to the sensor update event.
    update_ = nh_gz_->Subscribe(sensor_->Topic(),
                    &GazeboSensorPluginWifiReceiver::UpdateCallback, this);

    std::string scan_type = "";
    if (sdf->HasElement("scan_type"))
      scan_type = sdf->Get<std::string>("scan_type");
    else
      return;

    // Advertise Wifi messages
    pub_ = nh->advertise<isaac_hw_msgs::WifiSignals>(
                          TOPIC_HARDWARE_WIFI, 1);

    if (scan_type == "station") {
      // Create Timer to scope station wireless signals
      timer_ = nh->createTimer(ros::Duration(time_scan_station_),
                    &GazeboSensorPluginWifiReceiver::ScanStationCallback, this);
    } else if (scan_type == "all") {
      // Create Timer to scope all wireless signals
      timer_ = nh->createTimer(ros::Duration(time_scan_all_),
                    &GazeboSensorPluginWifiReceiver::ScanAllCallback, this);
    } else {
      return;
    }
    ROS_ERROR("Start WiFi receiver");
  }

  // Called on each sensor update event
  void UpdateCallback(ConstWirelessNodesPtr& msg) {
    mutex_msg_.lock();
    msg_ = *msg;
    mutex_msg_.unlock();
  }

  void ScanStationCallback(ros::TimerEvent const& event) {
    mutex_msg_.lock();
    msgs::WirelessNodes msg = msg_;
    mutex_msg_.unlock();
    isaac_hw_msgs::WifiSignals msg_station;
    msg_station.signals.resize(1);
    for (int i = 0; i < msg.node_size(); ++i) {
      if (msg.node(i).essid() == connected_essid_) {
        msg_station.signals[0].header.stamp = ros::Time::now();
        msg_station.signals[0].header.frame_id = sensor_->Name();
        msg_station.signals[0].ssid = msg.node(i).essid();
        msg_station.signals[0].signal_dbm = msg.node(i).signal_level();
        // Publish station message
        pub_.publish(msg_station);
        break;
      }
    }
  }

  void ScanAllCallback(ros::TimerEvent const& event) {
    mutex_msg_.lock();
    msgs::WirelessNodes msg = msg_;
    mutex_msg_.unlock();

    isaac_hw_msgs::WifiSignals msg_all;
    msg_all.signals.resize(msg.node_size());
    for (int i = 0; i < msg.node_size(); ++i) {
      msg_all.signals[i].header.stamp = ros::Time::now();
      msg_all.signals[i].header.frame_id = sensor_->Name();
      msg_all.signals[i].ssid = msg.node(i).essid();
      msg_all.signals[i].signal_dbm = msg.node(i).signal_level();
    }
    // Publish all message
    pub_.publish(msg_all);
  }

 private:
  // ROS variables
  ros::Publisher pub_;                            // Wifi state publisher
  ros::Timer timer_;                              // Timer callback to publish
  std::mutex mutex_msg_;                          // Protect wifi message
  // Gazebo variables
  gazebo::transport::NodePtr nh_gz_;
  sensors::WirelessReceiverPtr sensor_;
  gazebo::transport::SubscriberPtr update_;

  msgs::WirelessNodes msg_;
  std::string connected_essid_ = "";
  double time_scan_station_, time_scan_all_;      // Time in between scans
  int max_networks_;                              // Maximum amounts of APs (Access Points)
};

GZ_REGISTER_SENSOR_PLUGIN(GazeboSensorPluginWifiReceiver)

}   // namespace gazebo
