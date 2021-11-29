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

// Standard ROS includes
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

// FSW shared libraries
#include <config_reader/config_reader.h>
#include <ff_util/ff_nodelet.h>
#include <isaac_util/isaac_names.h>

// Hardware messages
#include <isaac_hw_msgs/WifiSignals.h>

// Interface class
#include <wifi/wifi.h>

/**
 * \ingroup hw
 */
namespace wifi {

class WifiNode : public ff_util::FreeFlyerNodelet {
 public:
  WifiNode() : ff_util::FreeFlyerNodelet(NODE_WIFI) {}
  ~WifiNode() {
    // Free the library resources
    // wifi_.WifiScanClose();
  }

 protected:
  // Called on flight software stack initialization - every NODELET_FATAIL
  // call below should be converted to an initialization fault...
  void Initialize(ros::NodeHandle *nh) {
    // Read the configuration
    config_reader::ConfigReader config_params;
    // Set the config path to ISAAC
    char *path = getenv("ISAAC_CONFIG_DIR");
    if (path != NULL)
      config_params.SetPath(path);
    config_params.AddFile("hw/wifi.config");
    if (!config_params.ReadFiles()) NODELET_FATAL("Couldn't read config file");

    // Read the device information from the config table
    config_reader::ConfigReader::Table devices;

    // Get the name of the device
    std::string name;
    if (!config_params.GetStr("interface_name", &name))
      NODELET_FATAL("Could not find row 'interface_name' in table");

    // Get time in between measurements for Scan Station
    if (!config_params.GetReal("time_scan_station", &time_scan_))
      NODELET_FATAL("Could not find row 'time_data' in table");

    if ((wifi_.WifiScanInit((const char *) name.c_str())) == -1)
      NODELET_FATAL("Could not Initiate the Scan");

    // Create a wifi state publisher
    pub_ = nh->advertise<isaac_hw_msgs::WifiSignals>(TOPIC_HARDWARE_WIFI, 1, true);


    // Create Timer to scope station wireless signals
    timer_ = nh->createTimer(ros::Duration(time_scan_),
                             &WifiNode::ScanStationCallback, this);
  }

  // CALLBACK FUNCTIONS

  // Asynchronous callback from the serial driver with feedback/result,
  // which we then push to the ROS messagign backbone as a joint state
  void ScanStationCallback(ros::TimerEvent const& event) {
    int status = wifi_.WifiScanStation(&station_);

    if (status == 0) {
      ROS_ERROR("No associated station\n");
    } else if (status == -1) {
      ROS_ERROR("Unable to get station information\n");
    } else {
      isaac_hw_msgs::WifiSignals msg;
      msg.signals.resize(1);
      msg.signals[0].header.stamp = ros::Time::now();
      msg.signals[0].header.frame_id = "wifi_receiver";
      msg.signals[0].bssid = wifi_.bssid_to_string(station_.bssid);
      msg.signals[0].ssid = station_.ssid;
      msg.signals[0].signal_dbm = station_.signal_dbm;
      pub_.publish(msg);
    }
  }

 private:
  Wifi wifi_;                       // Wifi interface library
  // Scan Station
  struct station_info station_;     // Information about AP (Access Point) we are connected to
  // Scan All
  static const int BSS_INFOS = 10;  // Maximum amounts of APs (Access Points)
  struct bss_info bss_[BSS_INFOS];  // Informatoin about APs (Access Points)

  // a placeholder where we convert BSSID to printable hardware mac address
  char   mac_[BSSID_STRING_LENGTH];
  int    status_;
  ros::Publisher pub_;           // Wifi state publisher
  double time_scan_;
  ros::Timer timer_;
  std::string prefix_;           // Joint prefix
};

PLUGINLIB_EXPORT_CLASS(wifi::WifiNode, nodelet::Nodelet);

}  // namespace wifi
