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
#include <ros/ros.h>

// FSW shared libraries
#include <config_reader/config_reader.h>
#include <ff_util/ff_nodelet.h>
#include <isaac_util/isaac_names.h>

// Hardware messages
#include <isaac_hw_msgs/WifiSignal.h>
#include <isaac_hw_msgs/WifiSignals.h>

// Interface class
#include <wifi/wifi.h>

#define CONVERT_S_TO_MS 0.001

/**
 * \ingroup hw
 */
namespace wifi {
class WifiTool {
 public:
  explicit WifiTool(ros::NodeHandle *nh) : nh_(nh) {
    Initialize();
  }
  ~WifiTool() {
    // Free the library resources
    wifi_.WifiScanClose();
  }

 protected:
  // Called on flight software stack initialization - every NODELET_FATAIL
  // call below should be converted to an initialization fault...
  void Initialize() {
    ROS_ERROR("Running wifi driver node");
    // Read the configuration
    config_reader::ConfigReader config_params;
    // Set the config path to ISAAC
    char *path = getenv("CUSTOM_CONFIG_DIR");
    if (path != NULL)
      config_params.SetPath(path);
    config_params.AddFile("hw/wifi.config");
    if (!config_params.ReadFiles()) ROS_FATAL("Couldn't read config file");

    // Read the device information from the config table
    config_reader::ConfigReader::Table devices;

    // Get the name of the device
    std::string name;
    if (!config_params.GetStr("interface_name", &name))
      ROS_FATAL("Could not find row 'interface_name' in table");

    // Get time in between measurements for Scan All
    if (!config_params.GetReal("time_scan_all", &time_scan_))
      ROS_FATAL("Could not find row 'time_scan_all' in table");

    if (!config_params.GetInt("max_networks", &BSS_INFOS_))
      ROS_FATAL("Could not find row 'max_networks' in table");
    bss_ = reinterpret_cast<struct bss_info*>(malloc(BSS_INFOS_ * sizeof(struct bss_info)));

    if ((wifi_.WifiScanInit((const char *) name.c_str())) == -1)
      ROS_FATAL("Could not Initiate the Scan");

    // Create a wifi state publisher
    pub_ = nh_->advertise<isaac_hw_msgs::WifiSignals>(
                            TOPIC_HARDWARE_WIFI, 1, true);

    // Create Timer to scope all wireless signals
    timer_ = nh_->createTimer(ros::Duration(time_scan_),
                             &WifiTool::ScanAllCallback, this);
  }

  // CALLBACK FUNCTIONS

  // Asynchronous callback from the serial driver with feedback/result,
  // which we then push to the ROS messagign backbone as a joint state
  void ScanAllCallback(ros::TimerEvent const& event) {
    int status = wifi_.WifiScanAll(bss_, BSS_INFOS_);

    if (status == 0) {
      ROS_ERROR("No associated station\n");
    } else if (status == -1) {
      ROS_ERROR("Unable to get station information\n");
    } else {
      isaac_hw_msgs::WifiSignal signal;
      isaac_hw_msgs::WifiSignals msg;
      for (int i = 0; i < status && i < BSS_INFOS_; ++i) {
        signal.header.stamp = ros::Time::now() - ros::Duration(bss_[i].seen_ms_ago * CONVERT_S_TO_MS);
        signal.header.frame_id = "wifi_receiver";
        signal.bssid = wifi_.bssid_to_string(bss_[i].bssid);
        signal.ssid = bss_[i].ssid;
        signal.signal_dbm = bss_[i].signal_mbm/100;
        msg.signals.push_back(signal);
      }
      pub_.publish(msg);
    }
  }

 private:
  // ROS variables
  ros::NodeHandle *nh_;
  ros::Publisher pub_;                  // Wifi state publisher
  double time_scan_;                    // Time in between scans
  ros::Timer timer_;                    // Wifi scan timer

  // Wifi Scan All
  Wifi wifi_;                           // Wifi interface library
  int BSS_INFOS_;                       // Maximum amounts of APs (Access Points)
  struct bss_info *bss_;                // Informatoin about APs (Access Points)
  int status_;                          // Return status of wifi scan
};
}  // namespace wifi

// Main entry point for application
int main(int argc, char *argv[]) {
  // Initialize a ros node
  ros::init(argc, argv, "wifi_node", ros::init_options::AnonymousName);
  ros::NodeHandle nh;
  wifi::WifiTool wifi(&nh);
  // Synchronous mode
  ros::spin();
  // Make for great success
  return 0;
}
