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

// ISAAC names include
#include <isaac_util/isaac_names.h>

// Hardware messages
#include <isaac_hw_msgs/RFIDTags.h>
#include <visualization_msgs/MarkerArray.h>

// FSW includes
#include <config_reader/config_reader.h>

namespace rfid_mapper {

class RFIDMapperNode{
 public:
  RFIDMapperNode() {
  }

  ~RFIDMapperNode() {}

  void Initialize(ros::NodeHandle* nh) {
    nh_ = nh;
    // Set the config path to ISAAC
    char *path;
    if ((path = getenv("CUSTOM_CONFIG_DIR")) == NULL)
      ROS_FATAL("Could not find the config path.");
    config_params_.SetPath(path);

    config_params_.AddFile("dense_map/RFID_mapper.config");
    if (!config_params_.ReadFiles())
      ROS_FATAL("Failed to read config files.");
    // Read the parameters defined in the launch file
    ReadParams();
    if (!calc_trace_ && !calc_volumetric_) {
      ROS_DEBUG_STREAM("Nothing to calculate, return");
      return;
    }

    // Create a transform buffer to listen for position
    tf_listener_ = std::shared_ptr<tf2_ros::TransformListener>(
                      new tf2_ros::TransformListener(tf_buffer_));

    // RFID Subscriber
    sub_  = nh->subscribe(TOPIC_HARDWARE_RFID, 100,
                      &RFIDMapperNode::RFIDCallback, this);

    // RFID publisher at a lower rate
    timer_pub_ = nh->createTimer(ros::Duration(20),
                      &RFIDMapperNode::RFIDPubCallback, this);
  }

 protected:
  // Read Parameters from the launch file
  void ReadParams() {
    if (!config_params_.GetBool("calc_trace", &calc_trace_))
      ROS_FATAL("Could not read the calc_trace parameter.");
    if (!config_params_.GetBool("calc_volumetric", &calc_volumetric_))
      ROS_FATAL("Could not read the calc_volumetric parameter.");
    if (!config_params_.GetReal("resolution", &resolution_))
      ROS_FATAL("Could not read the resolution parameter.");

    if (!config_params_.GetReal("min_signal_strength", &min_signal_strength_))
      ROS_FATAL("Could not read the min_signal_strength parameter.");
    if (!config_params_.GetReal("max_signal_strength", &max_signal_strength_))
      ROS_FATAL("Could not read the max_signal_strength parameter.");
    if (!config_params_.GetReal("offset_signal_strength", &offset_signal_strength_))
      ROS_FATAL("Could not read the offset_signal_strength parameter.");

    if (!config_params_.GetReal("alpha_trace", &alpha_trace_))
      ROS_FATAL("Could not read the alpha_trace parameter.");
    if (!config_params_.GetReal("alpha_volumetric", &alpha_volumetric_))
      ROS_FATAL("Could not read the alpha_volumetric parameter.");

    if (!config_params_.GetReal("x_min", &x_min_))
      ROS_FATAL("Could not read the x_min parameter.");
    if (!config_params_.GetReal("x_max", &x_max_))
      ROS_FATAL("Could not read the x_max parameter.");
    if (!config_params_.GetReal("y_min", &y_min_))
      ROS_FATAL("Could not read the y_min parameter.");
    if (!config_params_.GetReal("y_max", &y_max_))
      ROS_FATAL("Could not read the y_max parameter.");
    if (!config_params_.GetReal("z_min", &z_min_))
      ROS_FATAL("Could not read the z_min parameter.");
    if (!config_params_.GetReal("z_max", &z_max_))
      ROS_FATAL("Could not read the z_max parameter.");
  }

  // Callback for when a wifi signal is received
  void RFIDCallback(const isaac_hw_msgs::RFIDTags::ConstPtr& msg) {
    if (msg->signals.empty()) {
      ROS_DEBUG_STREAM("No station received, return");
      return;
    }
    // Scope Astrobee position
    geometry_msgs::TransformStamped tf;
    try {
    tf = tf_buffer_.lookupTransform(
       "world", "body", ros::Time(0));
    } catch (tf2::TransformException &ex) {
      ROS_DEBUG_STREAM("Transform failed" << ex.what());
      return;
    }
    if (tf.transform.translation.x == 0 &&
        tf.transform.translation.y == 0 &&
        (tf.transform.translation.z == -0.7 ||
         tf.transform.translation.z == 0))
      return;


    int station_pub;
    int max_signal_dbm  = -100;
    // Iterate over the received wifi signals
    for (int i = 0; i < msg->signals.size(); ++i) {
      // Find maximum intensity
      if (msg->signals[i].signal_dbm > max_signal_dbm) {
        max_signal_dbm = msg->signals[i].signal_dbm;
        station_pub = i;
      }

      // Map trace data
      if (calc_trace_) {
        // Check if the essid is new
        if (trace_maps_.find(msg->signals[i].tag) == trace_maps_.end()) {
          ROS_DEBUG_STREAM("New trace map " << msg->signals[i].tag <<
            " topic " << TOPIC_RFID_MAPPER_TRACE + std::string("_") + msg->signals[i].tag);
          volumetric_mapper::TraceMapper new_map_trace(nh_,
              TOPIC_RFID_MAPPER_TRACE + std::string("_") + msg->signals[i].tag,
              resolution_, min_signal_strength_, max_signal_strength_, alpha_trace_);
          trace_maps_.insert({msg->signals[i].tag, new_map_trace});
        }
        trace_maps_.find(msg->signals[i].tag)->second.AddTraceData(
                      msg->signals[i].signal_dbm, tf);
        trace_maps_.find(msg->signals[i].tag)->second.PubTraceData();
      }
      // Map volumetric data
      if (calc_volumetric_) {
        // Check if the essid is new
        if (volumetric_maps_.find(msg->signals[i].tag) == volumetric_maps_.end()) {
          ROS_DEBUG_STREAM("New vol map " << msg->signals[i].tag <<
            " topic " << TOPIC_RFID_MAPPER_MAP + std::string("_") + msg->signals[i].tag);
          volumetric_mapper::VolumetricMapper new_map_volumetric(nh_,
              TOPIC_RFID_MAPPER_MAP + std::string("_") + msg->signals[i].tag,
              resolution_, min_signal_strength_, max_signal_strength_, alpha_volumetric_,
              offset_signal_strength_);
          volumetric_maps_.insert({msg->signals[i].tag, new_map_volumetric});
        }
        volumetric_maps_.find(msg->signals[i].tag)->second.AddMapData(
                        msg->signals[i].signal_dbm, tf);
      }
    }
  }

  // Calback to publish the resulting wifi maps
  void RFIDPubCallback(const ros::TimerEvent&) {
    // Map volumetric data
    if (calc_volumetric_) {
      // Iterate over the received wifi signals
      std::map<std::string, volumetric_mapper::VolumetricMapper>::iterator it;
      for (it = volumetric_maps_.begin(); it != volumetric_maps_.end(); it++) {
        it->second.PubMapData(x_min_, x_max_, y_min_, y_max_, z_min_, z_max_);
      }
    }
  }

 private:
  ros::NodeHandle* nh_;
  // Parameters
  config_reader::ConfigReader config_params_;
  ros::Timer timer_pub_;
  tf2_ros::Buffer tf_buffer_;
  // ros::Timer pub_map_timer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  ros::Subscriber sub_;

  // Maps, string is essid
  std::map<std::string, volumetric_mapper::TraceMapper> trace_maps_;
  std::map<std::string, volumetric_mapper::VolumetricMapper> volumetric_maps_;

  // Parameters
  bool calc_trace_ = false;
  bool calc_volumetric_= false;
  double resolution_;
  double min_signal_strength_;
  double max_signal_strength_;
  double offset_signal_strength_;
  double alpha_trace_;
  double alpha_volumetric_;
  double x_min_, x_max_, y_min_, y_max_, z_min_, z_max_;
};
}  // namespace rfid_mapper

int main(int argc, char **argv) {
  ros::init(argc, argv, NODE_RFID_MAPPER);

  ros::NodeHandle nh;
  rfid_mapper::RFIDMapperNode wm;
  wm.Initialize(&nh);

  ros::spin();

  return 0;
}
