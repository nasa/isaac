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

// RFID Sensor library
#include <gazebo/sensors/RFIDSensor.hh>
#include <gazebo/physics/PhysicsTypes.hh>

// Tf2 includes
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>

// Sensor plugin interface
#include <astrobee_gazebo/astrobee_gazebo.h>

// FSW includes
#include <config_reader/config_reader.h>
#include <isaac_util/isaac_names.h>

// Wifi Sensor message
#include <isaac_hw_msgs/RFIDTags.h>

// TF2 support
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// STL includes
#include <string>

namespace gazebo {

class GazeboSensorPluginRFIDSensor : public FreeFlyerSensorPlugin {
 public:
  GazeboSensorPluginRFIDSensor() :
    FreeFlyerSensorPlugin("RFIDSensor", "", false) {
  }

  ~GazeboSensorPluginRFIDSensor() {}

 protected:
  // Called when plugin is loaded into gazebo
  void LoadCallback(ros::NodeHandle* nh,
    sensors::SensorPtr sensor, sdf::ElementPtr sdf) {
    // Get a link to the parent sensor
    sensor_ = std::dynamic_pointer_cast<sensors::RFIDSensor>(sensor);
    if (!sensor_) {
      gzerr << "GazeboSensorPluginWifi requires a valid sensor.\n";
      return;
    }

    // This ray will be used in TagStrength() for checking obstacles
    // between the transmitter and a given point.
    // ROS_ERROR_STREAM("World Name: " << sensor_->WorldName());
    // test_ray_ = boost::dynamic_pointer_cast<physics::RayShape>(
    //     physics::World(sensor_->WorldName()).GetPhysicsEngine()->CreateShape("ray", physics::CollisionPtr()));

    // // Read the sdf
    // if (sdf->HasElement("connected_essid"))
    //   connected_essid_ = sdf->Get<std::string>("connected_essid");

    // // Read the config file
    // config_reader::ConfigReader config_params;
    // // Set the config path to ISAAC
    // char *path;
    // if ((path = getenv("ISAAC_CONFIG_DIR")) == NULL) {
    //   NODELET_FATAL("Could not find config path");
    //   return;
    // }

    // config_params.SetPath(path);

    // config_params.AddFile("hw/wifi.config");
    // if (!config_params.ReadFiles()) NODELET_FATAL("Failed to read config files.");

    // // Get time in between measurements for Scan Station
    // if (!config_params.GetReal("time_scan_station", &time_scan_station_))
    //   NODELET_FATAL("Could not find row 'time_data' in table");

    // // Get time in between measurements for Scan All
    // if (!config_params.GetReal("time_scan_all", &time_scan_all_))
    //   NODELET_FATAL("Could not find row 'time_scan_all' in table");

    // if (!config_params.GetInt("max_networks", &max_networks_))
    //   NODELET_FATAL("Could not find row 'max_networks' in table");

    // // Publish static transform
    // geometry_msgs::TransformStamped msg;
    // static tf2_ros::StaticTransformBroadcaster br;
    // msg.header.frame_id = "body";
    // msg.child_frame_id = sensor_->Name();
    // msg.header.stamp = ros::Time::now();
    // msg.transform.translation.x = sensor_->Pose().Pos().X();
    // msg.transform.translation.y = sensor_->Pose().Pos().Y();
    // msg.transform.translation.z = sensor_->Pose().Pos().Z();
    // msg.transform.rotation.x = sensor_->Pose().Rot().X();
    // msg.transform.rotation.y = sensor_->Pose().Rot().Y();
    // msg.transform.rotation.z = sensor_->Pose().Rot().Z();
    // msg.transform.rotation.w = sensor_->Pose().Rot().W();
    // br.sendTransform(msg);

    // // Get node handle.
    // nh_gz_ = transport::NodePtr(new transport::Node());
    // // Initialise with default namespace (typically /gazebo/default/).
    // nh_gz_->Init();

    // // Connect to the sensor update event.
    // update_ = nh_gz_->Subscribe(sensor_->Topic(),
    //                 &GazeboSensorPluginWifiReceiver::UpdateCallback, this);

    // Create a transform buffer to listen for transforms
    tf_listener_ = std::shared_ptr<tf2_ros::TransformListener>(
      new tf2_ros::TransformListener(tf_buffer_));

    // Advertise RFID messages
    pub_ = nh->advertise<isaac_hw_msgs::RFIDTags>(
                          TOPIC_HARDWARE_RFID, 1);
    // Create Timer to scope station wireless signals
    timer_ = nh->createTimer(ros::Duration(3.0),
                  &GazeboSensorPluginRFIDSensor::CheckTags, this);
  }

  // Called on each sensor update event
  void UpdateCallback(ConstWirelessNodesPtr& msg) {
    // mutex_msg_.lock();
    // msg_ = *msg;
    // mutex_msg_.unlock();
  }

  void ScanStationCallback(ros::TimerEvent const& event) {
    // mutex_msg_.lock();
    // msgs::WirelessNodes msg = msg_;
    // mutex_msg_.unlock();
    // isaac_hw_msgs::WifiSignals msg_station;
    // msg_station.signals.resize(1);
    // for (int i = 0; i < msg.node_size(); ++i) {
    //   if (msg.node(i).essid() == connected_essid_) {
    //     msg_station.signals[0].header.stamp = ros::Time::now();
    //     msg_station.signals[0].header.frame_id = sensor_->Name();
    //     msg_station.signals[0].ssid = msg.node(i).essid();
    //     msg_station.signals[0].signal_dbm = msg.node(i).signal_level();
    //     // Publish station message
    //     pub_.publish(msg_station);
    //     break;
    //   }
    // }
  }

  void ScanAllCallback(ros::TimerEvent const& event) {
    // mutex_msg_.lock();
    // msgs::WirelessNodes msg = msg_;
    // mutex_msg_.unlock();

    // isaac_hw_msgs::WifiSignals msg_all;
    // msg_all.signals.resize(msg.node_size());
    // for (int i = 0; i < msg.node_size(); ++i) {
    //   msg_all.signals[i].header.stamp = ros::Time::now();
    //   msg_all.signals[i].header.frame_id = sensor_->Name();
    //   msg_all.signals[i].ssid = msg.node(i).essid();
    //   msg_all.signals[i].signal_dbm = msg.node(i).signal_level();
    // }
    // // Publish all message
    // pub_.publish(msg_all);
  }

  void CheckTags(ros::TimerEvent const& event) {
    isaac_hw_msgs::RFIDTags msg_tags;
    // Add the tag to all the RFID sensors.
    sensors::Sensor_V sensors = sensors::SensorManager::Instance()->GetSensors();
    for (sensors::Sensor_V::iterator iter = sensors.begin(); iter != sensors.end(); ++iter) {
      if ((*iter)->Type() == "rfidtag") {
          std::string sensor_name = (*iter)->Name();
          // ROS_ERROR_STREAM("RFID TAG FOUND " << sensor_name);

        // Get the current pose
        #if GAZEBO_MAJOR_VERSION > 7
        ignition::math::Vector3d end(GetModel()->WorldPose().Pos().X(),
                                      GetModel()->WorldPose().Pos().Y(),
                                      GetModel()->WorldPose().Pos().Z());
        #else
        ignition::math::Vector3d end(GetModel()->GetWorldPose().pos.x,
                                      GetModel()->GetWorldPose().pos.y,
                                      GetModel()->GetWorldPose().pos.z);
        #endif
        geometry_msgs::TransformStamped tf;
        try {
          // Look up the body frame in the berth frame
          tf = tf_buffer_.lookupTransform("world", (*iter)->Name(), ros::Time(0));
        } catch (tf2::TransformException &ex) {
          continue;
        }
        // Copy the transform
        ignition::math::Vector3d start(tf.transform.translation.x,
                                       tf.transform.translation.y,
                                       tf.transform.translation.z);
        // ROS_ERROR_STREAM("Position Sensor: " << end.X() << " " << end.Y() << " " << end.Z());
        // ROS_ERROR_STREAM("Tag Sensor: " << start.X() << " " << start.Y() << " " << start.Z());

        double distance = std::sqrt((start.X() - end.X()) * (start.X() - end.X())
                                  + (start.Y() - end.Y()) * (start.Y() - end.Y())
                                  + (start.Z() - end.Z()) * (start.Z() - end.Z()));
        // Looking for obstacles between start and end points
        // test_ray_->SetPoints(start, end);
        // std::string entityName;
        // double dist;
        // test_ray_->GetIntersection(dist, entityName);

        // ROS_ERROR_STREAM("entityName: " << entityName);


        // If the RFID tag is detected
        if (true) {
          // Fill in the message
          isaac_hw_msgs::RFIDTag msg_tag;
          msg_tag.header.stamp = ros::Time::now();
          msg_tag.header.frame_id = sensor_->Name();
          msg_tag.tag = (*iter)->Name();
          msg_tag.signal_dbm = distance;
          msg_tags.signals.push_back(msg_tag);
        }
      }
    }

    // Publish all message
    pub_.publish(msg_tags);
  }

 private:
  // ROS variables
  ros::Publisher pub_;                            // Wifi state publisher
  ros::Timer timer_;                              // Timer callback to publish
  std::mutex mutex_msg_;                          // Protect wifi message
  tf2_ros::Buffer tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  // Gazebo variables
  gazebo::transport::NodePtr nh_gz_;
  sensors::RFIDSensorPtr sensor_;
  gazebo::transport::SubscriberPtr update_;
  physics::RayShapePtr test_ray_;

  std::string connected_essid_ = "";
  double time_scan_station_, time_scan_all_;      // Time in between scans
  int max_networks_;                              // Maximum amounts of APs (Access Points)
};

GZ_REGISTER_SENSOR_PLUGIN(GazeboSensorPluginRFIDSensor)

}   // namespace gazebo
