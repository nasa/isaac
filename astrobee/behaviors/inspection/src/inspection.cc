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

// Include inspection library header
#include <inspection/inspection.h>
#include <inspection/panorama_survey.h>
#include <math.h>
/**
 * \ingroup beh
 */
namespace inspection {
/*
  This class provides the high-level logic that allows the freeflyer to
  define the optimal inspection pose. It reads the configurations and 
  evaluates:

  * Visibility constraints
  * Keepout and Keepin zones
  * Obstacle map

  It returns a vector of possible inspection poses that can be updated
  in case the move action fails due to planning or unmapped obstacle.
  It also constains functions that allow inspection visualization.
*/

Inspection::Inspection(ros::NodeHandle* nh, ff_util::ConfigServer* cfg) {
  // Setug config readers
  cfg_ = cfg;

  // Create a transform buffer to listen for transforms
  tf_listener_ = std::shared_ptr<tf2_ros::TransformListener>(new tf2_ros::TransformListener(tf_buffer_));

  // Service clients
  // Initialize the zones call
  client_z_.SetTimeoutCallback(std::bind(&Inspection::CheckZonesTimeoutCallback, this));
  client_z_.Create(nh, SERVICE_MOBILITY_GET_ZONES);

  // Initialize the obstacle map call
  client_o_.SetTimeoutCallback(std::bind(&Inspection::CheckMapTimeoutCallback, this));
  client_o_.Create(nh, SERVICE_MOBILITY_GET_OBSTACLE_MAP);

  // Publish debug messages
  pub_targets_ = nh->advertise<visualization_msgs::MarkerArray>(
    TOPIC_BEHAVIORS_INSPECTION_MARKERS + std::string("/targets"), 1, true);
  pub_markers_ = nh->advertise<visualization_msgs::MarkerArray>(
    TOPIC_BEHAVIORS_INSPECTION_MARKERS + std::string("/markers"), 1, true);
  pub_cam_ =
    nh->advertise<visualization_msgs::MarkerArray>(TOPIC_BEHAVIORS_INSPECTION_MARKERS + std::string("/cams"), 1, true);
}

void Inspection::ReadParam() {
  // Parameters Anomaly survey ---
  target_size_x_      = cfg_->Get<double>("target_size_x");
  target_size_y_      = cfg_->Get<double>("target_size_y");
  depth_cam_          = cfg_->Get<std::string>("depth_cam");

  // Get transform from target to cam
  // Camera uses z-axis pointing at the target and the target does z-axis pointing at camera
  try {
    geometry_msgs::TransformStamped tf_target_to_cam = tf_buffer_.lookupTransform("cam", "target",
                             ros::Time(0), ros::Duration(1.0));
    target_to_cam_rot_ = tf2::Quaternion(
                        tf_target_to_cam.transform.rotation.x,
                        tf_target_to_cam.transform.rotation.y,
                        tf_target_to_cam.transform.rotation.z,
                        tf_target_to_cam.transform.rotation.w);
  } catch (tf2::TransformException &ex) {
    ROS_ERROR("Failed getting target to sci_cam transform: %s", ex.what());
    target_to_cam_rot_ = tf2::Quaternion(0, 0, 0, 1);
  }

  // Parameters Panorama survey
  h_fov_    = cfg_->Get<double>("h_fov")    * M_PI  / 180.0;
  v_fov_    = cfg_->Get<double>("v_fov")    * M_PI  / 180.0;
  pan_min_  = cfg_->Get<double>("pan_min")  * M_PI  / 180.0;
  pan_max_  = cfg_->Get<double>("pan_max")  * M_PI  / 180.0;
  tilt_min_ = cfg_->Get<double>("tilt_min") * M_PI  / 180.0;
  tilt_max_ = cfg_->Get<double>("tilt_max") * M_PI  / 180.0;
  att_tol_  = cfg_->Get<double>("att_tol")  * M_PI  / 180.0;
  overlap_  = cfg_->Get<double>("overlap");
}

// Timeout on a zone check request
void Inspection::CheckZonesTimeoutCallback() {
  ROS_ERROR("Timeout connecting to the get zones service");
}

// Timeout on a map check request
void Inspection::CheckMapTimeoutCallback() {
  ROS_ERROR("Timeout connecting to the get map service");
}

// In case move failed, remove this point
bool Inspection::RemoveInspectionPose() {
  ROS_ERROR_STREAM("Counter " << inspection_counter_);
  points_[inspection_counter_].poses.erase(points_[inspection_counter_].poses.begin());
  if (points_[inspection_counter_].poses.empty())
    return false;
  else
    return true;
}

// Get the Inspection point on front of possibilities
geometry_msgs::PoseArray Inspection::GetCurrentInspectionPose() {
  // Draw camera frostum
  if (cameras_.find(curr_camera_) != cameras_.end()) {
    cameras_.find(curr_camera_)->second.DrawCameraFrustum(points_[inspection_counter_].poses.front(), pub_cam_);
  }

  geometry_msgs::PoseArray result;
  result.header = points_[inspection_counter_].header;
  result.poses.push_back(points_[inspection_counter_].poses.front());
  ROS_DEBUG_STREAM("next pose position x:" << result.poses.front().position.x
                                  << " y:" << result.poses.front().position.y
                                  << " z:" << result.poses.front().position.z
                              << "quat x:" << result.poses.front().orientation.x
                                  << " y:" << result.poses.front().orientation.y
                                  << " z:" << result.poses.front().orientation.z
                                  << " w:" << result.poses.front().orientation.w);
  return result;
}

// Skip pose
bool Inspection::NextInspectionPose() {
  ROS_ERROR_STREAM("NextInspectionPose " << inspection_counter_);
  if (inspection_counter_ + 1 < points_.size()) {
    inspection_counter_ += 1;
    return true;
  } else {
    return false;
  }
}
// Redo pose
bool Inspection::RedoInspectionPose() {
  ROS_ERROR_STREAM("RedoInspectionPose " << inspection_counter_);
  if (inspection_counter_ >= 0) {
    inspection_counter_ -= 1;
    return true;
  } else {
    return false;
  }
}
// Get poses
geometry_msgs::PoseArray Inspection::GetInspectionPoses() {
  ROS_ERROR_STREAM("GetInspectionPoses " << inspection_counter_);
  geometry_msgs::PoseArray result;
  result.header = points_.front().header;
  for (int i = inspection_counter_ + 1; i < points_.size(); ++i) {
    result.poses.push_back(points_[i].poses.front());
  }
  return result;
}

double Inspection::GetDistanceToTarget() {
  ROS_ERROR_STREAM("GetDistanceToTarget " << inspection_counter_);
  if (mode_ == "anomaly") {
    if (cameras_.find(curr_camera_) != cameras_.end()) {
      return cameras_.find(curr_camera_)->second.GetDistanceFromTarget(goal_.poses[inspection_counter_],
                                                              depth_cam_, target_size_x_, target_size_y_);
    }
  }
  return -1;
}

// Checks the given points agains whether the target is visible
// from a camera picture
bool Inspection::VisibilityConstraint(geometry_msgs::PoseArray &points, tf2::Transform target_transform) {
  // Go through all the points in sorted segment
  std::vector<geometry_msgs::Pose>::const_iterator it = points.poses.begin();
  tf2::Transform p1, p2, p3, p4;
  ROS_DEBUG_STREAM_ONCE("VisibilityConstraint target " << target_transform.getOrigin().x()
                                                       << target_transform.getOrigin().y()
                                                       << target_transform.getOrigin().z());
  while (it != points.poses.end()) {
    p1 =
      target_transform * tf2::Transform(tf2::Quaternion(0, 0, 0, 1), tf2::Vector3(target_size_x_, target_size_y_, 0));
    ROS_DEBUG_STREAM_ONCE("VisibilityConstraint p1 " << p1.getOrigin().x() << " " << p1.getOrigin().y() << " "
                                                     << p1.getOrigin().z());
    p2 = target_transform *
         tf2::Transform(tf2::Quaternion(0, 0, 0, 1), tf2::Vector3(target_size_x_, -target_size_y_, 0));
    ROS_DEBUG_STREAM_ONCE("VisibilityConstraint p2 " << p2.getOrigin().x() << " " << p2.getOrigin().y() << " "
                                                     << p2.getOrigin().z());
    p3 = target_transform *
         tf2::Transform(tf2::Quaternion(0, 0, 0, 1), tf2::Vector3(-target_size_x_, target_size_y_, 0));
    ROS_DEBUG_STREAM_ONCE("VisibilityConstraint p3 " << p3.getOrigin().x() << " " << p3.getOrigin().y() << " "
                                                     << p3.getOrigin().z());
    p4 = target_transform *
         tf2::Transform(tf2::Quaternion(0, 0, 0, 1), tf2::Vector3(-target_size_x_, -target_size_y_, 0));
    ROS_DEBUG_STREAM_ONCE("VisibilityConstraint p4 " << p4.getOrigin().x() << " " << p4.getOrigin().y() << " "
                                                     << p4.getOrigin().z());

    int x, y;
    if (cameras_.find(points.header.frame_id)
          ->second.GetCamXYFromPoint(*it, msg_conversions::tf2_transform_to_ros_pose(p1).position, x, y) &&
        cameras_.find(points.header.frame_id)
          ->second.GetCamXYFromPoint(*it, msg_conversions::tf2_transform_to_ros_pose(p2).position, x, y) &&
        cameras_.find(points.header.frame_id)
          ->second.GetCamXYFromPoint(*it, msg_conversions::tf2_transform_to_ros_pose(p3).position, x, y) &&
        cameras_.find(points.header.frame_id)
          ->second.GetCamXYFromPoint(*it, msg_conversions::tf2_transform_to_ros_pose(p4).position, x, y)) {
      ++it;
    } else {
      points.poses.erase(it);
    }
  }

  // Check if there are any points left
  if (points.poses.empty())
    return false;
  else
    return true;
}

// This function transforms the points from the camera rf to the body rf
bool Inspection::TransformList(geometry_msgs::PoseArray points_in, geometry_msgs::PoseArray &points_out,
                  tf2::Transform target_transform) {
  // Get transform from sci cam to body
  points_out = points_in;
  tf2::Transform sci_cam_to_body;
  try {
    geometry_msgs::TransformStamped tf_sci_cam_to_body = tf_buffer_.lookupTransform(points_in.header.frame_id, "body",
                             ros::Time(0), ros::Duration(1.0));
    sci_cam_to_body = msg_conversions::ros_tf_to_tf2_transform(tf_sci_cam_to_body.transform);
  } catch (tf2::TransformException &ex) {
    ROS_ERROR("Failed getting transform: %s", ex.what());
    sci_cam_to_body.setOrigin(tf2::Vector3(0, 0, 0));
    sci_cam_to_body.setRotation(tf2::Quaternion(0, 0, 0, 1));
  }

  tf2::Transform target_to_sci_cam;
  for (int i = 0; i < points_in.poses.size(); ++i) {
    // Convert to tf2 transform
    target_to_sci_cam = msg_conversions::ros_pose_to_tf2_transform(points_in.poses[i]);
    // Convert to body world pose
    tf2::Quaternion rotation = target_to_sci_cam.getRotation() * tf2::Quaternion(0, 0, -1, 0) * target_to_cam_rot_;
    target_to_sci_cam.setRotation(rotation);
    tf2::Transform robot_pose = target_transform * target_to_sci_cam * sci_cam_to_body;

    // Write back transformed point
    points_out.header.frame_id = "world";
    points_out.poses[i] = msg_conversions::tf2_transform_to_ros_pose(robot_pose);
  }
  return 0;
}

// Check if a point is inside a cuboid
bool Inspection::PointInsideCuboid(geometry_msgs::Point const& x,
                                      geometry_msgs::Vector3 const& cubemin,
                                      geometry_msgs::Vector3 const& cubemax) {
  if (x.x < std::min(cubemin.x, cubemax.x) ||
      x.y < std::min(cubemin.y, cubemax.y) ||
      x.z < std::min(cubemin.z, cubemax.z) ||
      x.x > std::max(cubemin.x, cubemax.x) ||
      x.y > std::max(cubemin.y, cubemax.y) ||
      x.z > std::max(cubemin.z, cubemax.z))
    return false;
  return true;
}

bool Inspection::ZonesConstraint(geometry_msgs::PoseArray &points) {
  ff_msgs::GetZones srv;
  if (client_z_.Call(srv)) {
    // Check each setpoint in the segment against the zones
    std::vector<geometry_msgs::Pose>::const_iterator it = points.poses.begin();
    while (it != points.poses.end()) {
      std::vector<ff_msgs::Zone>::iterator jt;
      // We must visit at least one keepin to be valid
      bool point_exists_within_keepin = false;
      bool point_exists_within_keepout = false;
      for (jt = srv.response.zones.begin(); jt != srv.response.zones.end(); jt++) {
        if (jt->type == ff_msgs::Zone::KEEPIN) {
          if (PointInsideCuboid(it->position, jt->min, jt->max))
            point_exists_within_keepin = true;
        }
        if (jt->type == ff_msgs::Zone::KEEPOUT) {
          if (PointInsideCuboid(it->position, jt->min, jt->max)) {
            ROS_DEBUG_STREAM("KEEPOUT violation");
            point_exists_within_keepout = true;
            break;
          }
        }
      }
      // Check that we are in a keepin
      if (!point_exists_within_keepin || point_exists_within_keepout) {
        ROS_DEBUG_STREAM("Not within KEEPIN/KEEPOUT zones");
        it = points.poses.erase(it);
      } else {
        ++it;
      }
    }
  } else {
    ROS_ERROR_STREAM("Could not call zones service, is it connected?");
  }

  // Check if there are any points left
  if (points.poses.empty())
    return false;
  else
    return true;
}

bool Inspection::ObstaclesConstraint(geometry_msgs::PoseArray &points) {
  ff_msgs::GetMap srv;
  if (client_o_.Call(srv)) {
    // Check the received points against the segment
    sensor_msgs::PointCloud point_cloud;
    sensor_msgs::convertPointCloud2ToPointCloud(srv.response.points, point_cloud);

    // Go through the point cloud of obstacles
    for (int i = 0 ; i < point_cloud.points.size(); ++i) {
      // Check each setpoint in the segment against the map
      std::vector<geometry_msgs::Pose>::const_iterator it = points.poses.begin();
      while (it != points.poses.end()) {
        // Check that we are in a keepin
        if (std::abs(it->position.x - point_cloud.points[i].x) <= srv.response.resolution &&
            std::abs(it->position.y - point_cloud.points[i].y) <= srv.response.resolution &&
            std::abs(it->position.z - point_cloud.points[i].z) <= srv.response.resolution) {
          it = points.poses.erase(it);
          break;
        } else {
          ++it;
        }
      }
    }
    } else {
      ROS_ERROR_STREAM("Could not call obstacle service, is it connected?");
    }
  // Check if there are any points left
  if (points.poses.empty())
    return false;
  else
    return true;
}

// Checks the given points agains whether the target is visible
// from a camera picture
void Inspection::DrawPoseMarkers(geometry_msgs::PoseArray &points, ros::Publisher &publisher) {
  // ROS_ERROR("Entering PubMapData");
  visualization_msgs::MarkerArray msg_visual;

  // Initialize marker message
  visualization_msgs::Marker marker;
  visualization_msgs::MarkerArray markers;
  // Fill in marker properties
  marker.header.frame_id = "world";
  marker.header.stamp = ros::Time::now();
  marker.ns = "";
  marker.type = visualization_msgs::Marker::ARROW;
  marker.action = visualization_msgs::Marker::ADD;
  // Arrow length
  marker.scale.x = 0.1;
  // Arrow width
  marker.scale.y = 0.01;
  // Arrow height
  marker.scale.z = 0.01;

  for (int i = 0; i < points.poses.size(); ++i) {
    // Pivot point
    marker.pose.position = points.poses[i].position;

    // Points along the x-axis
    marker.id =  i * 3;
    marker.pose.orientation = points.poses[i].orientation;
    // Define color
    marker.color.r = 1;
    marker.color.g = 0;
    marker.color.b = 0;
    marker.color.a = 1.0;
    // Add arrow for visualization
    msg_visual.markers.push_back(marker);

    // Points along the y-axis
    marker.id =  1 + i * 3;
    marker.pose.orientation = msg_conversions::eigen_to_ros_quat(
      msg_conversions::ros_to_eigen_quat(points.poses[i].orientation) * Eigen::Quaterniond(0.707, 0, 0, 0.707));
    // Define color
    marker.color.r = 0;
    marker.color.g = 1;
    marker.color.b = 0;
    marker.color.a = 1.0;
    // Add arrow for visualization
    msg_visual.markers.push_back(marker);

    // Points along the z-axis
    marker.id =  2 + i * 3;
    marker.pose.orientation = msg_conversions::eigen_to_ros_quat(
      msg_conversions::ros_to_eigen_quat(points.poses[i].orientation) * Eigen::Quaterniond(0.707, 0, -0.707, 0));
    // Define color
    marker.color.r = 0;
    marker.color.g = 0;
    marker.color.b = 1;
    marker.color.a = 1.0;
    // Add arrow for visualization
    msg_visual.markers.push_back(marker);
  }
  // Publish marker message
  publisher.publish(msg_visual);
}


// Generate inspection segment
bool Inspection::GenerateAnomalySurvey(geometry_msgs::PoseArray &points_anomaly) {
  mode_ = "anomaly";
  inspection_counter_ = -1;
  goal_ = points_anomaly;
  // Draw pose targets for rviz visualization
  DrawPoseMarkers(points_anomaly, pub_targets_);

  // Update parameters
  ReadParam();
  // Create the sorted point segment
  points_.clear();
  geometry_msgs::PoseArray survey_template;
  survey_template.header.frame_id = points_anomaly.header.frame_id;
  // Generate Sorted list of inspection candidates for this target
  GenerateSortedList(survey_template);

  // Insert Offset
  for (int i = 0; i < points_anomaly.poses.size(); ++i) {
    tf2::Transform anomaly_transform;
    anomaly_transform = msg_conversions::ros_pose_to_tf2_transform(points_anomaly.poses[i]);

    points_.push_back(survey_template);

    // Make sure camera is loaded
    std::string cam_name = points_anomaly.header.frame_id.c_str();
    curr_camera_ = cam_name.c_str();
    if (cameras_.find(cam_name) == cameras_.end())
      cameras_.emplace(std::piecewise_construct, std::make_tuple(cam_name),
                       std::make_tuple(cam_name, cfg_->Get<double>("max_distance"), cfg_->Get<double>("min_distance")));


    // Transform the points from the camera reference frame to the robot body
    ROS_DEBUG_STREAM("next pose position x:" << points_[i].poses.front().position.x
                                    << " y:" << points_[i].poses.front().position.y
                                    << " z:" << points_[i].poses.front().position.z
                                << "quat x:" << points_[i].poses.front().orientation.x
                                    << " y:" << points_[i].poses.front().orientation.y
                                    << " z:" << points_[i].poses.front().orientation.z
                                    << " w:" << points_[i].poses.front().orientation.w);
    TransformList(points_[i], points_[i], anomaly_transform);
    ROS_DEBUG_STREAM("next pose position x:" << points_[i].poses.front().position.x
                                   << " y:" << points_[i].poses.front().position.y
                                    << " z:" << points_[i].poses.front().position.z
                                << "quat x:" << points_[i].poses.front().orientation.x
                                   << " y:" << points_[i].poses.front().orientation.y
                                   << " z:" << points_[i].poses.front().orientation.z
                                    << " w:" << points_[i].poses.front().orientation.w);


    DrawPoseMarkers(points_[i], pub_markers_);

    // Draw the poses generated that capture the target
    points_[i].header.frame_id = "sci_cam";
    if (!VisibilityConstraint(points_[i], anomaly_transform)) {
      ROS_ERROR("Visibility Constrains: Did not find a possible inspection point");
      return false;
    }
    ROS_DEBUG_STREAM("Visibility Constrained");

    // Check candidate segment agains zones
    if (!ZonesConstraint(points_[i])) {
      ROS_ERROR_STREAM("Zones Constrains: Did not find a possible inspection point");
      return false;
    }
    ROS_DEBUG_STREAM("Zones Constrained");

    // Check candidate segment against obstacle map
    if (!ObstaclesConstraint(points_[i])) {
      ROS_ERROR_STREAM("Obstacles Constrains: Did not find a possible inspection point");
      return false;
    }
    ROS_DEBUG_STREAM("Obstacles Constrained");
  }
  return true;
}


// Insert here any geometric survey functionality
bool Inspection::GenerateGeometrySurvey(geometry_msgs::PoseArray &points_geometry) {
  mode_ = "geometry";
  inspection_counter_ = -1;
  goal_ = points_geometry;
  // Draw pose targets for rviz visualization
  DrawPoseMarkers(points_geometry, pub_targets_);

  points_.clear();

  geometry_msgs::PoseArray pose;
  pose.header = points_geometry.header;
  pose.poses.push_back(points_geometry.poses[0]);
  for (int i = 0; i < points_geometry.poses.size(); ++i) {
    pose.poses[0] = points_geometry.poses[i];
    points_.push_back(pose);
  }
  return true;
}

// Generate the survey for panorama pictures
bool Inspection::GeneratePanoramaSurvey(geometry_msgs::PoseArray &points_panorama) {
  mode_ = "panorama";
  inspection_counter_ = -1;
  goal_ = points_panorama;
  // Draw pose targets for rviz visualization
  DrawPoseMarkers(points_panorama, pub_targets_);
  points_.clear();
  // Update parameters
  ReadParam();

  // Make sure camera is loaded
  std::string cam_name = points_panorama.header.frame_id.c_str();
  curr_camera_ = cam_name.c_str();
  if (cameras_.find(cam_name) == cameras_.end())
    cameras_.emplace(std::piecewise_construct, std::make_tuple(cam_name),
                     std::make_tuple(cam_name, cfg_->Get<double>("max_distance"), cfg_->Get<double>("min_distance")));

  geometry_msgs::PoseArray panorama_relative;
  geometry_msgs::PoseArray panorama_transformed;

  // Insert point
  geometry_msgs::Pose point;
  panorama_relative.header.frame_id = cam_name.c_str();
  point.position.x = 0.0;
  point.position.y = 0.0;
  point.position.z = 0.0;
  tf2::Quaternion panorama_rotation;

  // Calculate fields of view
  float h_fov, v_fov;
  h_fov = (h_fov_ < 0) ? cameras_.find(cam_name)->second.GetHFOV() : h_fov_;
  v_fov = (v_fov_ < 0) ? cameras_.find(cam_name)->second.GetVFOV() : v_fov_;

  int nrows, ncols;
  std::vector<PanoAttitude> orientations;

  // pano_orientations() doesn't support panos with non-zero center (not needed)
  if (pan_min_ != -pan_max_) {
    ROS_ERROR_STREAM(" Pan min: " << pan_min_ << " ; Pan max: " << pan_max_
                                  << "; They are different! Making pan_min = pan_max");
    return false;
  }
  if (tilt_min_ != -tilt_max_) {
    ROS_ERROR_STREAM(" Tilt min: " << tilt_min_ << " ; Tilt max: " << tilt_max_
                                   << "; They are different! Making tilt_min = tilt_max");
    return false;
  }

  // Generate coverage pattern pan/tilt values
  GeneratePanoOrientations(&orientations, &nrows, &ncols,
                    pan_max_, tilt_max_,
                    h_fov, v_fov,
                    overlap_, att_tol_);

  // Go through all the panorama center locations
  panorama_relative.poses.clear();
  panorama_relative.poses.push_back(point);
  // panorama_relative.poses.resize(1)
  for (const auto& point_panorama : points_panorama.poses) {
    for (const auto& orient : orientations) {
      ROS_DEBUG_STREAM("pan:" << orient.pan * 180 / M_PI << " tilt:" << orient.tilt * 180 / M_PI);
      panorama_rotation.setRPY(0, orient.tilt, orient.pan);
      panorama_relative.poses[0].orientation = msg_conversions::tf2_quat_to_ros_quat(panorama_rotation);

      // Transform the points from the camera reference frame to the robot body
      TransformList(panorama_relative, panorama_transformed,
                    msg_conversions::ros_pose_to_tf2_transform(point_panorama));
      points_.push_back(panorama_transformed);
    }
  }
  return true;
}

// Insert here any volumetric survey functionality
bool Inspection::GenerateVolumetricSurvey(geometry_msgs::PoseArray &points_volume) {
  mode_ = "volumetric";
  inspection_counter_ = -1;
  goal_ = points_volume;
  // Draw pose targets for rviz visualization
  DrawPoseMarkers(points_volume, pub_targets_);
  points_.clear();

  geometry_msgs::PoseArray pose;
  pose.header = points_volume.header;
  pose.poses.push_back(points_volume.poses[0]);
  for (int i = 0; i < points_volume.poses.size(); ++i) {
    pose.poses[0] = points_volume.poses[i];
    points_.push_back(pose);
  }
  return true;
}
}  // namespace inspection

