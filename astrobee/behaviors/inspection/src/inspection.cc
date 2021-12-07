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
#define PI 3.1415
#define EPS 1e-5
/**
 * \ingroup beh
 */
namespace inspection {
/*
  This class provides the high-level logic that allows the freeflyer to
  define the optimal inspection pose. It evaluates:

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

    cfg_cam_.AddFile("cameras.config");
    if (!cfg_cam_.ReadFiles())
      ROS_FATAL("Failed to read config files.");

    // Create a transform buffer to listen for transforms
    tf_listener_ = std::shared_ptr<tf2_ros::TransformListener>(
      new tf2_ros::TransformListener(tf_buffer_));
    // Service clients
    // Initialize the zones call
    client_z_.SetConnectedCallback(std::bind(&Inspection::ConnectedCallback, this));
    client_z_.SetTimeoutCallback(std::bind(&Inspection::CheckZonesTimeoutCallback, this));
    client_z_.Create(nh, SERVICE_MOBILITY_GET_ZONES);
    // Initialize the obstacle map call
    client_o_.SetConnectedCallback(std::bind(&Inspection::ConnectedCallback, this));
    client_o_.SetTimeoutCallback(std::bind(&Inspection::CheckMapTimeoutCallback, this));
    client_o_.Create(nh, SERVICE_MOBILITY_GET_OBSTACLE_MAP);

    // Publish
    pub_no_filter_ = nh->advertise<visualization_msgs::MarkerArray>(
                      "markers/no_filter", 1, true);
    pub_vis_check_ = nh->advertise<visualization_msgs::MarkerArray>(
                      "markers/vis_check", 1, true);
    pub_zones_check_ = nh->advertise<visualization_msgs::MarkerArray>(
                      "markers/zones_check", 1, true);
    pub_map_check_ = nh->advertise<visualization_msgs::MarkerArray>(
                      "markers/map_check", 1, true);

    ReadParam();
  }

  void Inspection::ReadParam() {
    // Parameters Anomaly survey
    opt_distance_       = cfg_->Get<double>("optimal_distance");
    dist_resolution_    = cfg_->Get<double>("distance_resolution");
    angle_resolution_   = cfg_->Get<double>("angle_resolution");
    max_angle_          = cfg_->Get<double>("max_angle");
    max_distance_       = cfg_->Get<double>("max_distance");
    min_distance_       = cfg_->Get<double>("min_distance");
    target_size_x_      = cfg_->Get<double>("target_size_x");
    target_size_y_      = cfg_->Get<double>("target_size_y");
    vent_to_scicam_rot_ = tf2::Quaternion(cfg_->Get<double>("vent_to_sci_cam_rotation_x"),
                                          cfg_->Get<double>("vent_to_sci_cam_rotation_y"),
                                          cfg_->Get<double>("vent_to_sci_cam_rotation_z"),
                                          cfg_->Get<double>("vent_to_sci_cam_rotation_w"));

    // Parameters Panorama survey
    pan_min_  = cfg_->Get<double>("pan_min") * PI / 180.0;
    pan_max_  = cfg_->Get<double>("pan_max") * PI / 180.0;
    tilt_min_ = cfg_->Get<double>("tilt_min") * PI / 180.0;
    tilt_max_ = cfg_->Get<double>("tilt_max") * PI / 180.0;
    overlap_  = cfg_->Get<double>("overlap");
  }

  // Ensure all clients are connected
  void Inspection::ConnectedCallback() {
    ROS_DEBUG_STREAM("ConnectedCallback()");
    if (!client_z_.IsConnected()) return;       // Zone check service
    if (!client_o_.IsConnected()) return;       // Map check service
    // fsm_.Update(READY);                         // Ready!
  }

  // Timeout on a zone check request
  void Inspection::CheckZonesTimeoutCallback() {
    ROS_ERROR("Timeout connecting to the get zones service");
  }

  // Timeout on a map check request
  void Inspection::CheckMapTimeoutCallback() {
    ROS_ERROR("Timeout connecting to the get map service");
  }

  bool Inspection::RemoveInspectionPose() {
    points_.poses.erase(points_.poses.begin());

    DrawInspectionPoses(points_, pub_map_check_);
    if (points_.poses.empty())
      return false;
    else
      return true;
  }

  geometry_msgs::Pose Inspection::GetInspectionPose() {
    return points_.poses.front();
  }

  // MOVE ACTION CLIENT
  // Generate inspection segment
  bool Inspection::GenSegment(geometry_msgs::Pose goal) {
    // Insert Offset
    tf2::Transform vent_transform;
    vent_transform.setOrigin(tf2::Vector3(
                      goal.position.x,
                      goal.position.y,
                      goal.position.z));
    vent_transform.setRotation(tf2::Quaternion(
                      goal.orientation.x,
                      goal.orientation.y,
                      goal.orientation.z,
                      goal.orientation.w));

    // Create the sorted point segment
    points_.poses.clear();
    points_.header.frame_id = "sci_cam";
    GenerateSortedList(points_);
    // ROS_ERROR_STREAM("end GenerateSortedList");

    // Draw the poses generated that capture the target
    if (!VisibilityConstraint(points_)) {
      ROS_ERROR_STREAM("Visibility Constrained: Did not find a possible inspection point");
      return false;
    }
    // DrawInspectionPoses(points_, pub_vis_check_);

    // Transform the points from the camera reference frame to the robot body
    TransformList(points_, points_, vent_transform);

    // Check candidate segment agains zones
    if (!ZonesConstraint(points_)) {
      ROS_ERROR_STREAM("Zones Constrained: Did not find a possible inspection point");
      return false;
    }

    // Check candidate segment against obstacle map
    if (!ObstaclesConstraint(points_)) {
      ROS_ERROR_STREAM("Obstacles Constrained: Did not find a possible inspection point");
      return false;
    }
    // ROS_ERROR_STREAM("end ObstaclesConstraint");
    return true;
  }

  // This function generates a sorted list based on the max viewing angle and resolution
  bool Inspection::GenerateSortedList(geometry_msgs::PoseArray &points) {
    geometry_msgs::Pose point;

    // Insert point
    point.orientation.x = 1;
    point.orientation.y = 0;
    point.orientation.z = 0;
    point.orientation.w = 0;

    // Go through all the alternative points in preference order
    for (double r = 0; (r < max_distance_ - opt_distance_) ||
                       (r < opt_distance_ - min_distance_); r += dist_resolution_) {
      for (double theta = 0; theta < max_angle_; theta += angle_resolution_) {
        for (double phi = 0; phi < 2*3.14; phi += angle_resolution_) {
              // ROS_ERROR_STREAM("r: " << r << " phi: " << phi << " z: " << theta);
          if ((opt_distance_ + r < max_distance_) && r != 0) {   // avoid publishing twice on zero
            // Insert point
            point.position.x = (opt_distance_ + r) * sin(theta) * cos(phi);
            point.position.y = (opt_distance_ + r) * sin(theta) * sin(phi);
            point.position.z = (opt_distance_ + r)  * cos(theta);
            points_.poses.push_back(point);
            // Insert point
            if (theta != 0) {   // avoid publishing twice on zero
              point.position.x = (opt_distance_ + r) * sin(-theta) * cos(phi);
              point.position.y = (opt_distance_ + r) * sin(-theta) * sin(phi);
              point.position.z = (opt_distance_ + r)  * cos(-theta);
              points_.poses.push_back(point);
            }
          }
          if (opt_distance_ - r > min_distance_) {
            // Insert point
            point.position.x = (opt_distance_ - r) * sin(theta) * cos(phi);
            point.position.y = (opt_distance_ - r) * sin(theta) * sin(phi);
            point.position.z = (opt_distance_ - r)  * cos(theta);
            points_.poses.push_back(point);
            // Insert point
            if (theta != 0) {   // avoid publishing twice on zero
              point.position.x = (opt_distance_ - r) * sin(-theta) * cos(phi);
              point.position.y = (opt_distance_ - r) * sin(-theta) * sin(phi);
              point.position.z = (opt_distance_ - r)  * cos(-theta);
              points_.poses.push_back(point);
            }
          }
          if (theta == 0)
            break;
        }
      }
    }
    return 0;
  }

  // Checks the given points agains whether the target is visible
  // from a camera picture
  bool Inspection::VisibilityConstraint(geometry_msgs::PoseArray &points) {
    // Get camera parameters
    Eigen::Matrix3d cam_mat;
    float fx, fy, s, cx, cy;
    int W, H;

    config_reader::ConfigReader::Table camera(&cfg_cam_, points.header.frame_id.c_str());
    // Read in distorted image size.
    if (!camera.GetInt("width", &W))
      fprintf(stderr, "Could not read camera width.");
    if (!camera.GetInt("height", &H))
      fprintf(stderr, "Could not read camera height.");

    config_reader::ConfigReader::Table vector(&camera, "intrinsic_matrix");
    for (int i = 0; i < 9; i++) {
      if (!vector.GetReal((i + 1), &cam_mat(i / 3, i % 3))) {
        fprintf(stderr, "Failed to read vector intrinsic_matrix.");
        break;
      }
    }
    // Read in focal length, optical offset and skew
    fx = cam_mat(0, 0);
    fy = cam_mat(1, 1);
    s  = cam_mat(0, 1);
    cx = cam_mat(0, 2);
    cy = cam_mat(1, 2);

    // Build the matrix with the points to evaluate
    Eigen::MatrixXd p(4, 4);
    p << target_size_x_,  target_size_x_, -target_size_x_, -target_size_x_,
         target_size_y_, -target_size_y_,  target_size_y_, -target_size_y_,
         0,             0,             0,              0,
         1,             1,             1,              1;

    // Build projection matrix
    float farmnear = max_distance_ - min_distance_;
    Eigen::Matrix4d P;
    P << 2*fx/W,     0,       0,                                                0,
         2*s/W,      2*fy/H,  0,                                                0,
         2*(cx/W)-1, 2*(cy/H)-1, max_distance_ / (farmnear),                    1,
         0,          0,          -min_distance_ * (max_distance_ / (farmnear)), 1;

    // Go through all the points in sorted segment
    std::vector<geometry_msgs::Pose>::const_iterator it = points.poses.begin();
    while (it != points.poses.end()) {
      // Build the View matrix
      Eigen::Quaterniond R(1, 0, 0, 0);                                    // Rotation Matrix Identity
      Eigen::Vector3d T(it->position.x, it->position.y, it->position.z);   // Translation Vector
      Eigen::Matrix4d V;                                                   // Transformation Matrix
      V.setIdentity();                                                     // Identity to make bottom row 0,0,0,1
      V.block<3, 3>(0, 0) = R.normalized().toRotationMatrix();;
      V.block<3, 1>(0, 3) = T;
      // Transform point
      Eigen::MatrixXd q = P * V.inverse() * p;

      bool eliminated = false;
      for (int i = 0; i < q.cols(); ++i) {
        if (q(0, i)/q(3, i) < -1 ||   // the point lies beyond the left border of the screen
            q(0, i)/q(3, i) >  1 ||   // the point lies beyond the right border of the screen
            q(1, i)/q(3, i) < -1 ||   // the point lies beyond the bottom border of the screen
            q(1, i)/q(3, i) >  1 ||   // the point lies beyond the top border of the screen
            q(2, i)/q(3, i) < -1 ||   // the point lies beyond the near plane of the camera,
                                      //     i.e., the point is behind the camera or too close for the camera to see.
            q(2, i)/q(3, i) >  1) {   // the point lies beyond the far plane of the camera,
                                      //     i.e., the point is too far away for the camera to see
              // Eliminate point
              points.poses.erase(it);
              eliminated = true;
              break;
        }
      }
      if (!eliminated)
        ++it;
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
                               ros::Time(0));
      sci_cam_to_body.setOrigin(tf2::Vector3(
                          tf_sci_cam_to_body.transform.translation.x,
                          tf_sci_cam_to_body.transform.translation.y,
                          tf_sci_cam_to_body.transform.translation.z));
      sci_cam_to_body.setRotation(tf2::Quaternion(
                          tf_sci_cam_to_body.transform.rotation.x,
                          tf_sci_cam_to_body.transform.rotation.y,
                          tf_sci_cam_to_body.transform.rotation.z,
                          tf_sci_cam_to_body.transform.rotation.w));
    } catch (tf2::TransformException &ex) {
      ROS_ERROR("ERROR getting sci_cam transform: %s", ex.what());
    }

    tf2::Transform target_to_sci_cam;
    for (int i = 0; i < points_in.poses.size(); ++i) {
      // Convert to tf2 transform
      target_to_sci_cam.setOrigin(tf2::Vector3(
                          points_in.poses[i].position.x,
                          points_in.poses[i].position.y,
                          points_in.poses[i].position.z));
      target_to_sci_cam.setRotation(tf2::Quaternion(points_in.poses[i].orientation.x,
                                          points_in.poses[i].orientation.y,
                                          points_in.poses[i].orientation.z,
                                          points_in.poses[i].orientation.w));
      // Convert to body world pose
      tf2::Transform robot_pose = target_transform * target_to_sci_cam * sci_cam_to_body;

      // Write back transformed point
      points_out.header.frame_id = "world";
      points_out.poses[i].position.x = robot_pose.getOrigin().x();
      points_out.poses[i].position.y = robot_pose.getOrigin().y();
      points_out.poses[i].position.z = robot_pose.getOrigin().z();
      points_out.poses[i].orientation.x = robot_pose.getRotation().x();
      points_out.poses[i].orientation.y = robot_pose.getRotation().y();
      points_out.poses[i].orientation.z = robot_pose.getRotation().z();
      points_out.poses[i].orientation.w = robot_pose.getRotation().w();
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
    ROS_DEBUG_STREAM("Service zones");
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
          if (jt->type == ff_msgs::Zone::KEEPIN)
            if (PointInsideCuboid(it->position, jt->min, jt->max))
              point_exists_within_keepin = true;
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
    }

    // Check if there are any points left
    if (points.poses.empty())
      return false;
    else
      return true;
  }

  // Checks the given points agains whether the target is visible
  // from a camera picture
  void Inspection::DrawInspectionPoses(geometry_msgs::PoseArray &points, ros::Publisher &publisher) {
    // ROS_ERROR("Entering PubMapData");
    visualization_msgs::MarkerArray msg_visual;

    double x[] = {0, 0, 0};
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
    marker.scale.z = 0.05;

    for (int i = 0; i < points.poses.size(); ++i) {
      marker.id = i;
      // Pivot point
      marker.pose.position = points.poses[i].position;
      // Poits along the x-axis
      marker.pose.orientation = points.poses[i].orientation;
      // Define color
      // marker.color = IntensityMapColor((float) i / (float) points.size(), 1.0);
      if (i == 0) {
        marker.color.r = 0;
        marker.color.g = 1;
        marker.color.b = 0;
        marker.color.a = 1.0;
      } else if (i == 1) {
        marker.color.r = 1;
        marker.color.g = 0;
        marker.color.b = 0;
        marker.color.a = 1.0;
      }
      // Add arrow for visualization
      msg_visual.markers.push_back(marker);
    }
    // Publish marker message
    publisher.publish(msg_visual);
  }

void Inspection::DrawInspectionFrostum() {
}

// Generate the survey for panorama pictures
void Inspection::GeneratePanoramaSurvey(geometry_msgs::PoseArray &points_panorama) {
  geometry_msgs::PoseArray panorama_relative;
  geometry_msgs::PoseArray panorama_transformed;
  geometry_msgs::PoseArray panorama_survey;

  // Insert point
  geometry_msgs::Pose point;
  panorama_relative.header.frame_id = points_panorama.header.frame_id.c_str();
  point.position.x = 0.0;
  point.position.y = 0.0;
  point.position.z = 0.0;
  tf2::Quaternion panorama_rotation;


  // Get camera parameters
  Eigen::Matrix3d cam_mat;
  float fx, fy;
  int W, H;

  // Read in distorted image size.
  config_reader::ConfigReader::Table camera(&cfg_cam_, points_panorama.header.frame_id.c_str());
  if (!camera.GetInt("width", &W)) {
    ROS_ERROR("Could not read camera width.");
  }
  if (!camera.GetInt("height", &H)) {
    ROS_ERROR("Could not read camera height.");
  }
  config_reader::ConfigReader::Table vector(&camera, "intrinsic_matrix");
  for (int i = 0; i < 9; i++) {
    if (!vector.GetReal((i + 1), &cam_mat(i / 3, i % 3))) {
      ROS_ERROR("Failed to read vector intrinsic_matrix.");
      break;
    }
  }
  // Read in focal length
  fx = cam_mat(0, 0);
  fy = cam_mat(1, 1);

  // Calculate field of views
  float h_fov = 2 * atan(W / (2 * fx));
  float v_fov = 2 * atan(H / (2 * fy));

  // Calculate spacing between pictures
  double k_pan  = (pan_max_ - pan_min_) / std::ceil((pan_max_ - pan_min_) / (h_fov * (1 - overlap_)));
  double k_tilt = (tilt_max_ - tilt_min_) / std::ceil((tilt_max_ - tilt_min_) / (v_fov * (1 - overlap_)));

  // Case where max and min is zero
  if (std::isnan(k_pan)) k_pan = PI;
  if (std::isnan(k_tilt)) k_tilt = PI;

  // If it's a full 360, skip the last one
  if (pan_max_ - pan_min_ >= 2*PI) pan_max_-= 2 * EPS;  // Go through all the points

  // Generate all the pan/tilt values
  for (double tilt = tilt_min_; tilt <= tilt_max_ + EPS; tilt += k_tilt) {
    for (double pan = pan_min_; pan <= pan_max_ + EPS; pan += k_pan) {
      ROS_DEBUG_STREAM("pan:" << pan * 180 / PI << " tilt:" << tilt * 180 / PI);
      panorama_rotation.setRPY(0, tilt, pan);
      panorama_rotation = panorama_rotation * tf2::Quaternion(0, 0, -1, 0) * vent_to_scicam_rot_;
      point.orientation.x = panorama_rotation.x();
      point.orientation.y = panorama_rotation.y();
      point.orientation.z = panorama_rotation.z();
      point.orientation.w = panorama_rotation.w();
      panorama_relative.poses.push_back(point);
      if (tilt == -PI/2 || tilt == PI/2)
        break;
    }
  }

  // Go through all the panorama points
  for (int i = 0; i < points_panorama.poses.size(); ++i) {
    // Transform the points from the camera reference frame to the robot body

    tf2::Transform panorama_pose;
    panorama_pose.setOrigin(tf2::Vector3(
                      points_panorama.poses[i].position.x,
                      points_panorama.poses[i].position.y,
                      points_panorama.poses[i].position.z));
    panorama_pose.setRotation(tf2::Quaternion(
                      points_panorama.poses[i].orientation.x,
                      points_panorama.poses[i].orientation.y,
                      points_panorama.poses[i].orientation.z,
                      points_panorama.poses[i].orientation.w));
    TransformList(panorama_relative, panorama_transformed, panorama_pose);

    panorama_survey.poses.insert(std::end(panorama_survey.poses), std::begin(panorama_transformed.poses),
                                  std::end(panorama_transformed.poses));
  }
  points_panorama = panorama_survey;
}

}  // namespace inspection

