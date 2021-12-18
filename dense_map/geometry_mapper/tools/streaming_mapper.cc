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
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/CameraInfo.h>
#include <ff_msgs/VisualLandmarks.h>
#include <ff_msgs/EkfState.h>
#include <eigen_conversions/eigen_msg.h>
#include <std_msgs/Float64MultiArray.h>

// PCL includes
#include <pcl/io/ply_io.h>

// OpenCV includes
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>

// Eigen includes
#include <Eigen/Geometry>
#include <Eigen/Core>

// FreeFlyer includes
#include <ff_util/ff_names.h>
#include <ff_common/utils.h>
#include <camera/camera_model.h>
#include <config_reader/config_reader.h>

// isaac includes
#include <dense_map_utils.h>
#include <texture_processing.h>

// System includes
#include <unistd.h>
#include <string>
#include <map>
#include <iostream>
#include <fstream>
#include <thread>
#include <mutex>

namespace dense_map {

class StreamingMapper {
 public:
  StreamingMapper();
  virtual ~StreamingMapper();
  void Initialize(ros::NodeHandle& nh);
  void ProcessingLoop();

 private:
  void ReadParams(ros::NodeHandle const& nh);

  void publishTexturedMesh(mve::TriangleMesh::ConstPtr mesh, std::shared_ptr<BVHTree> bvh_tree,
                           double max_iso_times_exposure, double iso, double exposure, int processed_camera_count,
                           cv::Mat const& image, double image_timestamp, camera::CameraModel const& cam,
                           std::vector<double>& smallest_cost_per_face, std::string const& out_prefix);

  void TextureCamSimPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& pose);
  void TextureCamSimInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& info);
  void LandmarkCallback(ff_msgs::VisualLandmarks::ConstPtr const& vl);
  void EkfStateCallback(ff_msgs::EkfState::ConstPtr const& ekf_state);
  void EkfPoseCallback(geometry_msgs::PoseStamped::ConstPtr const& ekf_pose);
  void SciCamExifCallback(std_msgs::Float64MultiArray::ConstPtr const& exif);
  void CompressedTextureCallback(const sensor_msgs::CompressedImageConstPtr& msg);
  void UncompressedTextureCallback(const sensor_msgs::ImageConstPtr& msg);
  void AddTextureCamPose(double nav_cam_timestamp, Eigen::Affine3d const& nav_cam_pose);
  void WipeOldImages();

  std::shared_ptr<image_transport::ImageTransport> image_transport;
  ros::Subscriber compressed_texture_image_sub;
  image_transport::Subscriber uncompressed_texture_image_sub;
  ros::Subscriber texture_cam_pose_sub;
  ros::Subscriber texture_cam_info_sub;
  ros::Subscriber landmark_sub;
  ros::Subscriber ekf_state_sub;
  ros::Subscriber ekf_pose_sub;
  ros::Subscriber sci_cam_exif_sub;
  ros::Publisher texture_img_pub;
  ros::Publisher texture_obj_pub;
  ros::Publisher texture_mtl_pub;

  // Parameters
  config_reader::ConfigReader config_params;

  camera::CameraParameters m_texture_cam_params;
  sensor_msgs::Image m_texture_obj_msg, m_texture_mtl_msg;

  double m_navcam_to_texture_cam_timestamp_offset;
  Eigen::MatrixXd m_texture_cam_to_navcam_trans;
  Eigen::MatrixXd m_navcam_to_body_trans;

  std::string nav_cam_pose_topic, ekf_state_topic, ekf_pose_topic, texture_cam_topic, sci_cam_exif_topic,
    texture_cam_type, mesh_file;
  bool sim_mode, save_to_disk, use_single_texture;

  // For meshing
  mve::TriangleMesh::Ptr mesh;
  std::shared_ptr<mve::MeshInfo> mesh_info;
  std::shared_ptr<tex::Graph> graph;  // TODO(oalexan1): Is this necessary?
  std::shared_ptr<BVHTree> bvh_tree;

  // Each callback must have a lock
  std::mutex nav_cam_pose_lock, texture_cam_pose_lock, texture_cam_info_lock, texture_cam_image_lock, sci_cam_exif_lock;

  // Data indexed by timestamp
  std::map<double, Eigen::Affine3d> nav_cam_localization_poses;
  std::map<double, Eigen::Affine3d> nav_cam_landmark_poses;
  std::map<double, Eigen::Affine3d> texture_cam_poses;
  std::map<double, cv::Mat> texture_cam_images;
  std::map<double, std::vector<double> > sci_cam_exif;

  // For processing pictures as the camera moves along
  double dist_between_processed_cams, max_iso_times_exposure;
  Eigen::Vector3d last_processed_cam_ctr;
  std::vector<double> smallest_cost_per_face;
  int processed_camera_count;
  double num_exclude_boundary_pixels;

  // The info for each face which allows one later to project an image onto
  // the face and compute the texture.
  std::vector<dense_map::FaceInfo> face_projection_info;

  tex::Model texture_model;
  std::vector<dense_map::IsaacTextureAtlas::Ptr> texture_atlases;
  cv::Mat model_texture;

  // The pixel size to use to sample the image
  double pixel_size;

  // Number of threads to use
  int num_threads;

  std::shared_ptr<std::thread> thread;
};

StreamingMapper::StreamingMapper()
    : m_texture_cam_params(Eigen::Vector2i(0, 0), Eigen::Vector2d(0, 0), Eigen::Vector2d(0, 0)),
      m_navcam_to_texture_cam_timestamp_offset(0.0),
      m_texture_cam_to_navcam_trans(Eigen::MatrixXd::Identity(4, 4)),
      m_navcam_to_body_trans(Eigen::MatrixXd::Identity(4, 4)),
      last_processed_cam_ctr(Eigen::Vector3d(std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0)),
      processed_camera_count(0) {}

StreamingMapper::~StreamingMapper(void) { thread->join(); }

void StreamingMapper::Initialize(ros::NodeHandle& nh) {
  std::vector<std::string> args;
  bool main_thread = true;
  ff_common::InitFreeFlyerApplication(args, main_thread);

  // Set the config path to ISAAC
  char* path;
  if ((path = getenv("ISAAC_CONFIG_DIR")) == NULL) ROS_FATAL("Could not find the config path.");
  config_params.SetPath(path);

  config_params.AddFile("dense_map/streaming_mapper.config");
  if (!config_params.ReadFiles()) ROS_FATAL("Failed to read config files.");
  // Read the parameters defined in the launch file
  StreamingMapper::ReadParams(nh);

  // Read configuration data
  if (!sim_mode) {
    // Read the bot config file
    std::vector<std::string> cam_names = {"nav_cam", "haz_cam", "sci_cam"};
    std::vector<camera::CameraParameters> cam_params;
    std::vector<Eigen::Affine3d>          ref_to_cam_trans;
    std::vector<double>                   ref_to_cam_timestamp_offsets;
    Eigen::Affine3d                       hazcam_depth_to_image_transform;
    Eigen::Affine3d                       navcam_to_body_trans;

    dense_map::readConfigFile(  // Inputs
      cam_names, "nav_cam_transform", "haz_cam_depth_to_image_transform",
      // Outputs
      cam_params, ref_to_cam_trans, ref_to_cam_timestamp_offsets, navcam_to_body_trans,
      hazcam_depth_to_image_transform);

    m_navcam_to_body_trans = navcam_to_body_trans.matrix();

    {
      // Note the lock, because m_texture_cam_params is a shared resource
      const std::lock_guard<std::mutex> lock(texture_cam_info_lock);

      // Transform to convert from given camera to nav camera coordinates
      if (texture_cam_type == "nav_cam") {
        // Index 0 below, based on the order in cam_names
        m_texture_cam_to_navcam_trans = ref_to_cam_trans[0].inverse().matrix();
        m_navcam_to_texture_cam_timestamp_offset = ref_to_cam_timestamp_offsets[0];
        m_texture_cam_params = cam_params[0];
      } else if (texture_cam_type == "haz_cam") {
        // Index 1 below, based on the order in cam_names
        m_texture_cam_to_navcam_trans = ref_to_cam_trans[1].inverse().matrix();
        m_navcam_to_texture_cam_timestamp_offset = ref_to_cam_timestamp_offsets[1];
        m_texture_cam_params = cam_params[1];
      } else if (texture_cam_type == "sci_cam") {
        // Index 2 below, based on the order in cam_names
        m_texture_cam_to_navcam_trans = ref_to_cam_trans[2].inverse().matrix();
        m_navcam_to_texture_cam_timestamp_offset = ref_to_cam_timestamp_offsets[2];
        m_texture_cam_params = cam_params[2];
      } else {
        LOG(FATAL) << "Invalid texture cam type: " << texture_cam_type;
      }

      std::cout << "m_navcam_to_texture_cam_timestamp_offset: "
                << m_navcam_to_texture_cam_timestamp_offset << "\n";
      std::cout << "texture cam focal vector: "
                << m_texture_cam_params.GetFocalVector().transpose() << "\n";
    }
  }

  // Set up the publishers
  std::string mapper_img_topic = "/ism/" + texture_cam_type + "/img";
  std::string mapper_obj_topic = "/ism/" + texture_cam_type + "/obj";
  std::string mapper_mtl_topic = "/ism/" + texture_cam_type + "/mtl";
  ROS_INFO_STREAM("Publishing topics: " << mapper_img_topic << ' ' << mapper_obj_topic << ' ' << mapper_mtl_topic);
  texture_img_pub = nh.advertise<sensor_msgs::Image>(mapper_img_topic, 1);
  texture_obj_pub = nh.advertise<sensor_msgs::Image>(mapper_obj_topic, 1);
  texture_mtl_pub = nh.advertise<sensor_msgs::Image>(mapper_mtl_topic, 1);

  // Set up the subscribers
  if (sim_mode) {
    // Subscribe to the simulated texture cam image pose and intrinsics.
    // Keep a bunch of them in the queue.
    // The name of these topics are TOPIC_HAZ_CAM_SIM_POSE, etc., in ff_names.h.
    std::string texture_cam_sim_pose_topic = "/sim/" + texture_cam_type + "/pose";
    std::string texture_cam_sim_info_topic = "/sim/" + texture_cam_type + "/info";
    ROS_INFO_STREAM("Subscribing to  " << texture_cam_sim_pose_topic);
    ROS_INFO_STREAM("Subscribing to  " << texture_cam_sim_info_topic);
    texture_cam_pose_sub =
      nh.subscribe(texture_cam_sim_pose_topic, 10, &StreamingMapper::TextureCamSimPoseCallback, this);
    texture_cam_info_sub =
      nh.subscribe(texture_cam_sim_info_topic, 10, &StreamingMapper::TextureCamSimInfoCallback, this);
  } else {
    // Get the nav cam pose topic or the ekf state topic, or the ekf
    // pose topic, from which the other camera poses are deduced.
    if (!nav_cam_pose_topic.empty()) {
      ROS_INFO_STREAM("Subscribing to nav cam pose topic: " << nav_cam_pose_topic);
      landmark_sub = nh.subscribe(nav_cam_pose_topic, 10, &StreamingMapper::LandmarkCallback, this);
    } else if (!ekf_state_topic.empty()) {
      ROS_INFO_STREAM("Subscribing to ekf state topic: " << ekf_state_topic);
      ekf_state_sub = nh.subscribe(ekf_state_topic, 10, &StreamingMapper::EkfStateCallback, this);
    } else if (!ekf_pose_topic.empty()) {
      ROS_INFO_STREAM("Subscribing to ekf pose topic: " << ekf_pose_topic);
      ekf_pose_sub = nh.subscribe(ekf_pose_topic, 10, &StreamingMapper::EkfPoseCallback, this);
    }
  }

  sci_cam_exif_topic = "/hw/sci_cam_exif";
  ROS_INFO_STREAM("Subscribing to sci cam exif topic: " << sci_cam_exif_topic);
  sci_cam_exif_sub = nh.subscribe(sci_cam_exif_topic, 10, &StreamingMapper::SciCamExifCallback, this);

  // Subscribe to images. Keep just 2 in the queue as they can be big.
  image_transport.reset(new image_transport::ImageTransport(nh));

  std::string compressed = "/compressed";
  if (texture_cam_topic.length() >= compressed.length() &&
      texture_cam_topic.compare(texture_cam_topic.length() - compressed.length(), compressed.length(), compressed) ==
        0) {
    // texture_cam_topic ends with the string /compressed
    ROS_INFO_STREAM("Subscribing to compressed image topic: " << texture_cam_topic);
    compressed_texture_image_sub =
      nh.subscribe(texture_cam_topic, 2, &StreamingMapper::CompressedTextureCallback, this);
  } else {
    ROS_INFO_STREAM("Subscribing to uncompressed image topic: " << texture_cam_topic);
    uncompressed_texture_image_sub =
      image_transport->subscribe(texture_cam_topic, 2, &StreamingMapper::UncompressedTextureCallback, this);
  }

  // Load the mesh
  try {
    dense_map::loadMeshBuildTree(mesh_file, mesh, mesh_info, graph, bvh_tree);

    std::vector<unsigned int> const& faces = mesh->get_faces();
    int num_faces = faces.size();
    smallest_cost_per_face = std::vector<double>(num_faces, 1.0e+100);

    if (use_single_texture)
      dense_map::formModel(mesh, pixel_size, num_threads, face_projection_info, texture_atlases, texture_model);
  } catch (std::exception& e) {
    LOG(FATAL) << "Could not load mesh.\n" << e.what() << "\n";
  }

  // Start a new thread in which to do the processing
  thread.reset(new std::thread(&StreamingMapper::ProcessingLoop, this));
}

// TODO(oalexan1): Document all these.
void StreamingMapper::ReadParams(ros::NodeHandle const& nh) {
  if (!config_params.GetStr("nav_cam_pose_topic", &nav_cam_pose_topic))
    ROS_FATAL("Could not read the nav_cam_pose_topic parameter.");
  if (!config_params.GetStr("ekf_pose_topic", &ekf_pose_topic))
    ROS_FATAL("Could not read the ekf_pose_topic parameter.");
  if (!config_params.GetStr("ekf_state_topic", &ekf_state_topic))
    ROS_FATAL("Could not read the ekf_state_topic parameter.");
  if (!config_params.GetStr("texture_cam_topic", &texture_cam_topic))
    ROS_FATAL("Could not read the texture_cam_topic parameter.");
  if (!config_params.GetStr("texture_cam_type", &texture_cam_type))
    ROS_FATAL("Could not read the texture_cam_type parameter.");
  if (!config_params.GetStr("mesh_file", &mesh_file)) ROS_FATAL("Could not read the mesh_file parameter.");
  if (!config_params.GetReal("dist_between_processed_cams", &dist_between_processed_cams))
    ROS_FATAL("Could not read the dist_between_processed_cams parameter.");
  if (!config_params.GetReal("max_iso_times_exposure", &max_iso_times_exposure))
    ROS_FATAL("Could not read the max_iso_times_exposure parameter.");
  if (!config_params.GetReal("pixel_size", &pixel_size)) ROS_FATAL("Could not read the pixel_size parameter.");
  if (!config_params.GetReal("num_exclude_boundary_pixels", &num_exclude_boundary_pixels))
    ROS_FATAL("Could not read the num_exclude_boundary_pixels parameter.");
  if (!config_params.GetInt("num_threads", &num_threads)) ROS_FATAL("Could not read the num_threads parameter.");
  if (!nh.hasParam("sim_mode")) LOG(FATAL) << "sim_mode flag not specified.";

  // The sim_mode parameter is very important, it must be read early
  // on as a lot of logic depends on its value.
  nh.getParam("sim_mode", sim_mode);

  if (!config_params.GetBool("save_to_disk", &save_to_disk)) ROS_FATAL("Could not read the save_to_disk parameter.");

  if (!config_params.GetBool("use_single_texture", &use_single_texture))
    ROS_FATAL("Could not read the use_single_texture parameter.");

  ROS_INFO_STREAM("Texture camera type = " << texture_cam_type);
  ROS_INFO_STREAM("Mesh = " << mesh_file);
  if (!sim_mode) {
    int num = (!nav_cam_pose_topic.empty()) + (!ekf_state_topic.empty()) + (!ekf_pose_topic.empty());
    if (num != 1)
      LOG(FATAL) << "Must specify exactly only one of nav_cam_pose_topic, "
                 << "ekf_state_topic, ekf_pose_topic.";

    ROS_INFO_STREAM("dist_between_processed_cams = " << dist_between_processed_cams);
    ROS_INFO_STREAM("max_iso_times_exposure = " << max_iso_times_exposure);
    ROS_INFO_STREAM("use_single_texture = " << use_single_texture);
    ROS_INFO_STREAM("pixel_size = " << pixel_size);
    ROS_INFO_STREAM("num_threads = " << num_threads);
    ROS_INFO_STREAM("sim_mode = " << sim_mode);
    ROS_INFO_STREAM("save_to_disk = " << save_to_disk);
    ROS_INFO_STREAM("num_exclude_boundary_pixels = " << num_exclude_boundary_pixels);
    ROS_INFO_STREAM("ASTROBEE_ROBOT = " << getenv("ASTROBEE_ROBOT"));
  }

  if (texture_cam_type != "nav_cam" && texture_cam_type != "sci_cam" && texture_cam_type != "haz_cam" &&
      texture_cam_type != "heat_cam" && texture_cam_type != "acoustics_cam")
    LOG(FATAL) << "Invalid texture cam type: " << texture_cam_type;

  // For a camera like sci_cam, the word "sci" better be part of texture_cam_topic.
  std::string cam_name = texture_cam_type.substr(0, 3);
  if (texture_cam_topic.find(cam_name) == std::string::npos)
    LOG(FATAL) << "Texture topic " << texture_cam_topic << " is expected to contain the string " << cam_name;
}

void StreamingMapper::publishTexturedMesh(mve::TriangleMesh::ConstPtr mesh, std::shared_ptr<BVHTree> bvh_tree,
                                          double max_iso_times_exposure, double iso, double exposure,
                                          int processed_camera_count, cv::Mat const& image, double image_timestamp,
                                          camera::CameraModel const& cam, std::vector<double>& smallest_cost_per_face,
                                          std::string const& out_prefix) {
  omp_set_dynamic(0);                // Explicitly disable dynamic teams
  omp_set_num_threads(num_threads);  // Use this many threads for all consecutive
                                     // parallel regions

  // Colorize the image if grayscale. Careful here to avoid making a
  // copy if we don't need one.  Keep img_ptr pointing at the final
  // image.
  const cv::Mat* img_ptr = NULL;
  cv::Mat color_image;
  std::string mat_type = dense_map::matType(image);
  if (mat_type == "8UC3") {
    // Don't colorize
    img_ptr = &image;
  } else if (mat_type == "8UC1") {
    // Do colorize
    cv::cvtColor(image, color_image, cv::COLOR_GRAY2BGR);
    img_ptr = &color_image;
  } else {
    LOG(FATAL) << "Unknown image type: " << mat_type;
  }

  util::WallTimer timer1;

  // Exposure adjustment
  cv::Mat scaled_image;
  int same_type = -1;
  double offset = 0.0;
  if (iso > 0.0 && exposure > 0.0) {
    dense_map::exposureCorrection(max_iso_times_exposure, iso, exposure, *img_ptr, scaled_image);
    img_ptr = &scaled_image;
  }

  // std::cout << "Exposure correction took " << timer1.get_elapsed()/1000.0 <<
  // " seconds\n";

  // Prepare the image for publishing
  sensor_msgs::ImagePtr msg;
  std::vector<Eigen::Vector3i> face_vec;
  std::map<int, Eigen::Vector2d> uv_map;
  if (!use_single_texture) {
    // Find the UV coordinates and the faces having them
    dense_map::projectTexture(mesh, bvh_tree, *img_ptr, cam, num_exclude_boundary_pixels,
                              smallest_cost_per_face,
                              face_vec, uv_map);

    msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", *img_ptr).toImageMsg();
  } else {
    dense_map::projectTexture(mesh, bvh_tree, *img_ptr, cam, smallest_cost_per_face, pixel_size, num_threads,
                              face_projection_info, texture_atlases, texture_model, model_texture);
    // Note that the output image has an alpha channel
    msg = cv_bridge::CvImage(std_msgs::Header(), "bgra8", model_texture).toImageMsg();
  }

  // Publish the image, use the same timestamp as for the input image
  msg->header.stamp = ros::Time(image_timestamp);

  texture_img_pub.publish(msg);

  // Publish the obj string as a mono image with one row
  std::string obj_str;

  if (!use_single_texture || processed_camera_count == 0 || save_to_disk) {
    if (!use_single_texture)
      formObjCustomUV(mesh, face_vec, uv_map, out_prefix, obj_str);
    else
      formObj(texture_model, out_prefix, obj_str);

    int obj_len = obj_str.size();
    int width = std::sqrt(1.0 * obj_len);
    int height = obj_len / width + 1;
    if (width * height <= obj_len) LOG(FATAL) << "Not enough room allocated for the image.";

    m_texture_obj_msg.header.stamp = ros::Time(image_timestamp);
    m_texture_obj_msg.width = width;
    m_texture_obj_msg.height = height;
    m_texture_obj_msg.step = width;
    m_texture_obj_msg.encoding = "mono8";
    m_texture_obj_msg.is_bigendian = false;
    m_texture_obj_msg.data.resize(width * height);

    std::copy(reinterpret_cast<const char*>(&obj_str[0]),            // input beg
              reinterpret_cast<const char*>(&obj_str[0]) + obj_len,  // input end
              m_texture_obj_msg.data.begin());                         // destination
    // Pad with nulls
    for (int it = obj_len; it < width * height; it++) m_texture_obj_msg.data[it] = '\0';

    // When using a single texture, publish just once
    if (!use_single_texture || processed_camera_count == 0)
      texture_obj_pub.publish(m_texture_obj_msg);
  }

  // Publish the mtl string as a mono image with one row (this hack is temporary)
  std::string mtl_str;
  if (!use_single_texture || processed_camera_count == 0 || save_to_disk) {
    formMtl(out_prefix, mtl_str);
    int mtl_len = mtl_str.size();
    m_texture_mtl_msg.header.stamp = ros::Time(image_timestamp);
    m_texture_mtl_msg.width = mtl_len;
    m_texture_mtl_msg.height = 1;
    m_texture_mtl_msg.step = mtl_len;
    m_texture_mtl_msg.encoding = "mono8";
    m_texture_mtl_msg.is_bigendian = false;
    m_texture_mtl_msg.data.resize(mtl_len);
    std::copy(reinterpret_cast<const char*>(&mtl_str[0]),            // input beg
              reinterpret_cast<const char*>(&mtl_str[0]) + mtl_len,  // input end
              m_texture_mtl_msg.data.begin());                         // destination

    if (!use_single_texture || processed_camera_count == 0)
      texture_mtl_pub.publish(m_texture_mtl_msg);
  }

  if (save_to_disk) {
    std::string obj_file = out_prefix + ".obj";
    std::cout << "Writing: " << obj_file << std::endl;
    std::ofstream obj_handle(obj_file);
    obj_handle << obj_str;
    obj_handle.close();

    std::string mtl_file = out_prefix + ".mtl";
    std::cout << "Writing: " << mtl_file << std::endl;
    std::ofstream mtl_handle(mtl_file);
    mtl_handle << mtl_str;
    mtl_handle.close();

    std::string png_file = out_prefix + ".png";
    std::cout << "Writing: " << png_file << std::endl;
    if (!use_single_texture)
      cv::imwrite(png_file, *img_ptr);
    else
      cv::imwrite(png_file, model_texture);
  }
}

// Add the latest received texture cam pose to the storage. This is meant to be used
// only for simulated data. Else use the landmark and ekf callbacks.
void StreamingMapper::TextureCamSimPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& pose) {
  if (!sim_mode) return;

  // Add the latest pose. Use a lock.
  const std::lock_guard<std::mutex> lock(texture_cam_pose_lock);

  // Convert the pose to Eigen::Affine3d
  Eigen::Affine3d texture_cam_pose;
  tf::poseMsgToEigen(pose->pose, texture_cam_pose);
  texture_cam_poses[pose->header.stamp.toSec()] = texture_cam_pose;

  // std::cout << "Received the sim texture cam pose at time: " << pose->header.stamp.toSec()
  // << "\n";

  // Wipe the oldest poses. Keep a lot of them as sometimes they can come very often.
  while (texture_cam_poses.size() > 100) {
    texture_cam_poses.erase(texture_cam_poses.begin());
  }
}

void StreamingMapper::TextureCamSimInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& info) {
  if (!sim_mode) return;

  // Initialize m_texture_cam_params just once. Use a lock.
  if (m_texture_cam_params.GetFocalVector() == Eigen::Vector2d(0, 0)) {
    const std::lock_guard<std::mutex> lock(texture_cam_info_lock);

    int image_width = info->width;
    int image_height = info->height;
    double focal_length = info->K[0];
    double optical_center_x = info->K[2];
    double optical_center_y = info->K[5];

    // We support only fish eye lenses (one distortion parameter), or
    // lenses with no distortion (zero distortion parameters).  Note:
    // The fisheye distortion model in freeflyer does not seem to
    // agree to well with the wide angle camera distorted images
    // output by gazebo. Hence it is better to use images with no
    // distortion in the simulated images.
    Eigen::VectorXd distortion;
    if (info->D[0] != 0) {
      distortion.resize(1);
      distortion[0] = info->D[0];
    }

    // std::cout << "Received the camera info at time: " << info->header.stamp.toSec() << "\n";
    m_texture_cam_params =
      camera::CameraParameters(Eigen::Vector2i(image_width, image_height), Eigen::Vector2d(focal_length, focal_length),
                               Eigen::Vector2d(optical_center_x, optical_center_y), distortion);
  }
}

// Compute the texture cam pose and timestamp based on the nav cam pose
// and timestamp, and save it in the storage.
void StreamingMapper::AddTextureCamPose(double nav_cam_timestamp, Eigen::Affine3d const& nav_cam_pose) {
  if (nav_cam_pose.matrix() == Eigen::Matrix<double, 4, 4>::Identity()) {
    // Skip bad poses
    return;
  }

  // Must compensate for the fact that the nav cam, haz cam, and texture cam all
  // have some time delay among them
  double texture_cam_timestamp = nav_cam_timestamp + m_navcam_to_texture_cam_timestamp_offset;

  // Keep in mind that nav_cam_pose is the transform from the nav cam
  // to the world.  Hence the matrices are multiplied in the order as
  // below.
  Eigen::Affine3d texture_cam_pose;
  texture_cam_pose.matrix() = nav_cam_pose.matrix() * m_texture_cam_to_navcam_trans;
  {
    // Add the latest pose. Use a lock.
    const std::lock_guard<std::mutex> lock(texture_cam_pose_lock);
    texture_cam_poses[texture_cam_timestamp] = texture_cam_pose;

    // Wipe the oldest poses. Keep a lot of them, as sometimes, particularly for ekf,
    // they can come very fast
    while (texture_cam_poses.size() > 100) texture_cam_poses.erase(texture_cam_poses.begin());
  }
  // TODO(oalexan1): This will need more testing!
}

// This callback takes as input the nav_cam pose as output by localization_node.
void StreamingMapper::LandmarkCallback(ff_msgs::VisualLandmarks::ConstPtr const& vl) {
  if (sim_mode) return;

  double nav_cam_timestamp = vl->header.stamp.toSec();
  Eigen::Affine3d nav_cam_pose;
  tf::poseMsgToEigen(vl->pose, nav_cam_pose);

  StreamingMapper::AddTextureCamPose(nav_cam_timestamp, nav_cam_pose);
}

// This callback takes as input the robot pose as output by ekf on
// ekf_state_topic (it should be set to /gnc/ekf)
void StreamingMapper::EkfStateCallback(ff_msgs::EkfState::ConstPtr const& ekf_state) {
  if (sim_mode) return;

  double nav_cam_timestamp = ekf_state->header.stamp.toSec();
  Eigen::Affine3d body_pose;
  tf::poseMsgToEigen(ekf_state->pose, body_pose);

  // Convert from body pose (body to world transform) to nav cam pose (nav cam to world transform)
  // Use the equation: body_to_world = navcam_to_world * body_to_navcam, written equivalently as:
  // navcam_to_world = body_to_world * navcam_to_body
  Eigen::Affine3d nav_cam_pose;
  nav_cam_pose.matrix() = body_pose.matrix() * m_navcam_to_body_trans;
  StreamingMapper::AddTextureCamPose(nav_cam_timestamp, nav_cam_pose);
}

// This callback takes as input the robot pose as output by ekf on /loc/pose.
void StreamingMapper::EkfPoseCallback(geometry_msgs::PoseStamped::ConstPtr const& ekf_pose) {
  if (sim_mode) return;

  double nav_cam_timestamp = ekf_pose->header.stamp.toSec();
  Eigen::Affine3d body_pose;
  tf::poseMsgToEigen(ekf_pose->pose, body_pose);

  // Convert from body pose (body to world transform) to nav cam pose (nav cam to world transform)
  // Use the equation: body_to_world = navcam_to_world * body_to_navcam, written equivalently as:
  // navcam_to_world = body_to_world * navcam_to_body
  Eigen::Affine3d nav_cam_pose;
  nav_cam_pose.matrix() = body_pose.matrix() * m_navcam_to_body_trans;
  StreamingMapper::AddTextureCamPose(nav_cam_timestamp, nav_cam_pose);
}

// This callback takes as input the robot pose as output by ekf on /loc/pose.
void StreamingMapper::SciCamExifCallback(std_msgs::Float64MultiArray::ConstPtr const& exif) {
  if (sim_mode) return;

  double timestamp = exif->data[dense_map::TIMESTAMP];

  std::vector<double> exif_vec;
  for (int it = 0; it < dense_map::NUM_EXIF; it++) {
    exif_vec.push_back(exif->data[it]);
  }

  {
    // Use a lock to modify shared data
    const std::lock_guard<std::mutex> lock(sci_cam_exif_lock);
    sci_cam_exif[timestamp] = exif_vec;

    // Wipe the oldest exif data
    while (sci_cam_exif.size() > 10) {
      sci_cam_exif.erase(sci_cam_exif.begin());
    }
  }
}

void StreamingMapper::WipeOldImages() {
  // Wipe the oldest images. Keep just a few.
  size_t num_to_keep = 10;  // for sci_cam
  if (texture_cam_type == "nav_cam") {
    // For small images that get published often keep a bunch of them,
    // as sometimes there is a delay for when the pose arrives,
    // and if not enough images are kept they are wiped before
    // they can be used.
    // TODO(oalexan1): This is not robust enough.
    num_to_keep = 50;
  }
  if (texture_cam_type == "haz_cam") {
    num_to_keep = 100;
  }

  while (texture_cam_images.size() > num_to_keep) texture_cam_images.erase(texture_cam_images.begin());

  // There is a second wiping in this code in a different place. After
  // an image is processed it will be wiped together with any images
  // older than it. So the real number of images is not as many as it
  // looks here.
}

void StreamingMapper::CompressedTextureCallback(const sensor_msgs::CompressedImageConstPtr& msg) {
  // Use a lock to ensure no shared data is messed up.
  const std::lock_guard<std::mutex> lock(texture_cam_image_lock);

  double texture_cam_image_timestamp = msg->header.stamp.toSec();
  try {
    // Copy the data to local storage to ensure no strange behavior
    //  cv::Mat tmp_img = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);
    //  cv::Mat tmp_img =
    texture_cam_images[texture_cam_image_timestamp] = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);

    //  cv::namedWindow( "Display window", cv::WINDOW_AUTOSIZE );  // Create a window for display.
    //  cv::imshow( "Display window", tmp_img);                   // Show our image inside it.
    //  cv::waitKey(0);

    //  texture_cam_images[texture_cam_image_timestamp] = tmp_img;
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("Could not decompress image!");
  }

  WipeOldImages();
}

// Record the texture cam image received
void StreamingMapper::UncompressedTextureCallback(const sensor_msgs::ImageConstPtr& msg) {
  // Use a lock to ensure no shared data is messed up.
  const std::lock_guard<std::mutex> lock(texture_cam_image_lock);

  double texture_cam_image_timestamp = msg->header.stamp.toSec();

  // std::cout << "Received image at time " << texture_cam_image_timestamp << std::endl;

  cv_bridge::CvImageConstPtr image_ptr;
  try {
    image_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  // Copy the data to local storage to ensure no strange behavior
  (image_ptr->image).copyTo(texture_cam_images[texture_cam_image_timestamp]);

  WipeOldImages();
}

// Iterate over the texture images. Overlay them on the 3D model and publish
// the result.
// Need to consider here if they are
// full res and compressed and color, or 1/4 res and grayscale and
// and uncompressed
//
void StreamingMapper::ProcessingLoop() {
  double last_attempted_texture_cam_timestamp = -1.0;

  bool success = false;

  while (ros::ok()) {
    if (!success) {
      // Sleep for a little if we had no luck so far
      double num_seconds = 0.2;
      usleep(static_cast<int>(1e+6 * num_seconds));
    } else {
      // We were just successful in processing. That should have taken a while,
      // so no need to sleep, and check for new data right away.
      success = false;
    }

    if (m_texture_cam_params.GetFocalVector() == Eigen::Vector2d(0, 0)) {
      // if the texture cam is not initialized yet, then wait
      continue;
    }

    // Safely copy the data we need (this data is small, so copying is cheap).

    std::map<double, Eigen::Affine3d> local_texture_cam_poses;
    {
      const std::lock_guard<std::mutex> lock(texture_cam_pose_lock);
      local_texture_cam_poses = texture_cam_poses;
    }

    camera::CameraParameters local_texture_cam_params(Eigen::Vector2i(0, 0), Eigen::Vector2d(0, 0),
                                                      Eigen::Vector2d(0, 0));
    {
      const std::lock_guard<std::mutex> lock(texture_cam_info_lock);
      local_texture_cam_params = m_texture_cam_params;
    }

    // Copy the exif data locally using a lock
    bool need_exif = (!sim_mode && texture_cam_type == "sci_cam");

    std::map<double, std::vector<double> > local_sci_cam_exif;
    if (need_exif) {
      const std::lock_guard<std::mutex> lock(sci_cam_exif_lock);
      local_sci_cam_exif = sci_cam_exif;
    }

    // TODO(oalexan1): Make the exif topic optional?

    double texture_cam_image_timestamp = -1.0;
    cv::Mat texture_cam_image;
    {
      // Iterate over the texture cam images we have, from newest to
      // oldest, and see for which of them we have enough data to
      // build the texture.
      const std::lock_guard<std::mutex> lock(texture_cam_image_lock);

      for (auto it = texture_cam_images.rbegin(); it != texture_cam_images.rend(); it++) {
        texture_cam_image_timestamp = it->first;

        // Do not do images older than the image we did previously
        if (texture_cam_image_timestamp <= last_attempted_texture_cam_timestamp) {
          break;
        }

        if (local_texture_cam_poses.empty()) {
          break;  // no poses yet
        }

        // Ensure that we can bracket the texture image timestamp between
        // the timestamps of the poses
        if (texture_cam_image_timestamp > local_texture_cam_poses.rbegin()->first) {
          continue;
        }
        if (texture_cam_image_timestamp < local_texture_cam_poses.begin()->first) {
          continue;
        }

        if (need_exif &&
            local_sci_cam_exif.find(texture_cam_image_timestamp) == local_sci_cam_exif.end()) {
          // Skip if the exif info did not arrive yet
          continue;
        }

        // While copying could be expensive, how else can we
        // guaranteed that our data is still there when we need it,
        // unless we use a lock throughout?
        (it->second).copyTo(texture_cam_image);

        success = true;

        // Wipe this image and images older than this as we will never use them
        while (texture_cam_images.size() > 0 &&
               texture_cam_images.begin()->first <= texture_cam_image_timestamp)
          texture_cam_images.erase(texture_cam_images.begin());

        // Stop given that we have found a good image to process
        break;
      }
    }  // end of lock

    if (!success) {
      continue;
    }

    util::WallTimer timer;

    // All is good with this timestamp. We may still not want to process it,
    // if the camera is too close now to the previously processed camera,
    // but at least mark it as good, so we won't have to examine it again.
    last_attempted_texture_cam_timestamp = texture_cam_image_timestamp;

    // Find the interpolated pose at the current image
    Eigen::Affine3d curr_texture_cam_pose;

    if (!dense_map::findInterpPose(texture_cam_image_timestamp, local_texture_cam_poses,
                                   curr_texture_cam_pose))
      LOG(FATAL) << "Could not bracket the timestamp. Should have worked at this stage.";

    camera::CameraModel cam(curr_texture_cam_pose.inverse(), local_texture_cam_params);

    Eigen::Vector3d cam_ctr = cam.GetPosition();
    double dist_to_prev_processed = (last_processed_cam_ctr - cam_ctr).norm();
    bool do_process = (std::isnan(dist_to_prev_processed) ||  // for the first camera to process
                       dist_to_prev_processed > dist_between_processed_cams);

    //  if (!std::isnan(dist_to_prev_processed))
    //    ROS_INFO_STREAM("Distance from current camera to last processed camera: "
    //                  << dist_to_prev_processed << " meters.");

    // Current camera is too close to previously processed camera, hence wait for the bot to move
    if (!do_process) {
      // ROS_INFO_STREAM("Won't process this camera image, as it is too close to previous one.");
      continue;
    }

    // May need to keep this comment long term
    // std::cout << "Processing the streaming camera image at time stamp "
    //           << texture_cam_image_timestamp << std::endl;

    double iso = -1.0, exposure = -1.0;
    if (need_exif) {
      auto exif_it = local_sci_cam_exif.find(texture_cam_image_timestamp);
      if (exif_it == local_sci_cam_exif.end()) {
        LOG(FATAL) << "Could not find the exif data.";
      }
      std::vector<double> const& exif = exif_it->second;
      iso = exif[dense_map::ISO];
      exposure = exif[dense_map::EXPOSURE_TIME];
    }

    // Publish the textured mesh in obj format
    std::ostringstream oss;
    oss << "processed_" << 1000 + processed_camera_count;  // add 1000 to list them nicely
    std::string out_prefix = oss.str();
    publishTexturedMesh(mesh, bvh_tree, max_iso_times_exposure, iso, exposure,
                        processed_camera_count, texture_cam_image, texture_cam_image_timestamp,
                        cam, smallest_cost_per_face, out_prefix);

    // Save this for next time
    last_processed_cam_ctr = cam_ctr;
    processed_camera_count++;

    std::cout << "Total processing took " << timer.get_elapsed() / 1000.0 << " seconds\n";
  }
}

}  // end of namespace dense_map

int main(int argc, char** argv) {
  ros::init(argc, argv, "streaming_mapper");

  std::cout.precision(17);  // to be able to print timestamps accurately

  ros::NodeHandle nh("~");
  dense_map::StreamingMapper sm;
  sm.Initialize(nh);

  ros::spin();

  return 0;
}
