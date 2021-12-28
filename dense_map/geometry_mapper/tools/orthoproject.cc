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

#include <ff_common/init.h>
#include <ff_common/thread.h>
#include <ff_common/utils.h>
#include <config_reader/config_reader.h>
#include <camera/camera_params.h>
#include <camera/camera_model.h>
#include <dense_map_utils.h>
#include <texture_processing.h>

#include <boost/filesystem.hpp>

#include <glog/logging.h>
#include <opencv2/imgcodecs.hpp>

#include <cmath>

// Project a given image and camera onto a given mesh, and save an .obj file

DEFINE_string(mesh, "", "The mesh to project onto.");

DEFINE_string(image, "", "The image to orthoproject. It should be distorted, as extracted "
              "from a bag.");

DEFINE_string(camera_to_world, "",
              "The camera to world file, as written by the geometry mapper. For example: "
              "run/1616779788.2920296_nav_cam_to_world.txt.");

DEFINE_string(camera_name, "", "The the camera name. For example: 'sci_cam'.");

DEFINE_string(output_dir, "", "The output texture directory. The texture name will be "
              "<output_dir>/run.obj.");

int main(int argc, char** argv) {
  ff_common::InitFreeFlyerApplication(&argc, &argv);

  if (FLAGS_mesh.empty() || FLAGS_image.empty() || FLAGS_camera_to_world.empty() ||
      FLAGS_camera_name.empty() || FLAGS_output_dir.empty())
    LOG(FATAL) << "Not all inputs were specified.";

  if (!boost::filesystem::exists(FLAGS_output_dir))
    if (!boost::filesystem::create_directories(FLAGS_output_dir) ||
        !boost::filesystem::is_directory(FLAGS_output_dir))
      LOG(FATAL) << "Failed to create directory: " << FLAGS_output_dir;

  // Load the mesh
  mve::TriangleMesh::Ptr mesh;
  std::shared_ptr<mve::MeshInfo> mesh_info;
  std::shared_ptr<tex::Graph> graph;
  std::shared_ptr<BVHTree> bvh_tree;
  dense_map::loadMeshBuildTree(FLAGS_mesh, mesh, mesh_info, graph, bvh_tree);

  std::cout << "Reading image: " << FLAGS_image << std::endl;
  cv::Mat image = cv::imread(FLAGS_image);

  // Load the camera intrinsics
  config_reader::ConfigReader config;
  config.AddFile("cameras.config");
  if (!config.ReadFiles()) LOG(FATAL) << "Could not read the configuration file.";
  std::cout << "Using camera: " << FLAGS_camera_name << std::endl;
  camera::CameraParameters cam_params(&config, FLAGS_camera_name.c_str());

  std::cout << "Reading pose: " << FLAGS_camera_to_world << std::endl;
  Eigen::Affine3d cam_to_world;
  dense_map::readAffine(cam_to_world, FLAGS_camera_to_world);
  std::cout << "Read pose\n" << cam_to_world.matrix() << std::endl;

  std::vector<Eigen::Vector3i> face_vec;
  std::map<int, Eigen::Vector2d> uv_map;
  int num_exclude_boundary_pixels = 0;

  std::vector<unsigned int> const& faces = mesh->get_faces();
  int num_faces = faces.size();
  std::vector<double> smallest_cost_per_face(num_faces, 1.0e+100);

  camera::CameraModel cam(cam_to_world.inverse(), cam_params);

  // Find the UV coordinates and the faces having them
  std::string out_prefix = "run";
  dense_map::projectTexture(mesh, bvh_tree, image, cam, num_exclude_boundary_pixels,
                            smallest_cost_per_face, face_vec, uv_map);

  std::string obj_str;
  dense_map::formObjCustomUV(mesh, face_vec, uv_map, out_prefix, obj_str);

  std::string mtl_str;
  dense_map::formMtl(out_prefix, mtl_str);

  std::string obj_file = FLAGS_output_dir + "/" + out_prefix + ".obj";
  std::cout << "Writing: " << obj_file << std::endl;
  std::ofstream obj_handle(obj_file);
  obj_handle << obj_str;
  obj_handle.close();

  std::string mtl_file = FLAGS_output_dir + "/" + out_prefix + ".mtl";
  std::cout << "Writing: " << mtl_file << std::endl;
  std::ofstream mtl_handle(mtl_file);
  mtl_handle << mtl_str;
  mtl_handle.close();

  std::string texture_file = FLAGS_output_dir + "/" + out_prefix + ".png";
  std::cout << "Writing: " << texture_file << std::endl;
  cv::imwrite(texture_file, image);

  return 0;
}
