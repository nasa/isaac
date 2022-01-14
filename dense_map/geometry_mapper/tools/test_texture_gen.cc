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

#include <gflags/gflags.h>
#include <glog/logging.h>

#include <cmath>

// Test creating a texture the way the streaming_mapper does it, but
// without involving ROS. Hence, create a single shared texture buffer
// which will be used for any image, then use it with one image. This
// is different than the orthoproject tool, which creates a texture
// buffer specific to each image with no reserve space.

DEFINE_string(mesh, "", "The mesh to project onto.");

DEFINE_string(image, "", "The image to orthoproject. It should be distorted, as extracted "
              "from a bag.");

DEFINE_string(camera_to_world, "",
              "The camera to world file, as written by the geometry mapper. For example: "
              "run/1616779788.2920296_nav_cam_to_world.txt.");

DEFINE_string(camera_name, "", "The the camera name. For example: 'sci_cam'.");

DEFINE_int32(num_exclude_boundary_pixels, 0,
             "Exclude pixels closer to the boundary than this when texturing.");

DEFINE_string(output_prefix, "", "The output prefix. The textured mesh name will be"
              " <output_prefix>.obj.");

int main(int argc, char** argv) {
  ff_common::InitFreeFlyerApplication(&argc, &argv);

  if (FLAGS_image.empty() || FLAGS_camera_to_world.empty())
    LOG(FATAL) << "Must specify an image and camera.";

  if (FLAGS_mesh.empty() || FLAGS_camera_name.empty() || FLAGS_output_prefix.empty())
    LOG(FATAL) << "Not all inputs were specified.";

  // Load the mesh
  mve::TriangleMesh::Ptr mesh;
  std::shared_ptr<mve::MeshInfo> mesh_info;
  std::shared_ptr<tex::Graph> graph;
  std::shared_ptr<BVHTree> bvh_tree;
  dense_map::loadMeshBuildTree(FLAGS_mesh, mesh, mesh_info, graph, bvh_tree);

  // Load the camera intrinsics
  config_reader::ConfigReader config;
  config.AddFile("cameras.config");
  if (!config.ReadFiles()) LOG(FATAL) << "Could not read the configuration file.";
  std::cout << "Using camera: " << FLAGS_camera_name << std::endl;
  camera::CameraParameters cam_params(&config, FLAGS_camera_name.c_str());

  // Read image and camera_to_world
  cv::Mat image = cv::imread(FLAGS_image);
  Eigen::Affine3d cam_to_world;
  dense_map::readAffine(cam_to_world, FLAGS_camera_to_world);

  double pixel_size = 0.001;
  int num_threads = 48;
  std::vector<unsigned int> const& faces = mesh->get_faces();
  int num_faces = faces.size();
  std::vector<double> smallest_cost_per_face(num_faces, 1.0e+100);
  std::vector<dense_map::FaceInfo> face_projection_info;
  dense_map::IsaacObjModel texture_model;
  std::vector<dense_map::IsaacTextureAtlas::Ptr> texture_atlases;
  cv::Mat model_texture;

  // Form the model
  dense_map::formModel(mesh, pixel_size, num_threads, face_projection_info,
                       texture_atlases, texture_model);

  // Set up the camera
  camera::CameraModel cam(cam_to_world.inverse(), cam_params);

  // Project a single image
  dense_map::projectTexture(mesh, bvh_tree, image, cam, smallest_cost_per_face,
                            pixel_size, num_threads, face_projection_info, texture_atlases,
                            texture_model, model_texture);
  std::string mtl_str;
  dense_map::formMtl(FLAGS_output_prefix, mtl_str);
  std::string obj_str;
  dense_map::formObj(texture_model, FLAGS_output_prefix, obj_str);

  // Create the output directory, if needed
  std::string out_dir = boost::filesystem::path(FLAGS_output_prefix).parent_path().string();
  if (out_dir != "") dense_map::createDir(out_dir);

  std::string obj_file = FLAGS_output_prefix + ".obj";
  std::cout << "Writing: " << obj_file << std::endl;
  std::ofstream obj_handle(obj_file);
  obj_handle << obj_str;
  obj_handle.close();

  std::string mtl_file = FLAGS_output_prefix + ".mtl";
  std::cout << "Writing: " << mtl_file << std::endl;
  std::ofstream mtl_handle(mtl_file);
  mtl_handle << mtl_str;
  mtl_handle.close();

  std::string png_file = FLAGS_output_prefix + ".png";
  std::cout << "Writing: " << png_file << std::endl;
  cv::imwrite(png_file, model_texture);

  return 0;
}
