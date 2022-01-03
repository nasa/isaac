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

DEFINE_string(image_list, "", "The list of images to orthoproject, one per line, "
              "unless specified individually as above.");

DEFINE_string(camera_list, "", "The list of cameras to world transforms use, one per line, "
              "unless specified individually as above.");

DEFINE_int32(num_exclude_boundary_pixels, 0,
             "Exclude pixels closer to the boundary than this when texturing.");

DEFINE_string(output_prefix, "", "The output prefix. The texture name will be"
              " <output_prefix>-<image base name without extension>.obj.");

int main(int argc, char** argv) {
  ff_common::InitFreeFlyerApplication(&argc, &argv);

  if ((FLAGS_image.empty() || FLAGS_camera_to_world.empty()) &&
      (FLAGS_image_list.empty() || FLAGS_camera_list.empty()))
    LOG(FATAL) << "Must specify either an image and camera, or an image list and camera list.";

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

  std::vector<std::string> images, cameras;
  if (!FLAGS_image.empty() && !FLAGS_camera_to_world.empty()) {
    images.push_back(FLAGS_image);
    cameras.push_back(FLAGS_camera_to_world);
  } else {
    std::string val;
    std::ifstream ifs(FLAGS_image_list);
    while (ifs >> val)
      images.push_back(val);
    ifs.close();
    ifs = std::ifstream(FLAGS_camera_list);
    while (ifs >> val)
      cameras.push_back(val);
    ifs.close();
  }

  if (images.size() != cameras.size())
    LOG(FATAL) << "As many images as cameras must be specified.\n";

  for (size_t it = 0; it < images.size(); it++) {
    cv::Mat image = cv::imread(images[it]);

    Eigen::Affine3d cam_to_world;
    dense_map::readAffine(cam_to_world, cameras[it]);

    std::string prefix = FLAGS_output_prefix + "-" +
      boost::filesystem::path(images[it]).stem().string();

    dense_map::meshProject(mesh, bvh_tree, image, cam_to_world.inverse(), cam_params,
                           FLAGS_num_exclude_boundary_pixels, prefix);
  }

  return 0;
}
