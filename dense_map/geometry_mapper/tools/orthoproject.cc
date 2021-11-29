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

#if 0

// Old broken code, project onto a plane. See current working code further down.

DEFINE_string(image, "",
              "The image to orthoproject. For each image named image.jpg, "
              "an input file image_world2cam.txt is expected, and the output "
              "orthoimage will be written to image_ortho.png");

DEFINE_string(image_list, "",
              "Instead of a single image, specify the list of images to orthoproject, "
              "with the same convention as above. ");

DEFINE_double(min_ray_length, 0.0,
              "The shortest distance from the camera to the mesh.");

DEFINE_double(max_ray_length, 10.0,
              "The longest distance from the camera to the mesh.");

DEFINE_double(pixel_size, 0.01,
              "The output orthoimage pixel size, in meters.");

DEFINE_bool(snap_plane, false,
            "Adjust the plane to project onto to make "
            "angles multiple of 45 degrees with the coordinate axes.");

DEFINE_string(plane_normal, "",
              "Override the auto-computed the normal of the plane to "
              "project onto with this. Specify as three values in quotes.");

// Shoot a set of rays from the camera onto the mesh and see
// where they intersect
void sample_mesh(cv::Mat const& image,
                 camera::CameraModel const& cam,
                 std::shared_ptr<BVHTree> const& bvh_tree,
                 std::vector<Eigen::Vector3d> & intersections) {
  // Wipe the output
  intersections.clear();

  Eigen::Vector3d cam_ctr = -cam.GetTransform().rotation().transpose() *
    cam.GetTransform().translation();

  // Sample the image uniformly
  int num_samples = 100;
  Eigen::Matrix3d cam_to_world_rot = cam.GetTransform().rotation().transpose();
#pragma omp parallel for
  for (int itx = 0; itx < num_samples; itx++) {
    for (int ity = 0; ity < num_samples; ity++) {
      double x = (image.cols - 1.0) * static_cast<double>(itx) / (num_samples - 1.0);
      double y = (image.rows - 1.0) * static_cast<double>(ity) / (num_samples - 1.0);

      Eigen::Vector2d dist_pix;
      dist_pix << x, y;

      Eigen::Vector2d undist_centered_pix;
      cam.GetParameters().Convert<camera::DISTORTED, camera::UNDISTORTED_C>
        (dist_pix, &undist_centered_pix);

      // Ray from camera going through the pixel
      Eigen::Vector3d cam_ray(undist_centered_pix.x() / cam.GetParameters().GetFocalVector()[0],
                              undist_centered_pix.y() / cam.GetParameters().GetFocalVector()[1],
                              1.0);
      cam_ray.normalize();

      Eigen::Vector3d world_ray = cam_to_world_rot * cam_ray;

      // Set up the ray structure for the mesh
      BVHTree::Ray bvh_ray;
      bvh_ray.origin = dense_map::eigen_to_vec3f(cam_ctr);
      bvh_ray.dir = dense_map::eigen_to_vec3f(world_ray);
      bvh_ray.dir.normalize();
      bvh_ray.tmin = FLAGS_min_ray_length;
      bvh_ray.tmax = FLAGS_max_ray_length;

      // Intersect the ray with the mesh
      BVHTree::Hit hit;
      if (!bvh_tree->intersect(bvh_ray, &hit))
        continue;
      double cam_to_mesh_dist = hit.t;

      Eigen::Vector3d intersection = cam_ctr + cam_to_mesh_dist * world_ray;

#pragma omp critical
      {
        intersections.push_back(intersection);
      }
    }
  }

  if (intersections.empty())
    LOG(FATAL) << "Could not project the image onto the mesh.\n";
}

// Create a new coordinate system having the desired plane as its x-y
// plane, and the normal to the plane as its z axis.
void setup_plane_coord_system(camera::CameraModel const& cam,
                              Eigen::Vector3d & plane_normal,
                              Eigen::Matrix3d & world2planecoord) {
  // The camera view (the ray starting at the camera center and going through
  // the image center).
  Eigen::Matrix3d cam_to_world_rot = cam.GetTransform().rotation().transpose();
  Eigen::Vector3d cam_dir = cam_to_world_rot * Eigen::Vector3d(0, 0, 1);

  // Ensure that the normal to the plane agrees with the camera direction
  double dot_prod = cam_dir.dot(plane_normal);
  if (dot_prod < 0)
    plane_normal *= -1.0;

  // The "up" direction
  Eigen::Vector3d z(0, 0, 1);

  Eigen::Vector3d lz = plane_normal.normalized();
  // The new y points "down", as suggested by the z axis, while still
  // being in the plane
  Eigen::Vector3d ly = -1.0 * (z - z.dot(plane_normal) * plane_normal);
  // Treat the special case when the plane is perfectly horizontal,
  // then the normal can't point down. Normalize.
  if (ly.norm() == 0)
    ly = Eigen::Vector3d(0, 1, 0);
  ly.normalize();
  // The new x is then the remaining direction so that we have a
  // right-handed coordinate system.
  Eigen::Vector3d lx = ly.cross(lz);
  if (lx.norm() == 0) {
    lx = Eigen::Vector3d(1, 0, 0);  // should never happen
    ly = lz.cross(lx);
  }
  lx.normalize();

  // The matrix to transform to the coordinate system having the
  // desired plane to project onto as its x-y plane.
  Eigen::Matrix3d planecoord2world(3, 3);
  planecoord2world.col(0) = lx;
  planecoord2world.col(1) = ly;
  planecoord2world.col(2) = lz;

  world2planecoord = planecoord2world.transpose();
}

void orthoproject(cv::Mat const& image,
                  Eigen::Matrix3d const& world2planecoord,
                  Eigen::Vector3d const& plane_normal,
                  camera::CameraModel const& cam,
                  std::shared_ptr<BVHTree> const& bvh_tree,
                  std::vector<Eigen::Vector3d> const& intersections,
                  double pixel_size,
                  cv::Mat & orthoimage) {
  // Find the bounding box of the mesh points seen by the camera
  // in the coordinate system of the desired plane

  double min_x = std::numeric_limits<double>::max(), max_x = -min_x;
  double min_y = min_x, max_y = max_x;
  double min_z = min_x, max_z = max_x;
  for (size_t it = 0; it < intersections.size(); it++) {
    Eigen::Vector3d local_pt = world2planecoord * intersections[it];
    if (local_pt.x() < min_x) min_x = local_pt.x();
    if (local_pt.x() > max_x) max_x = local_pt.x();
    if (local_pt.y() < min_y) min_y = local_pt.y();
    if (local_pt.y() > max_y) max_y = local_pt.y();
    if (local_pt.z() < min_z) min_z = local_pt.z();
    if (local_pt.z() > max_z) max_z = local_pt.z();
  }

  if (min_x > max_x || min_y > max_y || min_z > max_z)
    LOG(FATAL) << "The rays don't intersect the mesh.";

  // The output image dimensions
  int num_cols = ceil((max_x - min_x)/pixel_size);
  int num_rows = ceil((max_y - min_y)/pixel_size);

  orthoimage = cv::Mat(num_rows, num_cols, CV_8UC1, cv::Scalar(0));

#pragma omp parallel for
  for (int col = 0; col < num_cols; col++) {
    for (int row = 0; row < num_rows; row++) {
      // Create a point in the orthoimage plane corresponding to a future pixel
      double z = min_z;
      double y = min_y + row * pixel_size + pixel_size/2.0;
      double x = min_x + col * pixel_size + pixel_size/2.0;
      Eigen::Vector3d world_pt = world2planecoord.transpose() * Eigen::Vector3d(x, y, z);

      // Shoot a ray from that point to the mesh
      BVHTree::Ray bvh_ray;
      bvh_ray.dir    = dense_map::eigen_to_vec3f(plane_normal);
      bvh_ray.origin = dense_map::eigen_to_vec3f(world_pt);
      bvh_ray.tmin   = 0;
      bvh_ray.tmax   = max_z - min_z;

      // Intersect the ray with the mesh
      BVHTree::Hit hit;
      if (!bvh_tree->intersect(bvh_ray, &hit))
        continue;

      double plane_to_mesh_dist = hit.t;
      Eigen::Vector3d intersection = world_pt + plane_to_mesh_dist * plane_normal;
      Eigen::Vector3d cam_pt = cam.GetTransform() * intersection;

      // Skip points that project behind the camera
      if (cam_pt.z() <= 0)
        continue;

      // Get the distorted pixel value
      Eigen::Vector2d undist_centered_pix =
        cam.GetParameters().GetFocalVector().cwiseProduct(cam_pt.hnormalized());
      Eigen::Vector2d dist_pix;
      cam.GetParameters().Convert<camera::UNDISTORTED_C, camera::DISTORTED>
        (undist_centered_pix, &dist_pix);

      // Skip pixels that don't project in the image
      if (dist_pix.x() < 0 || dist_pix.x() > image.cols - 1 ||
          dist_pix.y() < 0 || dist_pix.y() > image.rows - 1)
        continue;

      // Find the pixel value
      cv::Size s(1, 1);
      cv::Mat interp_pix_val;
      cv::Point2f P;
      P.x = dist_pix[0];
      P.y = dist_pix[1];

      // Do bilinear interpolation into the image
      cv::getRectSubPix(image, s, P, interp_pix_val);
      uchar color = interp_pix_val.at<uchar>(0, 0);

      orthoimage.at<uchar>(row, col) = color;
    }
  }
}

void run_orthoproject(bool snap_plane,
                      std::string const& image_file,
                      double pixel_size,
                      std::string const& plane_normal_str,
                      std::shared_ptr<BVHTree> const& bvh_tree,
                      camera::CameraModel & cam) {
  // Manufacture the world_to_cam_file and output orthoimage_file
  std::string world_to_cam_file, orthoimage_file;

  world_to_cam_file = ff_common::ReplaceInStr(image_file,
                                           ".jpg", "_world2cam.txt");
  orthoimage_file = ff_common::ReplaceInStr(image_file, ".jpg", "_ortho.png");

  if (world_to_cam_file == image_file || orthoimage_file == image_file)
    LOG(FATAL) << "Expecting a .jpg image for file: " << image_file;

  Eigen::Affine3d world_to_cam;
  if (!dense_map::readAffine(world_to_cam, world_to_cam_file))
    LOG(FATAL) << "Could not read a 4x4 matrix from: " << world_to_cam_file << "\n";
  cam.SetTransform(world_to_cam);

  cv::Mat image = cv::imread(image_file, CV_LOAD_IMAGE_GRAYSCALE);
  if (image.rows == 0 || image.cols == 0)
    LOG(FATAL) << "Could not read image from: " << image_file;

  // Shoot a set of rays from the camera onto the mesh and see
  // where they intersect
  std::vector<Eigen::Vector3d> intersections;
  sample_mesh(image, cam, bvh_tree, intersections);

  Eigen::Vector3d plane_normal;
  if (plane_normal_str != "") {
    // The user wants to override the plane normal
    std::istringstream is(plane_normal_str);
    if (!(is >> plane_normal[0] >> plane_normal[1] >> plane_normal[2]))
      LOG(FATAL) << "Could not parse three values (the plane normal) from: " << plane_normal_str;
    std::cout << "Using user-defined normal to the plane: "
              << plane_normal.transpose() << std::endl;
  } else {
    // Find the best fit plane for the projections of the pixels onto the walls
    Eigen::Vector3d centroid;
    dense_map::bestFitPlane(intersections, centroid, plane_normal);

    std::cout << "Normal to the plane: " << plane_normal.transpose() << std::endl;

    // Make the plane have angles of 45 degrees with the coordinate axes
    if (snap_plane) {
      dense_map::snapPlaneNormal(plane_normal);
      std::cout << "Snapped normal to the plane: " << plane_normal.transpose() << std::endl;
    }
  }

  plane_normal.normalize();

  // Create a new coordinate system having the desired plane as its x-y
  // plane, and the normal to the plane as its z axis.
  Eigen::Matrix3d world2planecoord;
  setup_plane_coord_system(cam, plane_normal, world2planecoord);

  cv::Mat orthoimage;
  orthoproject(image, world2planecoord, plane_normal, cam,
               bvh_tree, intersections, pixel_size,
               orthoimage);

  std::cout << "Writing: " << orthoimage_file << std::endl;
  cv::imwrite(orthoimage_file, orthoimage);
}

int main(int argc, char** argv) {
  ff_common::InitFreeFlyerApplication(&argc, &argv);

  if (FLAGS_mesh.empty())
    LOG(FATAL) << "The mesh was not specified.";

  // Load the mesh
  mve::TriangleMesh::Ptr mesh;
  std::shared_ptr<mve::MeshInfo> mesh_info;
  std::shared_ptr<tex::Graph> graph;  // TODO(oalexan1): Is this necessary?
  std::shared_ptr<BVHTree> bvh_tree;
  dense_map::loadMeshBuildTree(FLAGS_mesh, mesh, mesh_info, graph, bvh_tree);

  // Load the camera intrinsics
  config_reader::ConfigReader config;
  config.AddFile("cameras.config");
  if (!config.ReadFiles())
    LOG(FATAL) << "Could not read the configuration file.";
  camera::CameraParameters cam_params(&config, "nav_cam");
  camera::CameraModel cam(Eigen::Vector3d(), Eigen::Matrix3d::Identity(),
                          cam_params);

  // Run either a single image or a list of images
  std::vector<std::string> image_list;
  if (!FLAGS_image.empty()) {
    image_list.push_back(FLAGS_image);
  } else if (!FLAGS_image_list.empty()) {
    std::ifstream ifs(FLAGS_image_list.c_str());
    std::string image_file;
    while (ifs >> image_file)
      image_list.push_back(image_file);
  } else {
    LOG(FATAL) << "Must specify either an image to orthoproject or an image list.";
  }

  for (size_t it = 0; it < image_list.size(); it++)
    run_orthoproject(FLAGS_snap_plane,
                     image_list[it],
                     FLAGS_pixel_size,
                     FLAGS_plane_normal,
                     bvh_tree, cam);

  return 0;
}

#endif

// Project a given image and camera onto a given mesh, and save an .obj file

DEFINE_string(mesh, "", "The mesh to project onto.");

DEFINE_string(image, "", "The image to orthoproject. It should be distorted, as extracted from a bag.");

DEFINE_string(camera_to_world, "",
              "The camera to world file, as written by the geometry mapper. For example: "
              "run/1616779788.2920296_nav_cam_to_world.txt.");

DEFINE_string(camera_name, "", "The the camera name. For example: 'sci_cam'.");

DEFINE_string(output_dir, "", "The output texture directory. The texture name will be <output_dir>/run.obj.");

int main(int argc, char** argv) {
  ff_common::InitFreeFlyerApplication(&argc, &argv);

  if (FLAGS_mesh.empty() || FLAGS_image.empty() || FLAGS_camera_to_world.empty() || FLAGS_camera_name.empty() ||
      FLAGS_output_dir.empty())
    LOG(FATAL) << "Not all inputs were specified.";

  if (!boost::filesystem::exists(FLAGS_output_dir))
    if (!boost::filesystem::create_directories(FLAGS_output_dir) || !boost::filesystem::is_directory(FLAGS_output_dir))
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
  dense_map::projectTexture(mesh, bvh_tree, image, cam, num_exclude_boundary_pixels, smallest_cost_per_face, face_vec,
                            uv_map);

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
