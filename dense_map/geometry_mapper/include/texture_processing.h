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

#ifndef TEXTURE_PROCESSING_H_
#define TEXTURE_PROCESSING_H_

// Here we store some utilities adapted from mvs-texturing, at
// https://github.com/nmoehrle/mvs-texturing which were adapted for
// use with ISAAC. That software was released under the BSD license.

#include <Eigen/Geometry>
#include <Eigen/Core>

// texrecon includes
#include <mve/mesh_io_ply.h>
#include <mve/mesh_info.h>
#include <mve/mesh.h>
#include <mve/image.h>
#include <acc/bvh_tree.h>
#include <tex/timer.h>
#include <tex/texturing.h>
#include <tex/tri.h>
#include <tex/texture_patch.h>
#include <tex/rectangular_bin.h>
#include <util/exception.h>
#include <math/vector.h>

// OpenCV
#include <opencv2/highgui/highgui.hpp>

// Astrobee and isaac
#include <camera/camera_model.h>
#include <dense_map_utils.h>

#include <vector>
#include <map>
#include <limits>
#include <string>

typedef acc::BVHTree<unsigned int, math::Vec3f> BVHTree;

namespace dense_map {

// In order to sample a face it is easier to first rotate it so it
// is in a plane so that the face normal points along the positive
// x axis. All points in the transformed face will have a constant
// x. Store the that value of x and together with the
// lower-left value of the bounding box in that plane (so min_y, and min_z),
// and the number of samples needed (with given pixel size) to
// sample that face (with a small padding on both sides).
// Also store the transform from that plane to the face itself.
struct FaceInfo {
  double x, min_y, min_z;

  Eigen::Affine3d YZPlaneToTriangleFace;

  // We will have [min_y, min_y + width * pixel_size] x [min_z, min_z + height * pixel_size]
  // contain the face transformed in that plane with the normal pointing along the z axis.
  int width, height;

  int padding;  // The padding to apply to each face bounding box before
                // sampling it

  // The pixel at (x, min_y, min_z) in the plane will end up at location (shift_u, shift_v)
  // in the texture.
  int shift_u, shift_v;

  // The vertices of a face after being transformed to a plane with x constant
  std::vector<Eigen::Vector3d> TransformedVertices;

  // Keep track of the fact that sometimes not all faces have their info
  // initialized
  FaceInfo() {
    int max_int = std::numeric_limits<int>::max();
    shift_u = max_int;
    shift_v = max_int;
    TransformedVertices.resize(3);
  }
};

// Small utilities
Eigen::Vector3d vec3f_to_eigen(math::Vec3f const& v);
math::Vec3f eigen_to_vec3f(Eigen::Vector3d const& V);

// A texture patch without holding a buffer to the texture but only vertex and face info
class IsaacTexturePatch {
 public:
  typedef std::shared_ptr<IsaacTexturePatch> Ptr;
  typedef std::shared_ptr<const IsaacTexturePatch> ConstPtr;
  typedef std::vector<std::size_t> Faces;
  typedef std::vector<math::Vec2f> Texcoords;

 private:
  int label;
  Faces faces;
  Texcoords texcoords;
  int width, height;

 public:
  /** Constructs a texture patch. */
  IsaacTexturePatch(int _label, std::vector<std::size_t> const& faces, std::vector<math::Vec2f> const& texcoords,
                    int width, int height);

  IsaacTexturePatch(IsaacTexturePatch const& texture_patch);

  static IsaacTexturePatch::Ptr create(IsaacTexturePatch::ConstPtr texture_patch);
  static IsaacTexturePatch::Ptr create(int label, std::vector<std::size_t> const& faces,
                                       std::vector<math::Vec2f> const& texcoords, int width, int height);

  IsaacTexturePatch::Ptr duplicate(void);

  std::vector<std::size_t>& get_faces(void);
  std::vector<std::size_t> const& get_faces(void) const;
  std::vector<math::Vec2f>& get_texcoords(void);
  std::vector<math::Vec2f> const& get_texcoords(void) const;

  int get_label(void) const;
  int get_width(void) const;
  int get_height(void) const;
  int get_size(void) const;
};

inline IsaacTexturePatch::IsaacTexturePatch(int label, std::vector<std::size_t> const& faces,
                                            std::vector<math::Vec2f> const& texcoords, int width, int height)
    : label(label), faces(faces), texcoords(texcoords), width(width), height(height) {}

IsaacTexturePatch::IsaacTexturePatch(IsaacTexturePatch const& texture_patch) {
  label = texture_patch.label;
  faces = std::vector<std::size_t>(texture_patch.faces);
  texcoords = std::vector<math::Vec2f>(texture_patch.texcoords);
  width = texture_patch.width;
  height = texture_patch.height;
}

inline IsaacTexturePatch::Ptr IsaacTexturePatch::create(IsaacTexturePatch::ConstPtr texture_patch) {
  return std::make_shared<IsaacTexturePatch>(*texture_patch);
}

inline IsaacTexturePatch::Ptr IsaacTexturePatch::create(int label, std::vector<std::size_t> const& faces,
                                                        std::vector<math::Vec2f> const& texcoords, int width,
                                                        int height) {
  return std::make_shared<IsaacTexturePatch>(label, faces, texcoords, width, height);
}

inline IsaacTexturePatch::Ptr IsaacTexturePatch::duplicate(void) { return Ptr(new IsaacTexturePatch(*this)); }

inline int IsaacTexturePatch::get_label(void) const { return label; }

inline int IsaacTexturePatch::get_width(void) const { return width; }

inline int IsaacTexturePatch::get_height(void) const { return height; }

inline std::vector<math::Vec2f>& IsaacTexturePatch::get_texcoords(void) { return texcoords; }

inline std::vector<std::size_t>& IsaacTexturePatch::get_faces(void) { return faces; }

inline std::vector<math::Vec2f> const& IsaacTexturePatch::get_texcoords(void) const { return texcoords; }

inline std::vector<std::size_t> const& IsaacTexturePatch::get_faces(void) const { return faces; }

inline int IsaacTexturePatch::get_size(void) const { return get_width() * get_height(); }

/**
 * Class representing a texture atlas.
 */
class IsaacTextureAtlas {
 public:
  typedef std::shared_ptr<IsaacTextureAtlas> Ptr;

  typedef std::vector<std::size_t> Faces;
  typedef std::vector<std::size_t> TexcoordIds;
  typedef std::vector<math::Vec2f> Texcoords;

  unsigned int get_width();
  unsigned int get_height();

 private:
  unsigned int width, height, determined_height;
  unsigned int const padding;
  bool finalized;

  Faces faces;
  Texcoords texcoords;
  TexcoordIds texcoord_ids;

  mve::ByteImage::Ptr image;

  RectangularBin::Ptr bin;

  void resize_atlas(void);

 public:
  IsaacTextureAtlas(unsigned int width, unsigned int height);

  static IsaacTextureAtlas::Ptr create(unsigned int width, unsigned int height);

  Faces& get_faces(void);
  TexcoordIds& get_texcoord_ids(void);
  Texcoords& get_texcoords(void);
  mve::ByteImage::Ptr& get_image(void);

  bool insert(IsaacTexturePatch::ConstPtr texture_patch);

  void scale_texcoords(void);

  void merge_texcoords(void);

  void finalize(void);
};

inline IsaacTextureAtlas::Ptr IsaacTextureAtlas::create(unsigned int width, unsigned int height) {
  return Ptr(new IsaacTextureAtlas(width, height));
}

inline IsaacTextureAtlas::Faces& IsaacTextureAtlas::get_faces(void) { return faces; }

inline IsaacTextureAtlas::TexcoordIds& IsaacTextureAtlas::get_texcoord_ids(void) { return texcoord_ids; }

inline IsaacTextureAtlas::Texcoords& IsaacTextureAtlas::get_texcoords(void) { return texcoords; }

inline mve::ByteImage::Ptr& IsaacTextureAtlas::get_image(void) { return image; }

inline unsigned int IsaacTextureAtlas::get_width() { return width; }

inline unsigned int IsaacTextureAtlas::get_height() { return height; }

// Load and prepare a mesh
void loadMeshBuildTree(std::string const& mesh_file, mve::TriangleMesh::Ptr& mesh,
                       std::shared_ptr<mve::MeshInfo>& mesh_info, std::shared_ptr<tex::Graph>& graph,
                       std::shared_ptr<BVHTree>& bvh_tree);

void formModel(mve::TriangleMesh::ConstPtr mesh, double pixel_size, int num_threads,
               // outputs
               std::vector<FaceInfo>& face_projection_info, std::vector<IsaacTextureAtlas::Ptr>& texture_atlases,
               tex::Model& model);

// Put an textured mesh obj file in a string
void formObj(tex::Model& texture_model, std::string const& out_prefix, std::string& obj_str);

// Put an textured mesh obj file in a string
void formObjCustomUV(mve::TriangleMesh::ConstPtr mesh, std::vector<Eigen::Vector3i> const& face_vec,
                     std::map<int, Eigen::Vector2d> const& uv_map, std::string const& out_prefix, std::string& obj_str);

void formMtl(std::string const& out_prefix, std::string& mtl_str);

// Project texture and find the UV coordinates
void projectTexture(mve::TriangleMesh::ConstPtr mesh, std::shared_ptr<BVHTree> bvh_tree, cv::Mat const& image,
                    camera::CameraModel const& cam, double num_exclude_boundary_pixels,
                    // outputs
                    std::vector<double>& smallest_cost_per_face, std::vector<Eigen::Vector3i>& face_vec,
                    std::map<int, Eigen::Vector2d>& uv_map);

// Project texture on a texture model that was pre-filled already, so
// only the texture pixel values need to be computed
void projectTexture(mve::TriangleMesh::ConstPtr mesh, std::shared_ptr<BVHTree> bvh_tree, cv::Mat const& image,
                    camera::CameraModel const& cam, std::vector<double>& smallest_cost_per_face, double pixel_size,
                    int num_threads, std::vector<FaceInfo> const& face_projection_info,
                    std::vector<IsaacTextureAtlas::Ptr>& texture_atlases, tex::Model& model, cv::Mat& out_texture);

// Save a model
void isaac_save_model(ObjModel* obj_model, std::string const& prefix);

}  // namespace dense_map

#endif  // TEXTURE_PROCESSING_H_
