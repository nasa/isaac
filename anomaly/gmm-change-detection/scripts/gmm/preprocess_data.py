#!/usr/bin/env python
# Copyright (c) 2017, United States Government, as represented by the
# Administrator of the National Aeronautics and Space Administration.
#
# All rights reserved.
#
# The "ISAAC - Integrated System for Autonomous and Adaptive Caretaking
# platform" software is licensed under the Apache License, Version 2.0
# (the "License"); you may not use this file except in compliance with the
# License. You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
# License for the specific language governing permissions and limitations
# under the License.


# Converts ROS sensor_msgs:PointCloud2 to a
# python-PCL-filtered numpy array

import logging
import math
import time

# Set the logging level to suppress INFO messages
logging.getLogger("open3d").setLevel(logging.ERROR)

import geometry_msgs
import matplotlib.pyplot as plt
import numpy as np
import open3d as o3d
import pandas as pd
import pyntcloud
import ros_numpy
import rosbag
import rospy
import sensor_msgs
import tf2_py as tf2
import tf2_ros
from PIL import Image
from scipy.spatial import cKDTree
from tf import transformations as ts
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud


# See https://stackoverflow.com/questions/39772424
def convert_pc2_pcl(data):

    # # Convert the PointCloud2 message to a list of points
    # points = list(sensor_msgs.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))

    # data.__class__ = sensor_msgs.msg._PointCloud2.PointCloud2
    # offset_sorted = {f.offset: f for f in data.fields}
    # data.fields = [f for (_, f) in sorted(offset_sorted.items())]
    # Conversion from PointCloud2 msg to np array.
    np_points = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(data, remove_nans=True)
    # np_points=np.zeros((pc.shape[0],3))

    # height = pc.shape[0]
    # if len(pc.shape) == 1:  # Unordered PC2 structure
    #     np_points = np.zeros((height, 3), dtype=np.float32)
    #     np_points[:, 0] = np.resize(pc["x"], height)
    #     np_points[:, 1] = np.resize(pc["y"], height)
    #     np_points[:, 2] = np.resize(pc["z"], height)

    # else:  # Ordered PC2 structure
    #     width = pc.shape[1]
    #     np_points = np.zeros((height * width, 3), dtype=np.float32)
    #     np_points[:, 0] = np.resize(pc["x"], height * width)
    #     np_points[:, 1] = np.resize(pc["y"], height * width)
    #     np_points[:, 2] = np.resize(pc["z"], height * width)

    return np_points


# Outlier and downsample filtering of data
def filter_pcl(np_points):

    p = pyntcloud.PyntCloud(pd.DataFrame(np_points, columns=["x", "y", "z"]))
    p.to_file("normal.ply")

    # pcl.save(fil.filter(), "inliers.pcd")
    # fil.set_negative(True)
    # pcl.save(fil.filter(), "outliers.pcd")

    # Downsample outlier filtered data
    # fil = fil.make_voxel_grid_filter()
    # fil.set_leaf_size(0.01, 0.01, 0.01)
    # pcl.save(fil.filter(), "downsample.pcd")

    # Create a cKDTree for efficient nearest neighbor search
    # kdtree = cKDTree(np_points)

    # # Compute the indices of the outliers using a statistical approach
    # distances, _ = kdtree.query(np_points, k=50)  # k=50 nearest neighbors
    # mean_distances = np.mean(distances, axis=1)
    # std_distances = np.std(distances, axis=1)
    # outlier_indices = np.where((distances > mean_distances[:, None] + std_distances[:, None]) |
    #                        (distances < mean_distances[:, None] - std_distances[:, None]))

    # # Filter out the outliers
    # np_arr = np.delete(np_points, outlier_indices, axis=0)
    np_arr = np_points

    return np_arr


# Transform the pc2 msg camera coordinates from the haz_cam to body:
def translate_cam_frame(pc2_msg):
    t = geometry_msgs.msg.TransformStamped()
    t.header.stamp = pc2_msg.header.stamp
    t.header.frame_id = "haz_cam"
    t.child_frame_id = "body"

    t.transform.translation.x = 0.036
    t.transform.translation.y = -0.083
    t.transform.translation.z = -0.133

    # In quaternions:
    t.transform.rotation.x = 0.500
    t.transform.rotation.y = -0.500
    t.transform.rotation.z = 0.500
    t.transform.rotation.w = -0.500

    body_tf = do_transform_cloud(pc2_msg, t)
    return body_tf


# Transform point cloud coordinates to map ground truth (from Marina and Ryan)
def ground_truth(msg, pc2_msg):
    # Get transformation info from hazcam pc2 message
    t = geometry_msgs.msg.TransformStamped()
    # t.header.stamp = rospy.Time.now()
    t.header.stamp = msg.header.stamp
    t.header.frame_id = "body"
    t.child_frame_id = "world"

    t.transform.translation.x = msg.pose.position.x
    t.transform.translation.y = msg.pose.position.y
    t.transform.translation.z = msg.pose.position.z

    t.transform.rotation.x = msg.pose.orientation.x
    t.transform.rotation.y = msg.pose.orientation.y
    t.transform.rotation.z = msg.pose.orientation.z
    t.transform.rotation.w = msg.pose.orientation.w

    cloud_out = do_transform_cloud(pc2_msg, t)
    return cloud_out


# Obtain and concatenate point clouds remapped with ground truth transform
def read_pc2_msgs(bagfile):
    skipped = 100  # Number of messages to skip over: 5Hz -> 0.05Hz
    n_points = 171 * 224  # 171 rows x 224 columns
    n_msgs = rosbag.Bag(bagfile).get_message_count("/hw/depth_haz/points")
    n_msgs_used = math.floor(n_msgs / skipped)
    # print(n_msgs_used)
    merged_pcl = np.empty((n_msgs_used * n_points, 3), dtype=np.float32)

    topics_bag = [
        "/gnc/ekf",
        "/hw/depth_haz/points",
        "/hw/depth_haz/extended/amplitude_int",
    ]
    pc2_msg = None

    count = 0
    i = 0
    with rosbag.Bag(bagfile, "r") as bag:
        for topic, msg, t in bag.read_messages(topics_bag):
            if topic == "/hw/depth_haz/points":
                count += 1
                if count % skipped == 0:
                    pc2_msg = msg
            if topic == "/gnc/ekf" and pc2_msg is not None:

                pc2_body = translate_cam_frame(pc2_msg)
                pc2_gt = ground_truth(msg, pc2_body)

                np_arr = convert_pc2_pcl(pc2_gt)
                # print(np_arr.shape)
                merged_pcl[i : i + n_points, :] = np_arr
                i += n_points
                pc2_msg = None

    p = pyntcloud.PyntCloud(pd.DataFrame(merged_pcl, columns=["x", "y", "z"]))
    p.to_file("ground_truth_run5.ply")
    return merged_pcl


# # Process data from bagfile
# def read_pc2_msgs(bagfile):


#     for topic, msg, t in rosbag.Bag(bagfile).read_messages():
#         if topic == "/hw/depth_haz/extended/amplitude_int":
#             np_im = ros_numpy.image.image_to_numpy(msg)
#             data = Image.fromarray(np_im)

#         if (
#             topic == "/hw/depth_haz/points"
#             or topic == "/hw/depth_haz/points/ground_truth"
#         ):
#             p = convert_pc2_pcl(msg)
#             np_arr = filter_pcl(p)

#             if "data" in locals():
#                 data.show()
#                 data.save("image.png")
#                 return np_arr
#     return np_arr


# Process PCD point cloud directly
def read_ply(pcdfile):

    # Read the .ply file
    pcd_read = o3d.io.read_point_cloud(pcdfile)
    np_points = np.asarray(pcd_read.points, dtype=np.float32)
    np_arr = filter_pcl(np_points)

    return np_arr


if __name__ == "__main__":
    # filtered_data = read_pc2_msgs('/home/jcsanto3/bagfile-data/groundtruth/run5_precut.bag')
    concat_ground_truth_msgs("./groundtruth_run5.bag")
