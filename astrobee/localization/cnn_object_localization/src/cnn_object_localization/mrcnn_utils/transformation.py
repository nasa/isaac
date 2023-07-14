# Copyright (c) 2017, United States Government, as represented by the
# Administrator of the National Aeronautics and Space Administration.
#
# All rights reserved.
#
# The Astrobee platform is licensed under the Apache License, Version 2.0
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


# Third party imports
import numpy as np

# ROS imports
import ros_numpy
import rospy
import tf2_py as tf2
import tf2_ros
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
from tf.transformations import *


class TransformAlignment:
    def __init__(self):
        self.tf_buffer = tf2_ros.Buffer()
        tf_listener = tf2_ros.TransformListener(self.tf_buffer)

    def perch_to_dock_transform(self, orig_pointcloud):
        try:
            trans = self.tf_buffer.lookup_transform(
                "dock_cam",
                orig_pointcloud.header.frame_id,
                orig_pointcloud.header.stamp,
                rospy.Duration(100),
            )
        except tf2.LookupException as ex:
            rospy.logwarn(ex)
            return
        except tf2.ExtrapolationException as ex:
            rospy.logwarn(ex)
            return

        return do_transform_cloud(orig_pointcloud, trans)

    def dock_to_world_transform(self, orig_pointcloud):
        try:
            trans = self.tf_buffer.lookup_transform(
                "dock_cam",
                orig_pointcloud.header.frame_id,
                orig_pointcloud.header.stamp,
                rospy.Duration(100),
            )
        except tf2.LookupException as ex:
            rospy.logwarn(ex)
            return
        except tf2.ExtrapolationException as ex:
            rospy.logwarn(ex)
            return

        return do_transform_cloud(orig_pointcloud, trans)

    # def transform_pose_estimate(self, pose):
    #     pose_stamped = tf2_geometry_msgs.PoseStamped()
    #     pose_stamped.pose = pose
    #     pose_stamped.header.frame_id = "dock_cam"
    #     pose_stamped.header.stamp = rospy.Time.now()
    #     try:
    #         # ** It is important to wait for the listener to start listening. Hence the rospy.Duration(1)
    #         output_pose_stamped = self.tf_buffer.transform(
    #             pose_stamped, "world", rospy.Duration(1)
    #         )
    #         return output_pose_stamped.pose

    #     except (
    #         tf2_ros.LookupException,
    #         tf2_ros.ConnectivityException,
    #         tf2_ros.ExtrapolationException,
    #     ):
    #         raise

    def msg_to_se3(self, p, q):
        """Conversion from geometric ROS messages into SE(3)

        @param msg: Message to transform. Acceptable types - C{geometry_msgs/Pose}, C{geometry_msgs/PoseStamped},
        C{geometry_msgs/Transform}, or C{geometry_msgs/TransformStamped}
        @return: a 4x4 SE(3) matrix as a numpy array
        @note: Throws TypeError if we receive an incorrect type.
        """
        norm = np.linalg.norm(q)
        if np.abs(norm - 1.0) > 1e-3:
            raise ValueError(
                "Received un-normalized quaternion (q = {0:s} ||q|| = {1:3.6f})".format(
                    str(q), np.linalg.norm(q)
                )
            )
        elif np.abs(norm - 1.0) > 1e-6:
            q = q / norm
        g = tr.quaternion_matrix(q)
        g[0:3, -1] = p
        return g

    def convert_pc_msg_to_np(self, pc_msg):
        # Fix rosbag issues, see: https://github.com/eric-wieser/ros_numpy/issues/23
        offset_sorted = {f.offset: f for f in pc_msg.fields}
        pc_msg.fields = [f for (_, f) in sorted(offset_sorted.items())]

        # Conversion from PointCloud2 msg to np array.
        pc_np = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(
            pc_msg, remove_nans=True
        )
        return pc_np  # point cloud in numpy and pcl format
