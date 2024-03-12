#Copyright (c) 2017, United States Government, as represented by the
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


# Python imports
from __future__ import division
import struct
import timeit

# Third party imports
import cv2
import numpy as np
from sklearn.cluster import DBSCAN

# ROS imports
import rospy
from cv_bridge import CvBridge
from rospkg import RosPack
from sensor_msgs.msg import Image as ROSImage
from sensor_msgs.msg import PointCloud2

# Local imports
from .mrcnn_utils.converter import *
from .mrcnn_utils.undistorter import Undistorter
from .mrcnn_utils.transformation import TransformAlignment


# Get package path in file directory
package = RosPack()
package_path = package.get_path("cnn_object_localization")


def totuple(a):
    try:
        return tuple(totuple(i) for i in a)
    except TypeError:
        return a


def round(px):
    return int(np.round(px))


class PerchCamProcess:
    def __init__(self):
        self.mask_topic = rospy.get_param("~image_topic", "det/mask")
        self.pointcloud_topic = rospy.get_param(
            "~pointcloud_topic", "/hw/depth_perch/points"
        )
        self.rgb_image_topic = rospy.get_param(
            "~rgb_image_topic", "hw/cam_dock"
        )
        self.mask = None
        self.pc = None
        self.img = None

        self.bridge = CvBridge()

        # self.camera_model = PinholeCameraModel()
        self.undist = Undistorter()

        self.pub_handrail = rospy.Publisher(
            "/hw/detected_handrail/points", PointCloud2, queue_size=10
        )
        self.pub_rgb_pointcloud = rospy.Publisher(
            "/hw/depth_perch/points/rgb", PointCloud2, queue_size=10
        )
        # Define subscribers
        self.mask_sub = rospy.Subscriber(
            self.mask_topic,
            ROSImage,
            self.mask_callback,
            queue_size=1,
            buff_size=2**24,
        )
        self.pointcloud_sub = rospy.Subscriber(
            self.pointcloud_topic,
            PointCloud2,
            self.pointcloud_callback,
            queue_size=1,
            buff_size=2**24,
        )
        self.rgb_image_sub = rospy.Subscriber(
            self.rgb_image_topic,
            ROSImage,
            self.img_callback,
            queue_size=1,
            buff_size=2**24,
        )

        self.align_transformer = TransformAlignment()

        self.mask_ = None

    def mask_callback(self, msg):
        # "Store" the message received.
        self.mask = msg

        # Compute stuff.
        self.compute_stuff()

    def img_callback(self, msg):
        img = np.asarray(self.bridge.imgmsg_to_cv2(msg, "bgr8"))
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        self.img = self.undist.undistort(
            cv2.resize(img, (320, 240), interpolation=cv2.INTER_AREA)
        )
        self.compute_stuff()

    def pointcloud_callback(self, msg):
        # "Store" the message received.
        self.pc = msg

        # Compute stuff.
        self.compute_stuff()

    def compute_stuff(self):
        if self.mask is not None and self.pc is not None and self.img is not None:
            rospy.loginfo("Processing mask and Pointcloud")
            start = timeit.default_timer()
            self.mask_ = cv2.cvtColor(
                np.asarray(self.bridge.imgmsg_to_cv2(self.mask, "rgb8")),
                cv2.COLOR_BGR2GRAY,
            )

            # transform to dock cam coordinate basis
            points_ = self.align_transformer.perch_to_dock_transform(self.pc)
            points_np = self.align_transformer.convert_pc_msg_to_np(points_)
            n_points = points_np.shape[0]
            points_handrail = []
            rgb_pointcloud = []

            K = self.undist.K
            f = K[0][0]
            cx = K[0][2]
            cy = K[1][2]

            min_pixel_z = 100 * np.ones(self.mask_.shape)
            pixel_point_map = -1 * np.ones(self.mask_.shape)
            for i in range(n_points):
                pt = points_np[i].reshape(
                    3,
                )
                px = int(np.floor(f * pt[0] / pt[2] + cx))
                py = int(np.round(f * pt[1] / pt[2] + cy))

                if py >= self.mask_.shape[0] or px >= self.mask_.shape[1]:
                    continue

                if py < 0 or px < 0:
                    continue

                px_rgb = self.img[py][px]
                rgb = struct.unpack(
                    "I", struct.pack("BBBB", px_rgb[2], px_rgb[1], px_rgb[0], 255)
                )[0]
                rgb_pointcloud.append([pt[0], pt[1], pt[2], rgb])

                # Different Approach
                if min_pixel_z[py][px] > pt[2]:
                    min_pixel_z[py][px] = pt[2]
                    pixel_point_map[py][px] = i

            handrail_pixels = np.where(self.mask_ > 0)
            points_handrail_indices = np.unique(
                pixel_point_map[handrail_pixels]
            ).astype(int)
            if points_handrail_indices[0] == -1:
                points_handrail_indices = points_handrail_indices[1:]

            points_handrail = points_np[points_handrail_indices]
            handrail_pc2 = convertPc2(
                points_handrail[
                    np.where(
                        DBSCAN(eps=0.02, min_samples=1).fit(points_handrail).labels_
                        == 0
                    )
                ]
            )
            self.pub_handrail.publish(handrail_pc2)
            stop = timeit.default_timer()
            rospy.loginfo(
                "~~~~~~~~~~~~~RGBD Alignment took "
                + str(stop - start)
                + " seconds.~~~~~~~~~~~~~~~~"
            )

            rgb_pc2 = convertPc2(rgb_pointcloud, rgb_include=True)
            self.pub_rgb_pointcloud.publish(rgb_pc2)
            rospy.loginfo(
                "Perch Cam Pointcloud segmented --> Filtered to only contain Handrail Points --> Published"
            )

            self.pc = None
            self.mask = None
