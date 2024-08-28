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


# ROS imports
import rospy
from sensor_msgs.msg import Image as ROSImage
from rospkg import RosPack

# Third party imports
import cv2
import numpy as np


class VisualizeImageDetections(object):
    def __init__(self):
        # Params
        self.image = None
        # Node cycle rate (in Hz).
        self.loop_rate = rospy.Rate(1)

        # Publishers
        self.image_sub = rospy.Subscriber(
            "det/features/annotated_img",
            ROSImage,
            self.imageCb,
            queue_size=1,
            buff_size=2**24,
        )

    def imageCb(self, img_msg):
        img = np.frombuffer(img_msg.data, dtype=np.uint8).reshape(
            img_msg.height, img_msg.width, -1
        )[:, :, ::-1]
        cv2.imshow("Annotated Image", img)
        cv2.waitKey(1)
