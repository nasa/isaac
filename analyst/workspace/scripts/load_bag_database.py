# Copyright © 2021, United States Government, as represented by the Administrator of the
# National Aeronautics and Space Administration. All rights reserved.
#
# The “ISAAC - Integrated System for Autonomous and Adaptive Caretaking platform” software is
# licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
#
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software distributed under the
# License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND,
# either express or implied. See the License for the specific language governing
# permissions and limitations under the License.
#
# ----------------------------------------------------------------------------------------------------
# ISAAC Interface
# Backend API
# ----------------------------------------------------------------------------------------------------
# Database interface class definition
# ----------------------------------------------------------------------------------------------------


# roslibpy needs a logger in order to output errors inside callbacks
import logging
import sys
import time
from os import listdir
from os.path import isfile, join

import rosbag
import yaml
from pyArango.connection import *

logging.basicConfig()


class LoadBagDatabase:
    def __init__(self, database, path, topics=[]):
        self.db = database
        # Check the folder contents
        bagfiles = [f for f in listdir(path) if f.endswith(".bag")]
        for bag in bagfiles:
            self.read_bag(path + bag, topics)

    def read_bag(self, bag_file, topics_list):
        # access bag
        print("Reading bag file ", bag_file)
        bag = rosbag.Bag(bag_file)
        topic_count = {}
        messages_total = bag.get_message_count(topics_list)
        message_count = 0
        # Go through all the messages in a topic
        for subtopic, msg, t in bag.read_messages(topics_list):
            # Print loading status
            message_count = message_count + 1
            print("Reading ", message_count, "/", messages_total, " ", end="\r")

            # Fix topic name
            subtopic = subtopic[1:].replace("/", "_")

            # Save topic count for later output
            if subtopic in topic_count.keys():
                topic_count[subtopic] = topic_count.get(subtopic) + 1
            else:
                topic_count[subtopic] = 1

            # Create topic collection if it doesn't exist already
            if not self.db.hasCollection(subtopic):
                self.db.createCollection(name=subtopic)
            # Convert message to yaml
            msg = yaml.safe_load(str(msg))
            # Upload to database
            aql = (
                "INSERT "
                + "{'message':"
                + str(msg)
                + "}"
                + " INTO "
                + subtopic
                + " LET newDoc = NEW RETURN newDoc"
            )
            queryResult = self.db.AQLQuery(aql)

        print("\nTopics found:")
        print(topic_count)
        bag.close()
