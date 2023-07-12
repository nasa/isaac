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


import cProfile
import itertools

# roslibpy needs a logger in order to output errors inside callbacks
import logging
import multiprocessing
import sys
import time
import os
from os import listdir
from os.path import isfile, join

import rosbag
import yaml
from pyArango.connection import *

# logging.basicConfig()

# # Create a lock
# lock = multiprocessing.Lock()


def read_bag(bag_file, topics, robot):
    # Connect to the database
    addresses = ["http://iui_arangodb:8529", "http://127.0.0.1:8529"]
    for address in addresses:
        try:
            conn = Connection(
                arangoURL=address, username="root", password="isaac", max_retries=5
            )
            break  # Connection successful, exit the loop
        except Exception as e:
            continue

    # Open the isaac database / create it if it does not exist
    if not conn.hasDatabase("isaac"):
        conn.createDatabase(name="isaac")

    db = conn["isaac"]

    # access bag
    bag = rosbag.Bag(bag_file)
    print("Reading bag file ", bag_file)
    topic_count = {}
    if topics == []:
        messages_total = bag.get_message_count()
    else:
        messages_total = bag.get_message_count(topics)
    message_count = 0
    # Go through all the messages in a topic
    for topic, msg, t in bag.read_messages():
        # Print loading status
        message_count = message_count + 1
        if message_count % 1000 == 0:
            print("Reading ", message_count, "/", messages_total, " ", end="\r")

        # Delete data field to make loading faster
        if hasattr(msg, "data"):
            setattr(msg, "data", "")
        elif hasattr(msg, "raw"):
            if hasattr(msg.raw, "data"):
                setattr(msg.raw, "data", "")

        # Save topic count for later output
        if topic in topic_count.keys():
            topic_count[topic] = topic_count.get(topic) + 1
        else:
            topic_count[topic] = 1

        # Get collection
        # Fix topic name
        topic = topic[1:].replace("/", "_")
        collection = db[topic]

        # Convert message to yaml
        # self.profiler1.enable()
        yaml_msg = yaml.safe_load(str(msg))
        yaml_msg["robot"] = robot
        # self.profiler1.disable()

        # Insert the YAML data into the collection
        # self.profiler2.enable()
        collection.createDocument(yaml_msg).save()
        # self.profiler2.disable()

    print("\nTopics found in " + bag_file + ":")
    print(topic_count)
    bag.close()


def read_bag_helper(zipped_vals):
    return read_bag(*zipped_vals)


class LoadBagDatabase:
    def __init__(self, paths, topics=[], robot="bumble"):
        # self.db = database
        # self.profiler1 = cProfile.Profile()
        # self.profiler2 = cProfile.Profile()
        bagfiles = []
        bags = []
        if isinstance(paths, str):
            paths = [paths]

        for path in paths:
            # Check the folder contents
            bagfiles += [path + "/" + f for f in listdir(path) if f.endswith(".bag")]

        print(bagfiles)
        # Connect to database
        addresses = ["http://iui_arangodb:8529", "http://127.0.0.1:8529"]
        for address in addresses:
            try:
                conn = Connection(
                    arangoURL=address, username="root", password="isaac", max_retries=5
                )
                break  # Connection successful, exit the loop
            except Exception as e:
                continue

        # Open the isaac database / create it if it does not exist
        if not conn.hasDatabase("isaac"):
            conn.createDatabase(name="isaac")
        db = conn["isaac"]

        for bagfile in bagfiles:
            bag = rosbag.Bag(bagfile)
            bag_topics = bag.get_type_and_topic_info()[1].keys()
            for topic in bag_topics:
                # Fix topic name
                topic = topic[1:].replace("/", "_")
                # Create topic collection if it doesn't exist already
                if topic not in db.collections:
                    print("Adding collection " + topic)
                    collection = db.createCollection(name=topic)

        # Initialize pool
        num_processes = os.cpu_count()
        pool = multiprocessing.Pool(num_processes)

        # Run operations on individual bags
        # izip arguments so we can pass as one argument to pool worker
        pool.map(
            read_bag_helper,
            list(
                zip(
                    bagfiles,
                    itertools.repeat(topics),
                    itertools.repeat(robot),
                )
            ),
        )

        pool.close()  # Prevents any more tasks from being submitted to the pool
        pool.join()  # Waits for all worker processes to complete

        # self.profiler1.print_stats()
        # self.profiler2.print_stats()


if __name__ == "__main__":
    path = os.path.join(os.path.expanduser('~'), "/data/bags/granite_lab/2023-04-19/bsharp/delayed/")
    topics = []
    robot = "bsharp"
    LoadBagDatabase(path, topics, robot)
    path = [os.path.join(os.path.expanduser('~'), "data/bags/20220608_Isaac9/bumble"), os.path.join(os.path.expanduser('~'), "data/bags/20220711_Isaac11/bumble")]
    topics = []
    robot = "bumble"
    LoadBagDatabase(path, topics, robot)
    path = [os.path.join(os.path.expanduser('~'), "data/bags/20220617_Isaac10/queen"), os.path.join(os.path.expanduser('~'), "data/bags/20220711_Isaac11/queen"), os.path.join(os.path.expanduser('~'), "data/bags/20220711_Isaac11/queen")]
    topics = []
    robot = "queen"
    LoadBagDatabase(path, topics, robot)
