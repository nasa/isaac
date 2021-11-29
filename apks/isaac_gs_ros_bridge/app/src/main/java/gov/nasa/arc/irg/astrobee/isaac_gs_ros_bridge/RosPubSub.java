
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

package gov.nasa.arc.irg.astrobee.isaac_gs_ros_bridge;

import android.util.Log;

import gov.nasa.arc.astrobee.android.gs.MessageType;

import org.jboss.netty.buffer.ChannelBuffer;
import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.ConnectedNode;
import org.ros.node.Node;
import org.ros.node.NodeMain;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;

import java.util.List;

import geometry_msgs.Pose;
import geometry_msgs.Point;
import geometry_msgs.PoseStamped;
import geometry_msgs.Quaternion;
import isaac_hw_msgs.WifiSignals;
import isaac_msgs.ImageInspectionGoal;
import isaac_msgs.ImageInspectionResult;
import isaac_msgs.InspectionActionResult;
import isaac_msgs.InspectionFeedback;
import isaac_msgs.InspectionGoal;
import isaac_msgs.InspectionResult;

public class RosPubSub implements NodeMain {

    private ConnectedNode mConnectedNode;

    private StartIsaacGsRosBridgeService mBridgeService = null;

    // Publishers
    private Publisher<ImageInspectionResult> mImageInspectionResultPublisher;
    private Publisher<InspectionGoal> mInspectionGoalPublisher;

    // Subscribers
    private Subscriber<ImageInspectionGoal> mImageInspectionGoalSubscriber;
    private Subscriber<InspectionFeedback> mInspectionFeedbackSubscriber;
    private Subscriber<InspectionResult> mInspectionResultSubscriber;
    private Subscriber<WifiSignals> mWifiSignalsSubscriber;

    public RosPubSub () {
    }

    @Override
    public void onStart(ConnectedNode connectedNode) {
        mBridgeService = StartIsaacGsRosBridgeService.getInstance();
        Log.i(mBridgeService.ISAAC_GS_ROS_BRIDGE_TAG, "Starting ros pub sub node.");

        mConnectedNode = connectedNode;

        // Set up publishers
        mImageInspectionResultPublisher =
                connectedNode.newPublisher(Constants.TOPIC_GUEST_SCIENCE_IMAGE_INSPECTION_RESULT,
                        ImageInspectionResult._TYPE);
        mImageInspectionResultPublisher.setLatchMode(true);

        mInspectionGoalPublisher =
                connectedNode.newPublisher(Constants.TOPIC_GUEST_SCIENCE_INSPECTION_GOAL,
                        InspectionGoal._TYPE);
        mInspectionGoalPublisher.setLatchMode(true);

        // Set up subscribers
        mImageInspectionGoalSubscriber =
                connectedNode.newSubscriber(Constants.TOPIC_GUEST_SCIENCE_IMAGE_INSPECTION_GOAL,
                        ImageInspectionGoal._TYPE);
        mImageInspectionGoalSubscriber.addMessageListener(new MessageListener<ImageInspectionGoal>() {
            @Override
            public void onNewMessage(ImageInspectionGoal imageInspectionGoal) {
                imageInspectionGoalCallback(imageInspectionGoal);
            }
        });

        mInspectionFeedbackSubscriber =
                connectedNode.newSubscriber(Constants.TOPIC_GUEST_SCIENCE_INSPECTION_FEEDBACK,
                        InspectionFeedback._TYPE);
        mInspectionFeedbackSubscriber.addMessageListener(new MessageListener<InspectionFeedback>() {
            @Override
            public void onNewMessage(InspectionFeedback inspectionFeedback) {
                inspectionFeedbackCallback(inspectionFeedback);
            }
        });

        mInspectionResultSubscriber =
                connectedNode.newSubscriber(Constants.TOPIC_GUEST_SCIENCE_INSPECTION_RESULT,
                        InspectionResult._TYPE);
        mInspectionResultSubscriber.addMessageListener(new MessageListener<InspectionResult>() {
            @Override
            public void onNewMessage(InspectionResult inspectionResult) {
                inspectionResultCallback(inspectionResult);
            }
        });

        mWifiSignalsSubscriber =
                connectedNode.newSubscriber(Constants.TOPIC_HARDWARE_WIFI, WifiSignals._TYPE);
        mWifiSignalsSubscriber.addMessageListener(new MessageListener<WifiSignals>() {
            @Override
            public void onNewMessage(WifiSignals signals) {
                wifiSignalsCallback(signals);
            }
        });
    }

    @Override
    public GraphName getDefaultNodeName() { return GraphName.of("isaac_ros_gs_bridge_node"); }

    @Override
    public void onShutdown(Node node) {
        Log.i(mBridgeService.ISAAC_GS_ROS_BRIDGE_TAG, "Shutting down ros pub sub node.");

        mBridgeService = null;
    }

    @Override
    public void onShutdownComplete(Node node) {
        Log.i(mBridgeService.ISAAC_GS_ROS_BRIDGE_TAG, "Shutdown complete for ros pub sub node.");
    }

    @Override
    public void onError(Node node, Throwable throwable) {
        Log.i(mBridgeService.ISAAC_GS_ROS_BRIDGE_TAG, "Error encountered by ros pub sub node.");
    }

    public boolean handleGuestScienceCustomCmd(String command) {
        String[] cmd_split = command.split("@");

        if (cmd_split.length < 2) {
            Log.e(mBridgeService.ISAAC_GS_ROS_BRIDGE_TAG, "All command strings must have 2 args.");
            return false;
        }

        if (cmd_split[0].compareTo("iir") == 0) {
            // This is an image inspection result
            Log.i(mBridgeService.ISAAC_GS_ROS_BRIDGE_TAG, "Got image inspection result");
            ImageInspectionResult imageInspectionResultMsg = mImageInspectionResultPublisher.newMessage();
            imageInspectionResultMsg.setResponse(Integer.parseInt(cmd_split[1]));
            // Check to see if the result string contained something
            if (cmd_split.length > 2) {
                imageInspectionResultMsg.setResult(cmd_split[2]);
            }
            mImageInspectionResultPublisher.publish(imageInspectionResultMsg);
        } else if (cmd_split[0].compareTo("ig") == 0) {
            // This is an inspection goal message
            Log.i(mBridgeService.ISAAC_GS_ROS_BRIDGE_TAG, "Got inspection goal");
            InspectionGoal inspectionGoalMsg = mInspectionGoalPublisher.newMessage();
            inspectionGoalMsg.setCommand(Byte.parseByte(cmd_split[1]));
            // Need to figure out how many poses we have, subtract 2 for gs command name
            // and inspection command arg
            int num_args = cmd_split.length - 2;
            int num_poses = num_args/7;
            if (num_args % 7 == 0) {
                List<PoseStamped>  inspectPoses = inspectionGoalMsg.getInspectPoses();
                for (int i = 0; i < num_poses; i++) {
                    PoseStamped poseStamped = mConnectedNode.getTopicMessageFactory().newFromType(PoseStamped._TYPE);
                    Pose pose = mConnectedNode.getTopicMessageFactory().newFromType(Pose._TYPE);
                    Point position = mConnectedNode.getTopicMessageFactory().newFromType(Point._TYPE);
                    position.setX(Double.parseDouble(cmd_split[((i * 7) + 2)]));
                    position.setY(Double.parseDouble(cmd_split[((i * 7) + 3)]));
                    position.setZ(Double.parseDouble(cmd_split[((i * 7) + 4)]));
                    pose.setPosition(position);
                    Quaternion orientation = mConnectedNode.getTopicMessageFactory().newFromType(Quaternion._TYPE);
                    orientation.setX(Double.parseDouble(cmd_split[((i * 7) + 5)]));
                    orientation.setY(Double.parseDouble(cmd_split[((i * 7) + 6)]));
                    orientation.setZ(Double.parseDouble(cmd_split[((i * 7) + 7)]));
                    orientation.setW(Double.parseDouble(cmd_split[((i * 7) + 8)]));
                    pose.setOrientation(orientation);
                    poseStamped.setPose(pose);
                    inspectPoses.add(poseStamped);
                }
                inspectionGoalMsg.setInspectPoses(inspectPoses);
            }
            mInspectionGoalPublisher.publish(inspectionGoalMsg);
        } else {
            Log.e(mBridgeService.ISAAC_GS_ROS_BRIDGE_TAG, "Unknown custom guest science command.");
            return false;
        }

        return true;
    }

    public void imageInspectionGoalCallback(ImageInspectionGoal goal) {
        Log.i(mBridgeService.ISAAC_GS_ROS_BRIDGE_TAG, "Got image inspection goal");
        // TODO(Katie) Extract header timestamp and frame id to be thorough
        String gs_data = goal.getType() + "@" + goal.getImgTopic() + "\0";

        // TODO(Katie) Support data bigger than 2048
        if (gs_data.length() >= 2048) {
            Log.e(mBridgeService.ISAAC_GS_ROS_BRIDGE_TAG, "Data for image inspection goal is too big!! Not sending!");
            return;
        }
        mBridgeService.sendData(MessageType.STRING, "iig", gs_data);
    }

    public void inspectionFeedbackCallback(InspectionFeedback feedback) {
        Log.i(mBridgeService.ISAAC_GS_ROS_BRIDGE_TAG, "Got inspection feedback");
        // TODO(Katie) Extract header timestamp and frame id to be thorough
        String gs_data = feedback.getState().getState() + "@";
        gs_data += feedback.getState().getFsmEvent() + "@";
        gs_data += feedback.getState().getFsmState() + "@";
        gs_data += feedback.getState().getFsmSubevent() + "@";
        gs_data += feedback.getState().getFsmSubstate() + "\0";

        // TODO(Katie) Support data bigger than 2048
        if (gs_data.length() >= 2048) {
            Log.e(mBridgeService.ISAAC_GS_ROS_BRIDGE_TAG, "Data for inspection feedback is too big!! Not sending!");
            return;
        }
        mBridgeService.sendData(MessageType.STRING, "if", gs_data);
    }

    public void inspectionResultCallback(InspectionResult result) {
        Log.i(mBridgeService.ISAAC_GS_ROS_BRIDGE_TAG, "Got inspection result");
        // TODO(Katie) Extract header timestamp and frame id to be thorough
        String gs_data = "";

        ChannelBuffer vent_result = result.getVentResult();
        gs_data += vent_result.capacity() + "@";
        for (int i = 0; i < vent_result.capacity(); i++) {
            gs_data += Byte.toString(vent_result.getByte(i)) + "@";
        }

        ChannelBuffer geometry_result = result.getGeometryResult();
        gs_data += geometry_result.capacity() + "@";
        for (int i = 0; i < geometry_result.capacity(); i++) {
            gs_data += Byte.toString(geometry_result.getByte(i)) + "@";
        }

        gs_data += result.getResponse() + "@";
        gs_data += result.getFsmResult() + "\0";

        // TODO(Katie) Support data bigger then 2048
        if (gs_data.length() >= 2048) {
            Log.e(mBridgeService.ISAAC_GS_ROS_BRIDGE_TAG, "Data for inspection result is too big!! Not sending!");
            return;
        }
        mBridgeService.sendData(MessageType.STRING, "ir", gs_data);
    }

    public void wifiSignalsCallback(WifiSignals signals) {
        // Log.i(mBridgeService.ISAAC_GS_ROS_BRIDGE_TAG, "Got wifi signals message");
        // TODO(Katie) Extract header timestamp and frame id to be thorough
        String gs_data = Integer.toString(signals.getSignals().size()) + "@";

        for (int i = 0; i < signals.getSignals().size(); i++) {
            gs_data += Integer.toString(signals.getSignals().get(i).getHeader().getStamp().secs) + "@";
            gs_data += Integer.toString(signals.getSignals().get(i).getHeader().getStamp().nsecs) + "@";
            gs_data += signals.getSignals().get(i).getHeader().getFrameId() + "@";
            gs_data += signals.getSignals().get(i).getBssid() + "@";
            gs_data += signals.getSignals().get(i).getSsid() + "@";
            gs_data += Byte.toString(signals.getSignals().get(i).getSignalDbm()) + "@";
        }

        gs_data = gs_data.substring(0, (gs_data.length() - 1));
        gs_data += "\0";

        // TODO(Katie) Support data bigger then 2048
        if (gs_data.length() >= 2048) {
            Log.e(mBridgeService.ISAAC_GS_ROS_BRIDGE_TAG, "Data for wifi signals is too big!! Not sending!");
            return;
        }
        mBridgeService.sendData(MessageType.STRING, "ws", gs_data);
    }
}
