
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

import org.json.JSONException;
import org.json.JSONObject;
import org.ros.address.InetAddressFactory;
import org.ros.node.DefaultNodeMainExecutor;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMainExecutor;

import gov.nasa.arc.astrobee.android.gs.MessageType;
import gov.nasa.arc.astrobee.android.gs.StartGuestScienceService;

import java.net.URI;

/**
 * Class meant to handle commands from the Ground Data System and execute them in Astrobee
 */

public class StartIsaacGsRosBridgeService extends StartGuestScienceService {

    private NodeMainExecutor mNodeMainExecutor = null;
    private NodeConfiguration mNodeConfiguration = null;
    private RosPubSub mPubSub = null;

    public static StartIsaacGsRosBridgeService instance = null;

    public static final String ISAAC_GS_ROS_BRIDGE_TAG = "isaac_gs_ros_bridge";

     /**
     * This function is called when the GS manager starts your apk.
     * Put all of your start up code in here.
     */
    @Override
    public void onGuestScienceStart() {
        // Set up the ros portion of the apk
        try {
            String uri_str = "http://llp:11311";
            URI masterURI = new URI(uri_str);

            mNodeConfiguration =
                    NodeConfiguration.newPublic(InetAddressFactory.newNonLoopback().getHostAddress());
            mNodeConfiguration.setMasterUri(masterURI);

            mNodeMainExecutor = DefaultNodeMainExecutor.newDefault();

            mPubSub = new RosPubSub();

            mNodeMainExecutor.execute(mPubSub, mNodeConfiguration);

        } catch (Exception e) {
            Log.i(ISAAC_GS_ROS_BRIDGE_TAG, "Failed to start ros portion: " + e.getMessage());
        }

        instance = this;

        // Inform the GS Manager and the GDS that the app has been started.
        sendStarted("info");
    }

    /**
     * This function is called when the GS manager stops your apk.
     * Put all of your clean up code in here. You should also call the terminate helper function
     * at the very end of this function.
     */
    @Override
    public void onGuestScienceStop() {
        // Stop the ros portion of the apk
        mNodeConfiguration = null;
        mNodeMainExecutor = null;
        mPubSub = null;

        // Inform the GS manager and the GDS that this app stopped.
        sendStopped("info");

        // Destroy all connection with the GS Manager.
        terminate();
    }

    /**
     * This function is called when the GS manager sends a custom command to your apk.
     * Please handle your commands in this function.
     *
     * @param command
     */
    @Override
    public void onGuestScienceCustomCmd(String command) {
        /* Inform the Guest Science Manager (GSM) and the Ground Data System (GDS)
         * that this app received a command. */
        sendReceivedCustomCommand("info");

        if (!mPubSub.handleGuestScienceCustomCmd(command)) {
            sendData(MessageType.JSON, "data", ("Unrecognized Commmand" + command));
        }
    }

    public static StartIsaacGsRosBridgeService getInstance() { return instance; }
}
