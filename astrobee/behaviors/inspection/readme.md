\page inspection Inspection Behavior

This directory provides two tools: inspection_tool and sci_cam_tool.

Using the inspection tool
---------

This tool is used to initiate inspection actions. To run the tool:
	
	rosrun inspection inspection_tool -$ACTION [OPTIONS]

As of now, the actions available are:

*pause*: Pauses the current action being performed, the goal is kept, so a resume command will resume the action.

*resume*: Resumes a paused or failed action (in the case where there is a motion fail or picture fail). It will restart on the "move to inspection pose" step. 

*repeat*: Resumes the survey repeating the last inspection pose.

*skip*: Skips the surrent inspection pose, useful if a certain pose is unreachable.


*save*: Saves the current goal, in the case flight software needs to be restarted. The current survey is saved in resources/current.txt. It can be loaded afterwards using 

*anomaly*: Starts an anomaly inspection action. The robot will come up to a target, take a picture and run the picture through the image anomaly detector to classify it.

*geometry*: Starts a geometry inspection, meaning that it will go to the commanded poses and take pictures to be processed by the geometry mapper.

*panorama*: it will do a panorama survey of all points specified

*volumetric*: This will perform a volumetric survey

Using sci_cam_tool
---------

This tool is used to control the sci cam plugin in the Astrobee simulator, more precisely the way it acquires pictures. To use it, perform the following steps:

Start the simulator, for example as:

    roslaunch astrobee sim.launch

Start this tool, for example as:

	rosrun inspection sci_cam_tool

There are three modes of operating the sci cam that one can choose from:

	1) takeSinglePicture
	2) turnOnContinuousPictureTaking
	3) turnOffContinuousPictureTaking

The behavior corresponds to taking a single picture at this time, allowing the camera to take pictures as often as it can, and disallowing this mode.

The pictures will be published on the topic

	/hw/cam_sci/compressed

They will also show up in the sci cam window in RVIZ.

If requests to take a single picture come at a high rate, some of them will be dropped.
