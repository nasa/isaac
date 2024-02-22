\page survey_planner Survey Planner

Planning and scheduling of queued survey actions using Playsys2 for PDDL solutions and behavior trees for execution.

Based on [preliminary work](https://github.com/traclabs/astrobee_task_planning_ws) by [Ana](https://github.com/ana-GT) at [Traclabs](https://traclabs.com).

## Starting survey planner

The survey_planner as of now starts separately from the isaac fsw. This is necessary because the planner crashes very easily, so having the ability to easily restart this component is necessary.

To start the planner and the terminal interface (in the robot, add the prefix `/opt/isaac/env_wrapper.sh`):

    roslaunch survey_planner survey_domain.launch
    rosrun plansys2_terminal plansys2_terminal

To monitor an execution and to input commands to the execution:

    rosrun survey_planner monitor_astrobee $ROBOTNAME

## Running actions manually

There are 5 different actions at the moment 'move', 'dock', 'undock', 'panorama', 'stereo'.
To run each action individually through the survey_planner you must add the correct predicates before sending the action command or else the plansys2 executor will crash. Also, if you want to re-send the action you'll want to re-send the predicates too.

Don't forget to change the commands to use the target robot's name (bumble is what is used in simulation)

If the survey_planner crashes sometimes the simulation has to be restarted for it to restart in a good state.

\subpage action_testing

## Running a full JEM survey scenario in multi-robot simulation

Unfortunately, this scenario is currently only available for
NASA-internal users because it requires access to configuration files
stored in the non-public `astrobee_ops` repo.

### Set up ops configuration

These instructions assume you've followed the native install
instructions for NASA-internal users both for
[`astrobee`](https://github.com/nasa/astrobee/blob/develop/doc/general_documentation/NASA_INSTALL.md)
and [`isaac`](https://github.com/nasa/isaac/blob/develop/INSTALL.md).

Set up GDS and `astrobee_ops` per [this NASA-internal wiki
page](https://babelfish.arc.nasa.gov/confluence/display/FFFSW/Running+GDS+in+Linux). You
can stop when you get to the "Setup Sci Cam Streaming" section.

### Environment variables to define in all terminals below

These could be added to `~/.bashrc` if you prefer:
```bash
OPS_REPO="${HOME}/astrobee/ops"  # Or wherever you put this
ISAAC_WS="${HOME}/isaac"  # Or wherever you put this
source "${ISAAC_WS}/devel/setup.bash"
```

### Running the scenario

1. Clean up any lingering processes from previous runs:
   ```bash
   # === In terminal 1 ===
   ps auxww | grep -P 'ros|survey|astrobee|gzserver|gazebo|inspection_tool'
   # If you see processes that look relevant, you can kill them with
   # 'killall <name>' or 'killall -9 <name>' if needed. Failing to clean
   # up any lingering processes seems to cause unpredictable faults like
   # robots not moving when commanded.
   ```

2. Start the simulator:
   ```bash
   # === In terminal 1 ===
   roslaunch isaac sim.launch honey:=true dds:=true gds:=true robot:=sim_pub
   # roslaunch will run indefinitely. Press Ctrl-C to shut down the system.
   ```

3. Configure the Astrobees, using one of these options:
   1. Using the Astrobee Workbench (a.k.a. "GDS") graphical interface:
      1. Configure the recording profile. Go to the "Engineering" tab
         and do the following for each robot:
         1. Use the "Select Bee..." menu at the top left to select the robot.
         2. Click "Grab Control".
         3. In the "Data to Disk" panel, select "SurveyNoDepth.json" in the
            leftmost drop-down menu. If this option is not available,
            review the "Setup GDS" section of the wiki page referenced
            above and restart.
         4. Click the leftmost "Configure Data" button next to the menu.
      2. Turn off face-forward motion. Go to the "Teleoperate" tab and do
         the following for each robot:
         1. Use the "Select Bee..." menu at the top left to select the robot.
         2. Click "Grab Control".
         3. In the "Bee Commanding" tab, in the "Options" column,
            uncheck the purple checkmark to the left of "Face Forward"
         4. Click "Apply Options". You should see the green checkmark to
            the right of "Face Forward" turn into a black "X".

   2. Using command-line tools:
      ```bash
      # === In terminal 2 ===
      cat <<EOF >/tmp/configure_cmds.txt
      # Configure the recording profile
      for prefix in "" "/honey"; do
          TOPIC_PREFIX="\${prefix}" "${OPS_REPO}/dock_scripts/hsc/cmd" -c bagger -config "${OPS_REPO}/gds/ControlStationConfig/DataToDisk-ISAAC/SurveyNoDepth.json"
      done
      # Turn off face-forward motion
      for ns in "" "-ns honey"; do
          rosrun executive teleop_tool \$ns -set_face_forward off
      done
      EOF

      # Note: On repeated runs you can just run this command again; the
      # temp file doesn't change.
      source /tmp/configure_cmds.txt
      ```

4. Start the survey manager:
   ```bash
   # === In terminal 2 ===
   roslaunch survey_planner survey.launch quick:=true
   # roslaunch will run indefinitely. Press Ctrl-C to shut down the system.
   # The quick:=true flag speeds up longer actions (for sim use only).

   # Note: This launch is fairly unreliable at startup so it's purposefully
   # kept separate from the main roslaunch, allowing us to manually
   # restart as needed without impacting the other ROS nodes.  To add
   # here when it's better understood: how can an operator check that
   # this startup succeeded? It pretty much always raises lots of
   # errors, but some of them seem benign.
   ```

5. Generate and run a plan using the PlanSys2 terminal:
   ```bash
   # === In terminal 3 ===
   cat <<EOF >/tmp/term_commands.txt
   source /tmp/problem.ps2.pddl
   get plan
   run
   EOF

   cat <<EOF >/tmp/plan_and_run.bash
   data="${ISAAC_WS}/src/astrobee/survey_manager/survey_planner/data"
   rosrun survey_planner problem_generator --terminal "--config=\${data}/jem_survey_static.yaml,\${data}/jem_survey_dynamic.yaml" --output=/tmp/problem.ps2.pddl
   cat /tmp/term_commands.txt | rosrun plansys2_terminal plansys2_terminal
   EOF

   # Note: On repeated runs you can just run this command again; the
   # temp files don't change.
   source /tmp/plan_and_run.bash
   # The plansys2_terminal will run until the plan completes, providing status
   # feedback. You can exit the terminal by pressing Ctrl-C and execution will
   # continue.
   ```

6. Wait for the scenario to complete. Some execution monitoring tips:
   - Use the Astrobee Workbench window "Overview" tab for a graphical
     view of robot state and progress.
   - The `plansys2_terminal` should display which survey manager actions
     are currently executing, along with how long they have taken so far
     relative to their estimated duration. Example: If an action was
     supposed to take 10 minutes but has taken 15 minutes so far, it is
     marked as "150%".
   - The survey manager `roslaunch` terminal should display more
     detailed console output.  Note that this is interleaved console
     output from several processes running in parallel, so it can be
     confusing to interpret.
   - You can interact with the terminal interfaces of individual
     commands that are currently executing. This allows retrying or
     skipping failed actions without triggering overall plan failure, as
     well as more detailed management of the `inspection_tool` that
     executes panoramas, like retrying or skipping individual panorama
     frames. To connect to the action running on a robot, choose the
     robot and run like this:
     ```bash
     rosrun survey_planner monitor_astrobee bumble  # or honey
     ```
