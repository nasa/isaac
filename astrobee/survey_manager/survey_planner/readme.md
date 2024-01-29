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