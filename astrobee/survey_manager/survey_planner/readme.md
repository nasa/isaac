\page survey_planner Survey Planner

Planning and scheduling of queued survey actions using Playsys2 for PDDL solutions and behavior trees for execution.

Based on [preliminary work](https://github.com/traclabs/astrobee_task_planning_ws) by [Ana](https://github.com/ana-GT) at [Traclabs](https://traclabs.com).

## Starting suvey planner

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

If the survey_planner crashes sometimes the simulation has to be restarted, for it to restart in a good state.

### Move

    set instance bumble robot
    set instance jem_bay7 location
    set instance jem_bay6 location
    set instance jem_bay5 location

    set predicate (robot-available bumble)
    set predicate (robot-at bumble jem_bay7)
    set predicate (location-available jem_bay6)

    set predicate (move-connected jem_bay7 jem_bay6)
    set predicate (location-real jem_bay6)
    set predicate (locations-different jem_bay5 jem_bay7)
    set predicate (move-connected jem_bay5 jem_bay6)
    set predicate (location-available jem_bay5)


    run action (move bumble jem_bay7 jem_bay6 jem_bay5)

### Dock

    set instance bumble robot
    set instance jem_bay7 location
    set instance berth1 location

    set predicate (robot-available bumble)
    set predicate (robot-at bumble jem_bay7)
    set predicate (dock-connected jem_bay7 berth1)
    set predicate (location-available berth1)

    run action (dock bumble jem_bay7 berth1)


### Undock


    set instance bumble robot
    set instance jem_bay6 location
    set instance jem_bay7 location
    set instance jem_bay8 location
    set instance berth1 location

    set predicate (robot-available bumble)
    set predicate (robot-at bumble berth1)
    set predicate (dock-connected jem_bay7 berth1)
    set predicate (location-available jem_bay7)
    set predicate (location-real jem_bay7)
    set predicate (locations-different jem_bay8 jem_bay6)
    set predicate (move-connected jem_bay8 jem_bay7)
    set predicate (move-connected jem_bay6 jem_bay7)
    set predicate (location-available jem_bay8)
    set predicate (location-available jem_bay6)

    run action (undock bumble berth1 jem_bay7 jem_bay8 jem_bay6)


### Panorama

    set instance bumble robot
    set instance jem_bay6 location
    set instance  o0 order

    set predicate (robot-available bumble)

    set function (= (order-identity o0) 0)
    set function (= (robot-order bumble) -1)

    set predicate (robot-at bumble jem_bay6)

    run action (panorama bumble o0 jem_bay6)


### Stereo


    set instance bumble robot
    set instance jem_bay7 location
    set instance jem_bay6 location
    set instance jem_bay5 location
    set instance jem_bay4 location
    set instance jem_bay3 location
    set instance  o0 order


    set predicate (robot-available bumble)
    set predicate (robot-at bumble jem_bay7)
    set predicate (location-real jem_bay4)
    set predicate (need-stereo bumble o0 jem_bay7 jem_bay4)
    set predicate (location-available jem_bay4)
    set predicate (locations-different jem_bay5 jem_bay3)
    set predicate (move-connected jem_bay5 jem_bay4)
    set predicate (move-connected jem_bay3 jem_bay4)
    set predicate (location-available jem_bay5)
    set predicate (location-available jem_bay3)

    set function (= (order-identity o0) 0)
    set function (= (robot-order bumble) -1)


    run action (stereo bumble o0 jem_bay7 jem_bay4 jem_bay5 jem_bay3)