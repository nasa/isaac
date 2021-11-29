\page repo_structure Repository Structure

## Folder description

The `isaac` folder is the primary entry point into flight software. For
example, if you run `roslaunch isaac <launch_file>` you are instructing ROS
to examine the 'launch' directory in this folder for a XML file called
`<launch_file>`, which describes how to start a specific experiment.

1. `config` - This folder holds all of the configuration files for flight
   software.
3. `launch` -  Used to launch the flight software stack. Most nodes export also
   export their own launch and test files.
4. `plans` - This folder holds plans, which are built using the Ground Data
   System (GDS) user interface tool.
5. `resources` - A directory containing all non-LUA resources used by nodes in
   the system. Additional files might be needed depending on the context.
6. `scripts` - A simple bash script to print out the environment variables,
   which can be used to check the context at any point in the launch sequence.

## Environment variables
\ingroup isaac

* `ISAAC_RESOURCE_DIR`: An absolute path to all non-LUA system resources that
  are used by the nodes in the system. For example, this includes sparse maps,
  zone files, clutter maps, etc.
* `ISAAC_CONFIG_DIR`: An absolute path to all LUA config files that store
  input parameters for the nodes in the system.

## Context determination

When launched, nodes must know the context in which they are being launched in
order to operate correctly. By context, we mean (a) the robot class being run,
(b) the world in which the robot is being run, and (c) paths to both a LUA
config and a resource directory. You have flexibility in how this is specified,
but note that we enforce the following strict precedence:

    /etc > environment variable > roslaunch arguments > default roslaunch values

For example, consider this launch process run on your local desktop (in this
case there will be no `/etc/robotname` file set by default)

    export ASTROBEE_ROBOT=p4d
    roslaunch isaac sim.launch

Based on the precedence rules, the environment variable will take precedence
over the roslaunch arguments. So, the ROS argument will be completely ignored
and a p4d robot will be launched. We advise that, unless you are doing a battery
of tests with the same context and want to avoid long roslaunch argument lists,
steer clear of environment variables.

The launch is carried out using the hierarchy of launch files shown below. As a
developer you can choose to launch at any points marked with a `[*]`. As a
convention, the context is passed down the hierarchy.

    [sim]                                      [*]
        -> [isaac_astrobee]                    [*]
            -> [glp]
                -> [node]                      [*]
            -> [ilp]
                -> [node] : [ff_nodelet]       [*]
            -> [astrobee]
                -> ...                         [*]

At the package level, ilp nodes inherit from ff_nodelet, and are launched using a
pattern defined by ff_nodelet.launch. This pattern respects our context
determination hierarchy. glp nodes do not inherit from ff_nodelet, and are meant for
general robot functionality.

## Default contexts

Assuming no environment variables are set or `/etc` files are created, the
default contexts defined by the launch files are the following:

* `isaac_astrobee.launch`: robot = {argument}, world = iss, drivers = true
* `sim.launch`: robot = sim, world = granite, drivers = false

## Remotely launching nodes

It is possible to launch the GLP, ILP, MLP, LLP and simulator on remote devices. The
corresponding arguments are `glp:=<ip>`, `ilp:=<ip>`, `mlp:=<ip>`, `llp:=<ip>` and
`sim:=<ip>`. There are two special values for `<ip>`:

* ip = `local` : launch on the local machine
* ip = `disabled` : disable this machine completely.

The {llp,mlp,sim} arguments with {IP,local,disabled} options is very powerful
and supports any configuration of robot with simulated nodes or hardware in the
loop.

## Launching only specific nodes

It is possible to specify on the command line the set of nodes to be launched
using the `nodes:=<comma_separated_list_of_nodes>` argument.

In this case, only the provided nodes will be launched on their destination
processors (llp or mlp). In addition, it is possible to avoid roslaunch to
perform any connection to a particular processor with the declaration
`{llp,mlp}:=disabled`. This is particularly useful if you need to test some
nodes on one processor and do not have access to the other processor.

For example, to test only the picoflexx cameras on the MLP, not attempting
connection to the LLP (in case it is absent from the test rig):

    roslaunch isaac isaac_astrobee.launch llp:=disabled nodes:=pico_driver,framestore

## Launch file examples

1. Start a local iss simulation with one p4d robot on namespace '/'

    roslaunch isaac isaac_astrobee.launch

2. Start with rviz on, the streaming mapper on, and with the robot
  undocked at a given position:
  
    roslaunch isaac sim.launch world:=iss rviz:=true streaming_mapper:=true \
      pose:="11.0 -7.0 5.0 0 0 0 1"

See 
  
    dense_map/geometry_mapper/readme.md

for how to enable the sci cam in simulation and have it publish
pictures before running the above command, and more details about the
streaming mapper.

If the bot is started docked, hence without specifying the pose as
above, it can be undocked with the command:

  rosrun dock dock_tool -undock      

and then the bot can be moved via:

    rosrun mobility teleop -move -pos "11.0 -5.0 5.0" -tolerance_pos 0.0001 -att "0 0 0 1"

3. Start the bot with the acoustics camera on.

  source ~/freeflyer_build/native/devel/setup.zsh
  source ~/projects/isaac/devel/setup.zsh 
  roslaunch isaac sim.launch world:=iss rviz:=true acoustics_cam:=true \
    pose:="11.2 -7.72 5.0 0 0 0 0 1"

See isaac/hardware/acoustics_cam/readme.md for more details.

