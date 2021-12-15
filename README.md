# ISAAC (Integrated System for Autonomous and Adaptive Caretaking)

The ISAAC project has three main technical thrusts:

1. **Integrated data**: The current state of the art is that data from sensors associated with different facility
   subsystems (Structures, GN&C, and ECLSS, etc.) remain siloed. ISAAC technology unifies facility data and models with
   autonomous robotics, linking data streams from facility subsystems, sensor networks, and robots, as well as linking 3D
   geometry and sensor data map layers, and detecting changes and anomalies.
2. **Integrated control interface**: Current state of the art manages facilities with largely independent interface tools
   for different subsystems. Interfaces have different heritage, design assumptions and operator interface styles. Subsystem
   interactions are hard to analyze due to poor connectivity between separate tools. ISAAC technology combines interface
   tools for facility subsystems and autonomous robots; improves system-level situation awareness, situation understanding,
   and operator productivity; enables linking and embedding between tools to improve subsystem interaction analysis; and
   supports the entire activity life cycle from planning through execution and analysis.
3. **Coordinated execution**: The current state of the art for executing facility activities that require coordination
   between subsystems is either manual commanding (operator tracks dependencies between subsystems) or simple sequencing
   that can be brittle to even minor contingencies during execution. ISAAC technology models dependencies, uses automated
   planning to translate a high-level task definition to a plan that can work given the current system state (e.g. include
   actions to open hatches so that a robot can move where needed), and leverages ISAAC’s integrated data technology to
   watch for execution contingencies and trigger replanning as needed. This technology will reduce the time to effect
   changes on the system during critical faults and emergencies.

This `isaac` repo serves as a master for integrating an end-to-end
demo that draws on code from the other repos as well as directly
including a significant amount of the ISAAC code, mostly relating to
the Astrobee robot). This repository includes:

- [Astrobee software](https://nasa.github.io/isaac/html/astrobee.html) for inspection, cargo transport, measuring wifi signal strength.
- [Dense mapping](https://nasa.github.io/isaac/html/geometric_streaming_mapper.html) to create a textured 3D map
- [Volumetric mapping](https://nasa.github.io/isaac/html/volumetric_mapper.html) to map volumetric signals, such as WiFi.
- [Image analysis](https://nasa.github.io/isaac/html/ano.html) module to train a neural network to detect anomalies

You may also be interested in the separate repository for the [ISAAC User Interface](https://github.com/nasa/isaac_user_interface),
which enables monitoring of multiple robots through a web browser.

### System requirements

The `isaac` repo depends on the `astrobee` repo, therefore it inherits
the same system requirements. You must use Ubuntu 16.04 to 20.04 64-bit. When
running in simulation, certain Gazebo plugins require appropriate
graphics drivers. See INSTALL.md in that repository for more
information.

### Usage

There are two main ways to install and run `isaac`:

1. For development: Build the `isaac` code on your native Ubuntu OS (or
   inside a normal virtual machine) in a way that makes it convenient
   for you to edit and incrementally recompile.

2. For demo: For the ISAAC integrated demo, many ISAAC repos are
   checked out, built, and run in a distributed fashion across
   multiple docker containers. The `isaac` code itself is built and
   run inside one of these containers. Note that the in-docker build
   is managed by the Dockerfile and completely separate from any build
   in your native OS, and you don't need to install for development
   prior to installing for demo.

[Instructions on installing and using the ISAAC Software](https://nasa.github.io/isaac/html/md_INSTALL.html). For running the [docker demos](https://nasa.github.io/isaac/html/md_DEMO_INSTALL.html)

### Documentation

[The documentation is auto-generated from the contents of this repository.](https://nasa.github.io/isaac/documentation.html)

To compile the documentation locally (make sure you have the latest doxygen installed):

    doxygen isaac.doxyfile

### Contributing

The ISAAC Software is open source, and we welcome contributions from the public.
Please submit pull requests to the `develop` branch. For us to merge any pull
requests, we must request that contributors sign and submit either an [Individual Contributor License Agreement](https://github.com/nasa/isaac/blob/94996bc1a20fa090336e67b3db5c10a9bb30f0f7/doc/cla/ISAAC_Individual%20CLA.pdf) or a [Corporate Contributor License
Agreement](https://github.com/nasa/isaac/blob/94996bc1a20fa090336e67b3db5c10a9bb30f0f7/doc/cla/ISAAC_Corporate%20CLA.pdf) due to NASA legal requirements. Thank you for your understanding.

### License

Copyright (c) 2021, United States Government, as represented by the
Administrator of the National Aeronautics and Space Administration.
All rights reserved.

The "ISAAC - Integrated System for Autonomous and Adaptive Caretaking
platform" software is licensed under the Apache License, Version 2.0
"License"); you may not use this file except in compliance with the License. You
may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0.

Unless required by applicable law or agreed to in writing, software distributed
under the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
CONDITIONS OF ANY KIND, either express or implied. See the License for the
specific language governing permissions and limitations under the License.
