# ISAAC Software

### About

The Integrated System for Autonomous and Adaptive Caretaking project is developing technology for combining robots inside a spacecraft with vehicle infrastructure subsystems, to form an integrated autonomous system.
The system validates its technology through demonstrations using the Astrobee free-flier robots, existing robots onboard the International Space Station as analogs for future robots that will be developed for the Gateway.

The ISAAC project actively develops in a variety of repos hosted at
Ames and JSC.

This `isaac` repo serves as a master for integrating an end-to-end
demo that draws on code from the other repos as well as directly
including a significant amount of the ISAAC code, mostly relating to
the Astrobee robot.


### System requirements

The `isaac` repo depends on the `astrobee` repo, therefore it inherits
the same system requirements. You must use Ubuntu 16.04 to 20.04 64-bit. When
running in simulation, certain Gazebo plugins require appropriate
graphics drivers.

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
