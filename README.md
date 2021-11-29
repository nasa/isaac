# ISAAC Software

### About

The ISAAC project actively develops in a variety of repos hosted at
Ames and JSC, as described on the (collaboration tools wiki
page)[https://babelfish.arc.nasa.gov/confluence/display/astrosoft/Astrosoft+Collaboration+Tools]

This `isaac` repo serves as a master for integrating an end-to-end
demo that draws on code from the other repos (as well as directly
including a significant amount of the ISAAC code, mostly relating to
the Astrobee robot).

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

### System requirements

The `isaac` repo depends on the `astrobee` repo, therefore it inherits
the same system requirements. You must use Ubuntu 16.04 to 20.04 64-bit. When
running in simulation, certain Gazebo plugins require appropriate
graphics drivers. See INSTALL.md in that repository for more
information.

### Usage

[Instructions on installing and using the ISAAC Software](INSTALL.md). [For running
the demos](DEMO_INSTALL.md)

### Documentation

There are Doxygen comments in the header files. To compile (make sure you have the latest doxygen installed):

    doxygen isaac.doxyfile

### Contributing

The ISAAC Software is open source, and we welcome contributions from the public.
Please submit pull requests to the `develop` branch. For us to merge any pull
requests, we must request that contributors sign and submit a Contributor License
Agreement due to NASA legal requirements. Thank you for your understanding.

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
