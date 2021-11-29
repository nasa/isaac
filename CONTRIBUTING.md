# Contributing to ISAAC

First off, thanks for taking the time to contribute!

## General code structure

This repo contains ROS nodes that are compiled using catkin following the
general (concepts)[http://wiki.ros.org/ROS/Tutorials/catkin/CreatingPackage]

Each package follows the general structure:

    package_name/
        CMakeLists.txt       -- 'Toplevel' CMake file, provided by catkin
        package.xml          -- Package manifest for package
        include/             -- Has all the .h incude files
        launch/              -- Contains an example launch file that launches this node
        src/                 -- Contains the package node source code
        tools/               -- Contains tools that can be started while the package is running

## How can I Contribute?
- Reporting bugs
- Suggesting Enhancements
- Pull Requests


## Coding conventions
Start reading our code and you'll get the hang of it. We genrally follow the
(google style guides)[https://google.github.io/styleguide/cppguide.html]


Thank you!
The Astrobee FSW team