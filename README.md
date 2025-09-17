# NERF_Robotic_Inspection


## This repository contains:

- A C++ ROS2 component build and linked with the C++ Godot 4 source code.


## Background

### Godot 4.4 and ROS2

Godot 4.4 is the latest iteration of the Godot game engine, known for its open-source nature, modular architecture, and robust community support. Compared to other popular game engines like Unity and Unreal, Godot stands out due to its lightweight footprint, ease of use, and complete access to its source code. These features make it particularly suitable for integration with other open-source projects, such as ROS2.

Godot's architecture is highly modular, allowing developers to extend or modify its functionality through modules. [Custom Godot modules in C++](https://docs.godotengine.org/en/stable/contributing/development/core_and_modules/custom_modules_in_cpp.html). In the context of ROS2, this modularity enables tight integration with ROS2's middleware, making it possible to run complex simulations and control systems entirely within the Godot environment.

Thanks to nordstream3 for this awesome repository https://github.com/nordstream3/Godot-4-ROS2-integration

## Installation

### Prerequisites

- Godot 4.4
- ROS2 (Humble Hawksbill or later)
- rqt_image_view ROS2 add-on

### Building ROS2 from source
You might be able to skip this step, however I had to build ROS2, because of inconsistent libraries on my computer.
Find the instructions for building ROS2 Humble from source [here](https://docs.ros.org/en/humble/Installation/Alternatives/Ubuntu-Development-Setup.html).


### Download Godot 4.4 source code (at time of writing 4.4 is not yet a release, so just pulling the main branch - latest)

1. Clone repository to a folder of your choice (more info [here](https://docs.godotengine.org/en/stable/contributing/development/compiling/getting_source.html)):
   ```bash
   git clone https://github.com/godotengine/godot.git

### Clone THIS repository

1. Clone repository:
   ```bash
   git clone https://github.com/Sgt-Hashtag/NERF_Robotic_Inspection.git

2. go to folder /path/to/godot_custom_modules/godot_ros and clone 4.4 tagged branch of godot cpp binding for GDExtension Api
   ```bash
   git clone https://github.com/godotengine/godot-cpp.git

2. Edit the "SCsub" file in /path/to/godot_custom_modules/godot_ros to match your ROS2 distribution and install directory. By default these values are:
   ```bash
   ros_distro = "humble"
   ros_dir = "/opt/ros/" + ros_distro
   
3. Build Godot 4.x with ROS2-module:
   ```bash
   cd /path/to/godot-source-code
   scons -j8 verbose=yes disable_exceptions=false SHOWBUILD=1 custom_modules= ../godot_custom_modules/ platform=linuxbsd

