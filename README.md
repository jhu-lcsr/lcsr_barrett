LCSR Barrett Configuration
==========================

## Introduction

This package contains configuration files and scripts for bringing up the LCSR Barrett WAM robot interfaces both on real hardware and in simulation with controllers for various tasks.

### Tools Used

This package uses the following platforms:
* [**ROS**](http://www.ros.org) The Robot Operating System
* [**Orocos**](http://www.orocos.org) The Open Robot Control Software
* [**`rtt_ros_integration`**](https://github.com/orocos/rtt_ros_integration) Tools for using Orocos Real Time Toolkit (RTT) and ROS together

For real and simulated hardware interfaces, it uses the following packages:
* [**`barrett_model`**](https://github.com/jhu-lcsr/barrett_model) An URDF model of the Barrett WAM and BHand
* [**`orocos_barrett`**](https://github.com/jhu-lcsr/orocos_barrett) Orocos interfaces to the real and simulated Barrett WAM robot and BHand

The controllers, built as Orocos RTT components, can be found in the following packages:
* [**`conman`**](https://github.com/jbohren/conman) A dynamically-switching controller manager architecture
* [**`lcsr_controllers`**](https://github.com/jhu-lcsr/lcsr_controllers) A collection of robot-agnostic controllers which use the above frameworks

When running the arm in simulation, we also use the following tools:
* [**Gazebo**](http://www.gazebosim.org) A modular physical 3D simulation framework
* [**`rtt_gazebo`**](https://github.com/jhu-lcsr/rtt_gazebo) Tools for running Orocos RTT components in Gazebo
* [**`gazebo_ros_pkgs`**](https://github.com/ros-simulation/gazebo_ros_pkgs) Tolls for integrating Gazebo and ROS

### Usage

The entry point for any experiments and examples in this repository is a ROS launchfile. Any application should be launchable either on the real robot or in simulation. These launchfiles load the following:

* An [Orocos Deployer](http://www.orocos.org/stable/documentation/ocl/v2.x/doc-xml/orocos-deployment.html) which connects to real hardware or hardware in simulation;
* An [Orocos Script](http://www.orocos.org/stable/documentation/rtt/v2.x/doc-xml/orocos-components-manual.html#orocos-scripting) (`.ops` file) which loads and configures a graph of Orocos RTT components to control the arm and hand;
* The ROS parameters for any controllers that were loaded by the specified Orocos script;
* A [`robot_state_publisher`](http://wiki.ros.org/robot_state_publisher) to publish the TF frames for the Barrett devices to ROS;
* When simulating the arm, it will also start an instance of the [**Gazebo Simulator**](http://www.gazebosim.org) via the [`gazebo_ros_pkgs`](https://github.com/ros-simulation/gazebo_ros_pkgs) tools for running Gazebo with ROS.

When working with the real hardare, the aforementioned Orocos Script is specified in the launchfile and is loaded directly into an Orocos deployer, but when working in simulation, the Orocos Script is loaded by the [`rtt_gazebo_deployer`](https://github.com/jhu-lcsr/rtt_gazebo/rtt_gazebo_deployer) plugin in the Gazebo `gzserver` process.

These orocos scripts do the following:

* import RTT plugins from other ROS packages
* create RTT components defined in the imported RTT plugins
* connect RTT data ports of the various components
* connect RTT data ports to ROS topic streams
* enable and disable the initial set of RTT components

Once the system is launched, a user interacts with it entirely via ROS interfaces.

## Building

This package requires all repositories listed in the lcsr_wam or lcsr_wam_sim LCSR Rosinstall depending on whether you intend to use the real or simulated hardware:

* Real hardware: https://github.com/jhu-lcsr/rosinstalls/blob/master/applications/lcsr_wam.rosinstall
* Simulateed hardware: https://github.com/jhu-lcsr/rosinstalls/blob/master/applications/lcsr_wam_sim.rosinstall

## Commanding the Real Robot

You can command the manual trapezoidal trajectory generator from ROS like so:

Home position:
```
rostopic pub -1 /barrett/man_traj/joint_position_cmd trajectory_msgs/JointTrajectoryPoint "{ positions: [0., -1.57, 0.00, 3.0, 0.0, -0.8, 0.0] }"
```

```
rostopic pub -1 /barrett/man_traj/joint_position_cmd trajectory_msgs/JointTrajectoryPoint "{ positions: [-0.4, -1.5, 0.00, 1.5, -1.2, 1.0, 1.8] }
```

You can command the gripper over ros as well:
```
rostopic pub  /hand_cmd oro_barrett_msgs/BHandCmd "{ cmd: [2.0, 2.0, 2.0, 0.0], mode: [2,2,2,1] }" 
```
