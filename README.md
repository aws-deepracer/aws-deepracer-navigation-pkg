# DeepRacer Navigation Package 

## Overview

The DeepRacer Navigation ROS package creates the *deepracer_navigation_node* which is part of the core AWS DeepRacer application and will be launched from the deepracer_launcher. More details about the application and the components can be found [here](https://github.com/aws-racer/aws-deepracer-launcher).

This node is responsible for collecting the reinforcement learning model inference results and mapping it to the servo message with throttle and steering angle values based on the action space for the particular model selected.

## License

The source code is released under Apache 2.0 (https://aws.amazon.com/apache-2-0/).

## Installation

### Prerequisites

The DeepRacer device comes with all the pre-requisite packages, build systems and libraries installed to build and run the deepracer_navigation_pkg. More details about pre installed set of packages and libraries on the DeepRacer, and installing required build systems can be found in the [Getting Started](https://github.com/aws-racer/aws-deepracer-launcher/blob/main/getting-started.md) section of the AWS DeepRacer Opensource page.

The deepracer_navigation_pkg specifically depends on the following ROS2 packages as build and execute dependencies:

* *deepracer_interfaces_pkg* - This packages contains the custom message and service type definitions used across the AWS DeepRacer core application.

## Downloading and Building

Open up a terminal on the DeepRacer device and run the following commands as root user.

1. Source the ROS2 Foxy setup bash script:

        source /opt/ros/foxy/setup.bash 

1. Create a workspace directory for the package:

        mkdir -p ~/deepracer_ws
        cd ~/deepracer_ws

1. Clone the deepracer_navigation_pkg on the DeepRacer device:

        git clone https://github.com/aws-racer/aws-deepracer-navigation-pkg.git

1. Fetch unreleased dependencies:

        cd ~/deepracer_ws/aws-deepracer-navigation-pkg
        rosws update

1. Resolve the deepracer_navigation_pkg dependencies:

        cd ~/deepracer_ws && rosdep install -i --from-path . --rosdistro foxy -y

1. Build the deepracer_navigation_pkg and deepracer_interfaces_pkg:

        cd ~/deepracer_ws && colcon build --packages-select deepracer_navigation_pkg deepracer_interfaces_pkg

## Usage

Although the *deepracer_navigation_node* is built to work with the AWS DeepRacer application, it can be run independently for development/testing/debugging purposes.

### Run the node

To launch the built deepracer_navigation_node as root user on the DeepRacer device open up another terminal on the DeepRacer device and run the following commands as root user:

1. Navigate to the deepracer workspace:

        cd ~/deepracer_ws

1. Source the ROS2 Foxy setup bash script:

        source /opt/ros/foxy/setup.bash 

1. Source the setup script for the installed packages:

        source ~/deepracer_ws/install/setup.bash 

1. Launch the deepracer_navigation_node using the launch script:

        ros2 launch deepracer_navigation_pkg deepracer_navigation_pkg_launch.py

## Launch Files

The deepracer_navigation_pkg_launch.py is also included in this package that gives an example of how to launch the deepracer_navigation_node.

    from launch import LaunchDescription
    from launch_ros.actions import Node

    def generate_launch_description():
        return LaunchDescription([
            Node(
                package='deepracer_navigation_pkg',
                namespace='deepracer_navigation_pkg',
                executable='deepracer_navigation_node',
                name='deepracer_navigation_node'
            )
        ])

## Node Details

### deepracer_navigation_node

#### Subscribed Topics

| Topic Name | Message Type | Description |
| ---------- | ------------ | ----------- |
|/inference_pkg/rl_results|InferResultsArray|This message holds the reinforcement learning inference results for the state input passed through the current model that is selected in the device console.|


#### Published Topics

| Topic Name | Message Type | Description |
| ---------- | ------------ | ----------- |
|/deepracer_navigation_pkg/auto_drive|ServoCtrlMsg|Publish a message with steering angle and throttle data sent to the servo package to move the car.|

#### Services

| Service Name | Service Type | Description |
| ---------- | ------------ | ----------- |
|action_space_service|LoadModelSrv|A service that is called when a new model is loaded and helps set the action space to be considered while mapping the inference results.|
|throttle_service|NavThrottleSrv|A service that is called to dynamically set the scale value to multiply to the throttle in autonomous mode for each action.|
