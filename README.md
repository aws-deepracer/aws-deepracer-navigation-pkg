# AWS DeepRacer navigation package 

## Overview

The AWS DeepRacer navigation ROS package creates the `deepracer_navigation_node`, which is part of the core AWS DeepRacer application and launches from the `deepracer_launcher`. For more information about the application and the components, see the [aws-deepracer-launcher repository](https://github.com/aws-deepracer/aws-deepracer-launcher).

This node is responsible for collecting the reinforcement learning model inference results and mapping them to the servo message with throttle and steering angle values based on the action space for the particular model selected.

## License

The source code is released under [Apache 2.0](https://aws.amazon.com/apache-2-0/).

## Installation

Follow these instructions to install the AWS DeepRacer navigation package.

### Prerequisites

The AWS DeepRacer device comes with all the prerequisite packages and libraries installed to run the `deepracer_navigation_pkg`. For more information about the preinstalled set of packages and libraries on the AWS DeepRacer and about installing the required build systems, see [Getting started with AWS DeepRacer OpenSource](https://github.com/aws-deepracer/aws-deepracer-launcher/blob/main/getting-started.md).

The `deepracer_navigation_pkg` specifically depends on the following ROS 2 packages as build and run dependencies:

* `deepracer_interfaces_pkg`: This package contains the custom message and service type definitions used across the AWS DeepRacer core application.

## Downloading and building

Open a terminal on the AWS DeepRacer device and run the following commands as the root user.

1. Switch to the root user before you source the ROS 2 installation:

        sudo su

1. Source the ROS 2 Foxy setup bash script:

        source /opt/ros/foxy/setup.bash 

1. Create a workspace directory for the package:

        mkdir -p ~/deepracer_ws
        cd ~/deepracer_ws

1. Clone the `deepracer_navigation_pkg` on the AWS DeepRacer device:

        git clone https://github.com/aws-deepracer/aws-deepracer-navigation-pkg.git

1. Fetch unreleased dependencies:

        cd ~/deepracer_ws/aws-deepracer-navigation-pkg
        rosws update

1. Resolve the `deepracer_navigation_pkg` dependencies:

        cd ~/deepracer_ws/aws-deepracer-navigation-pkg && rosdep install -i --from-path . --rosdistro foxy -y

1. Build the `deepracer_navigation_pkg` and `deepracer_interfaces_pkg`:

        cd ~/deepracer_ws/aws-deepracer-navigation-pkg && colcon build --packages-select deepracer_navigation_pkg deepracer_interfaces_pkg

## Usage

Although the `deepracer_navigation_node` is built to work with the AWS DeepRacer application, you can run it independently for development, testing, and debugging purposes.

### Run the node

To launch the built `deepracer_navigation_node` as the root user on the AWS DeepRacer device, open another terminal on the AWS DeepRacer device and run the following commands as the root user.

1. Switch to the root user before you source the ROS 2 installation:

        sudo su

1. Source the ROS 2 Foxy setup bash script:

        source /opt/ros/foxy/setup.bash 

1. Source the setup script for the installed packages:

        source ~/deepracer_ws/aws-deepracer-navigation-pkg/install/setup.bash 

1. Launch the `deepracer_navigation_node` using the launch script:

        ros2 launch deepracer_navigation_pkg deepracer_navigation_pkg_launch.py

## Launch files

The `deepracer_navigation_pkg_launch.py`, included in this package, gives an example of how to launch the `deepracer_navigation_node`.

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

## Node details

### `deepracer_navigation_node`

#### Subscribed topics

| Topic name | Message type | Description |
| ---------- | ------------ | ----------- |
|/`inference_pkg`/`rl_results`|`InferResultsArray`|This message holds the reinforcement learning inference results for the state input passed through the current model that is selected in the device console.|


#### Published topics

| Topic name | Message type | Description |
| ---------- | ------------ | ----------- |
|/`deepracer_navigation_pkg`/`auto_drive`|`ServoCtrlMsg`|Publish a message with steering angle and throttle data sent to the servo package to move the car.|

#### Services

| Service name | Service type | Description |
| ---------- | ------------ | ----------- |
|`action_space_service`|`LoadModelSrv`|A service that is called when a new model is loaded and helps set the action space to be considered while mapping the inference results.|
|`throttle_service`|`NavThrottleSrv`|A service that is called to dynamically set the scale value to multiply to the throttle in autonomous mode for each action.|

## Resources

* [Getting started with AWS DeepRacer OpenSource](https://github.com/aws-deepracer/aws-deepracer-launcher/blob/main/getting-started.md)

