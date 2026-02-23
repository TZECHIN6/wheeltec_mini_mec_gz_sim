# wheeltec_mini_mec_gz_sim
A template project integrating ROS 2 and Gazebo simulator with a Mecanum Drive robot.

![Screenshot from 2025-03-31 18-02-49](https://github.com/user-attachments/assets/0bb2d0c8-f3e3-473c-bc0e-9f97a24b8715)

## Included packages

* `ros_gz_example_description` - holds the sdf description of the simulated system and any other assets.

* `ros_gz_example_gazebo` - holds gazebo specific code and configurations. Namely this is where systems end up.

* `ros_gz_example_application` - holds ros2 specific code and configurations.

* `ros_gz_example_bringup` - holds launch files and high level utilities.


## Install

> [!NOTE]
> Please switch to `humble` branch if you are using ROS2 Humble with Gazebo Fortress.

### Requirements

1. Install ROS2 Humble and Gazebo Fortress (the gazebo sim version is `Gazebo Sim, version 6.17.0` when writing this)

1. Install necessary tools

    ```bash
    sudo apt install python3-vcstool python3-colcon-common-extensions git wget
    ```

## Usage

> [!Warning]
> The `model.sdf` of wheeltec mini mec robot is depicated and might be replaced by the `model_test.sdf` after further testing.

1. Clone the project to your workspace

    ```bash
    mkdir -p ~/ros2_ws/src
    cd ~/ros2_ws/src
    git clone https://github.com/TZECHIN6/wheeltec_mini_mec_gz_sim.git -b humble
    ```

1. Install dependencies

    ```bash
    cd ~/ros2_ws
    source /opt/ros/humble/setup.bash
    sudo rosdep init
    rosdep update
    rosdep install --from-paths src --ignore-src -r -i -y --rosdistro humble
    ```

1. Build the project

    ```bash
    colcon build --cmake-args -DBUILD_TESTING=ON
    ```

1. Source the workspace

    ```bash
    source ~/ros2_ws/install/setup.sh
    ```

1. Launch the simulation

    ```bash
    ros2 launch ros_gz_example_bringup wheeltec_mini_mec.launch.py
    ```

## Contributions

Any suggestions are welcomed! Feel free to start an issue or PR.

If you found this project is helpful, please give it a star. ☺️

_For a more detailed guide on using this template see [documentation](https://gazebosim.org/docs/latest/ros_gz_project_template_guide)._
