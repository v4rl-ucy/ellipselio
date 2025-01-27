EllipseLio
==============================================================================

## Description

EllipseLio is a lidar-inertial odometry approach that uses spectral decomposition to derive geometric primitives from points in a map. These primitives are used to perform scan-to-map registration.

## Installation (Ubuntu 22.04)

- Install ROS2 Humble (includes PCL)
  - https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html (use full desktop install)
- Create a colcon workspace
  - mkdir -p ~/ros2_ws/src
- Clone EllipseLio into your colcon workspace
  - cd ~/ros2_ws/src && git clone git@github.com:VIS4ROB-lab/ellipse_lio.git
- Clone the dataset_tools and raw_image_pipeline repositories to easily run the Oxford Spires dataset
  - cd ~/ros2_ws/src && git clone git@github.com:VIS4ROB-lab/dataset_tools.git
  - cd ~/ros2_ws/src && git clone git@github.com:VIS4ROB-lab/raw_image_pipeline.git
- Build EllipseLio with colcon build
  - cd ~/ros2_ws
  - colcon build --symlink-install
- Source the colcon workspace in your shell
  - echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc

## Post installation testing
- Download a rosbag2 sequence from the Oxford Spires dataset (e.g., observatory-quarter-01) 
    - https://ori-drs.github.io/datasets/oxford-spires/
- Modify the dataset_name and results_folder path arguments in dataset_tools/launch/spires_dataset_{cam/no_cam}.launch.py
- Run one of the following commands to start EllipseLio
    - ros2 launch dataset_tools spires_dataset_cam.launch.py
    - ros2 launch dataset_tools spires_dataset_no_cam.launch.py

## Dependencies
- ROS2 Humble
- PCL
- OpenCV

## Project Owners

Rowan Border <rborder.robots@gmail.com>
