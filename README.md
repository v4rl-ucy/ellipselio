<div align="center">
    <h1>EllipseLIO</h1>
    <a href="https://github.com/VIS4ROB-lab/ellipselio"><img src="https://img.shields.io/badge/-C++-blue?logo=cplusplus" /></a>
    <a href="https://github.com/VIS4ROB-lab/ellipselio"><img src="https://img.shields.io/badge/ROS2-blue" /></a>
    <a href=""><img src="https://img.shields.io/badge/Linux-FCC624?logo=linux&logoColor=black" /></a>
    <a href="https://github.com/VIS4ROB-lab/ellipselio/blob/main/LICENSE"><img src="https://img.shields.io/badge/License-MIT-green.svg" alt="License: MIT" /></a>
    <br />
    <br />
    <a href="https://www.youtube.com/watch?v=CU6aAiTIO6Y">Video</a>
    <span>&nbsp;&nbsp;•&nbsp;&nbsp;</span>
    <a href="https://github.com/VIS4ROB-lab/ellipselio/blob/main/README.md">Install</a>
    <span>&nbsp;&nbsp;•&nbsp;&nbsp;</span>
    <a href="https://arxiv.org/abs/placeholder">Paper</a>
    <span>&nbsp;&nbsp;•&nbsp;&nbsp;</span>
    <a href="https://github.com/VIS4ROB-lab/ellipselio/issues">Report Issues</a>
  <br />
  <br />
  <p align="center"><img src=pictures/GenZ-ICP.gif alt="animated" width="500" /></p>

  [EllipseLIO][arXivlink] is an **Adaptive LiDAR Inertial Odometry Approach with an Ellipsoid Representation**
</div>

[arXivlink]: https://arxiv.org/abs/placeholder

## ROS 2 Humble

### Build

```sh
mkdir -p ~/colcon_ws/src
cd ~/colcon_ws/src
git clone https://github.com/cocel-postech/genz-icp.git
cd ..
colcon build --packages-select genz_icp --cmake-args -DCMAKE_BUILD_TYPE=Release --symlink-install
source ~/colcon_ws/install/setup.bash
```

### Run with pre-tuned config

```sh
ros2 launch genz_icp odometry.launch.py topic:=<topic_name> config_file:=<config_file_name>.yaml
ros2 bag play <rosbag_file_name>.db3
```

### Run with only topic

```sh
ros2 launch genz_icp odometry.launch.py topic:=<topic_name>
ros2 bag play <rosbag_file_name>.db3
```

## :pencil: Citation

If you use EllipseLIO please cite our preprint ([arXiv][arXivLink])
```
@ARTICLE{lee2024genzicp,
  author={Lee, Daehan and Lim, Hyungtae and Han, Soohee},
  journal={IEEE Robotics and Automation Letters (RA-L)}, 
  title={{GenZ-ICP: Generalizable and Degeneracy-Robust LiDAR Odometry Using an Adaptive Weighting}}, 
  year={2025},
  volume={10},
  number={1},
  pages={152-159},
  keywords={Localization;Mapping;SLAM},
  doi={10.1109/LRA.2024.3498779}
}
```

## :sparkles: Contributors

Like [KISS-ICP](https://github.com/PRBonn/kiss-icp),
we envision GenZ-ICP as a community-driven project, we love to see how the project is growing thanks to the contributions from the community. We would love to see your face in the list below, just open a Pull Request!

<a href="https://github.com/cocel-postech/genz-icp/graphs/contributors">
  <img src="https://contrib.rocks/image?repo=cocel-postech/genz-icp" />
</a>

## :pray: Acknowledgement

Many thanks to KISS team—[Ignacio Vizzo][nacholink], [Tiziano Guadagnino][guadagninolink], [Benedikt Mersch][merschlink]—to provide outstanding LiDAR odometry codes!

Please refer to [KISS-ICP][kissicplink] for more information

[nacholink]: https://github.com/nachovizzo
[guadagninolink]: https://github.com/tizianoGuadagnino
[merschlink]: https://github.com/benemer
[kissicplink]: https://github.com/PRBonn/kiss-icp

## :mailbox: Contact information

If you have any questions, please do not hesitate to contact
* [Rowan Border][rblink] :envelope: rborder `dot` robots `at` gmail `dot` com

[rblink]: https://github.com/rowanborder

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
