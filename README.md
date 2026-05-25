<div align="center">
    <h1>EllipseLIO</h1>
    <a href="https://github.com/v4rl-ucy/ellipselio"><img src="https://img.shields.io/badge/-C++-blue?logo=cplusplus" /></a>
    <a href="https://github.com/v4rl-ucy/ellipselio"><img src="https://img.shields.io/badge/ROS2-blue" /></a>
    <a href="https://github.com/v4rl-ucy/ellipselio"><img src="https://img.shields.io/badge/Linux-FCC624?logo=linux&logoColor=black" /></a>
    <a href="https://github.com/v4rl-ucy/ellipselio/blob/main/LICENSE"><img src="https://img.shields.io/badge/License-MIT-green.svg" alt="MIT License" /></a>
    <br />
    <br />
    <a href="https://youtu.be/eIZ8CK4TAuA">Video</a>
    <span>&nbsp;&nbsp;•&nbsp;&nbsp;</span>
    <a href="https://github.com/v4rl-ucy/ellipselio/blob/main/README.md">Install</a>
    <span>&nbsp;&nbsp;•&nbsp;&nbsp;</span>
    <a href="http://arxiv.org/abs/2605.21150">Paper</a>
    <span>&nbsp;&nbsp;•&nbsp;&nbsp;</span>
    <a href="https://github.com/v4rl-ucy/ellipselio/issues">Report Issues</a>
  <br />
  <br />
  <p align="center"><img src=ellipselio.gif alt="animated" /></p>

  [EllipseLIO][arXivlink] is an **Adaptive LiDAR Inertial Odometry Approach with an Ellipsoid Representation**
</div>

[arXivlink]: http://arxiv.org/abs/2605.21150

## ROS2 Humble

### Build

```sh
mkdir -p ~/colcon_ws/src
cd ~/colcon_ws/src
git clone git@github.com:v4rl-ucy/ellipselio.git
cd ..
colcon build --packages-select ellipselio --cmake-args -DCMAKE_BUILD_TYPE=Release --symlink-install
source ~/colcon_ws/install/setup.bash
```

### Run with a config file

```sh
ros2 launch ellipselio ellipselio_standalone.launch.py config_file:=<config_file_name>
ros2 bag play <rosbag_file_name>
```

## :pencil: Citation

If you use EllipseLIO please cite our preprint on [arXiv][arXivLink]
```
@article{border2026ellipselio,
   author = {Border, Rowan and Chli, Margarita},
   journal = {arXiv},
   title = {{EllipseLIO}: Adaptive {LiDAR} Inertial Odometry with an Ellipsoid Representation},
   url = {http://arxiv.org/abs/2605.21150},
   year = {2026}
}

```

## :pray: Acknowledgements

Many thanks to the authors of [FAST-LIO2][fastliolink], [IKFoM][ikfomlink], and [i-Octree][ioctreelink] for open-sourcing their work, which made the development of EllipseLIO possible. 

[fastliolink]: https://github.com/hku-mars/FAST_LIO
[ikfomlink]: https://github.com/hku-mars/IKFoM
[ioctreelink]: https://github.com/zhujun3753/i-octree

## :mailbox: Contact information

If you have any questions, please do not hesitate to contact
* [Rowan Border][rblink] :envelope: rborder `dot` robots `at` gmail `dot` com

[rblink]: https://github.com/rowanborder
