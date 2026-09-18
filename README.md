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
    <a href="http://arxiv.org/abs/2605.21150">arXiv</a>
    <span>&nbsp;&nbsp;•&nbsp;&nbsp;</span>
    <a href="https://ieeexplore.ieee.org/document/11661668">IEEE</a>
    <span>&nbsp;&nbsp;•&nbsp;&nbsp;</span>
    <a href="https://github.com/v4rl-ucy/ellipselio/issues">Report Issues</a>
  <br />
  <br />
  <p align="center"><img src=ellipselio.gif alt="animated" /></p>

  [EllipseLIO][arXivlink] is an **Adaptive LiDAR Inertial Odometry Approach with an Ellipsoid Representation**
</div>

[arXivlink]: http://arxiv.org/abs/2605.21150
[IEEElink]: https://ieeexplore.ieee.org/document/11661668

## ROS2 Humble and Jazzy

### Build

```sh
mkdir -p ~/colcon_ws/src
cd ~/colcon_ws/src
git clone git@github.com:v4rl-ucy/ellipselio.git
cd ..
colcon build --packages-select ellipselio --cmake-args -DCMAKE_BUILD_TYPE=Release --symlink-install
source ~/colcon_ws/install/setup.bash
```

### Run standalone with a bag file

```sh
ros2 launch ellipselio ellipselio_standalone.launch.py config_file:=<config_file_name>
ros2 bag play --clock <imu_rate> <bag_folder> --topics <lidar_topic> <imu_topic>
```

When running a dataset with raw Livox LiDAR data (e.g., GEODE Gamma) you also need to run a separate [node][converterLink] to convert the custom Livox messages to standard PointCloud2 messages.

[converterLink]: https://github.com/v4rl-ucy/livox_to_pointcloud2

### Included dataset configs

| Config file | Dataset |
| --- | --- |
| [`config/os128_ncd.yaml`](config/os128_ncd.yaml) | [`Newer College Multi-Cam`](https://ori-drs.github.io/newer-college-dataset/multi-cam/) |
| [`config/os64_ncd.yaml`](config/os64_ncd.yaml) | [`Newer College Stereo-Cam`](https://ori-drs.github.io/newer-college-dataset/stereo-cam/) |
| [`config/qt64_spires.yaml`](config/qt64_spires.yaml) | [`Oxford Spires`](https://dynamic.robots.ox.ac.uk/datasets/oxford-spires/) |
| [`config/vlp16_bot.yaml`](config/vlp16_bot.yaml) | [`BotanicGarden`](https://github.com/robot-pesg/BotanicGarden) |
| [`config/vlp16_geode.yaml`](config/vlp16_geode.yaml) | [`GEODE Alpha`](https://thisparticle.github.io/geode/) |
| [`config/os64_geode.yaml`](config/os64_geode.yaml) | [`GEODE Beta`](https://thisparticle.github.io/geode/) |
| [`config/avia_geode.yaml`](config/avia_geode.yaml) | [`GEODE Gamma`](https://thisparticle.github.io/geode/) |
| [`config/vlp16_graco.yaml`](config/vlp16_graco.yaml) | [`GRACO`](https://github.com/SYSU-RoboticsLab/GrAco) |
| [`config/vlp16_grandtour.yaml`](config/vlp16_grandtour.yaml) | [`GrandTour VLP-16`](https://grand-tour.leggedrobotics.com/dataset) |
| [`config/xt32_grandtour.yaml`](config/xt32_grandtour.yaml) | [`GrandTour XT-32`](https://grand-tour.leggedrobotics.com/dataset) |
| [`config/mid360_grandtour.yaml`](config/mid360_grandtour.yaml) | [`GrandTour Mid-360`](https://grand-tour.leggedrobotics.com/dataset) |

### Run standalone with live data

```sh
ros2 launch ellipselio ellipselio_standalone.launch.py config_file:=<config_file_name> use_sim_time:=false
```

### Publication control

Published outputs can be enabled or disabled independently in the YAML
configuration. Disabled outputs skip their publication timers and
message-conversion work. Internal odometry and mapping still run normally.

```yaml
publish:
    map: true
    scan: true
    markers: true
    odometry: true
    analytics: true
    tf: true
```

| Parameter | Output |
| --- | --- |
| `publish.map` | `/cloud_map` |
| `publish.scan` | `/cloud_scan` |
| `publish.markers` | `/visualization_marker` |
| `publish.odometry` | `/ellipselio_odom` |
| `publish.analytics` | `/analytics` |
| `publish.tf` | `/tf` |

For example, a headless run that only publishes odometry and TF can use:

```yaml
publish:
    map: false
    scan: false
    markers: false
    odometry: true
    analytics: false
    tf: true
```

The publication settings are read at node startup, so restart the node after
changing them.

## :pencil: Citation

If you use EllipseLIO in your work please cite our RA-L [paper][IEEELink]
```
@article{border2026ellipselio,
   author = {Rowan Border and Margarita Chli},
   doi = {10.1109/LRA.2026.3726372},
   issue = {10},
   journal = {IEEE Robotics and Automation Letters},
   pages = {11474-11481},
   title = {EllipseLIO: Adaptive LiDAR Inertial Odometry with an Ellipsoid Representation},
   volume = {11},
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
