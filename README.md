# Changelog
- This fork/branch is for supporting Livox LiDAR.
- Additionally, published visualization topics are controlled through yaml files to save CPU usage.


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

## ROS2 Humble and Jazzy

### Dependencies

EllipseLIO depends on
[`livox_ros_driver2`](https://github.com/Livox-SDK/livox_ros_driver2) for
native Livox `CustomMsg` support. Install and build `livox_ros_driver2`
according to its upstream instructions, then source its workspace before
building or running EllipseLIO:

```sh
source <livox_workspace>/install/setup.bash
```

The native message path avoids an intermediate PointCloud2 converter and
preserves each point timestamp as `timebase + offset_time`.

### Build

```sh
mkdir -p ~/colcon_ws/src
cd ~/colcon_ws/src
git clone git@github.com:v4rl-ucy/ellipselio.git
cd ..
source <livox_workspace>/install/setup.bash
colcon build --packages-select ellipselio --cmake-args -DCMAKE_BUILD_TYPE=Release --symlink-install
source ~/colcon_ws/install/setup.bash
```

### Run standalone with a bag file

```sh
ros2 launch ellipselio ellipselio_standalone.launch.py config_file:=<config_file_name>
ros2 bag play --clock <imu_rate> <bag_folder> --topics <lidar_topic> <imu_topic>
```

### Included dataset configs

| Config file | Dataset |
| --- | --- |
| [`config/mid360.yaml`](config/mid360.yaml) | Livox MID-360 using `livox_ros_driver2/msg/CustomMsg` |
| [`config/os128_ncd.yaml`](config/os128_ncd.yaml) | [`Newer College Multi-Cam`](https://ori-drs.github.io/newer-college-dataset/multi-cam/) |
| [`config/os64_ncd.yaml`](config/os64_ncd.yaml) | [`Newer College Stereo-Cam`](https://ori-drs.github.io/newer-college-dataset/stereo-cam/) |
| [`config/qt64_spires.yaml`](config/qt64_spires.yaml) | [`Oxford Spires`](https://dynamic.robots.ox.ac.uk/datasets/oxford-spires/) |
| [`config/vlp16_bot.yaml`](config/vlp16_bot.yaml) | [`BotanicGarden`](https://github.com/robot-pesg/BotanicGarden) |
| [`config/vlp16_geode.yaml`](config/vlp16_geode.yaml) | [`GEODE Alpha`](https://thisparticle.github.io/geode/) |
| [`config/os64_geode.yaml`](config/os64_geode.yaml) | [`GEODE Beta`](https://thisparticle.github.io/geode/) |
| [`config/vlp16_graco.yaml`](config/vlp16_graco.yaml) | [`GRACO`](https://github.com/SYSU-RoboticsLab/GrAco) |

### Run standalone with live data

```sh
ros2 launch ellipselio ellipselio_standalone.launch.py config_file:=<config_file_name> use_sim_time:=false
```

For a Livox MID-360, publish `livox_ros_driver2/msg/CustomMsg` on
`/livox/lidar` and IMU data on `/livox/imu`, then use:

```sh
ros2 launch ellipselio ellipselio_standalone.launch.py config_file:=mid360.yaml use_sim_time:=false
```

The MID-360 config enables the native message subscriber with:

```yaml
lidar:
    type: 1
    topic: "/livox/lidar"
    use_custom_msg: true
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
| `publish.tf` | `/tf` transforms |

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
changing them. The odometry publisher uses reliable QoS for compatibility with
the RViz Odometry display.

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
