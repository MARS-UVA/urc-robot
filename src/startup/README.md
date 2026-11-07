# Gazebo sensor payload

Run `./gazebo_deploy.sh` from the workspace root. Gazebo and RViz start with
all four sensors enabled. The script forwards ROS launch arguments; for example:

```bash
./gazebo_deploy.sh enable_rviz:=false
./gazebo_deploy.sh enable_side_cameras:=false
```

The sensors follow the Xacro and Gazebo-to-ROS bridge pattern in `mars-jetson`.
These are simulated sensors. Physical camera and LiDAR drivers are not started.
RTAB-Map mapping is not started by this sensor configuration.

## Mounts

All positions are in meters relative to `base_link`, whose axes are +X forward,
+Y left, +Z up. The CAD chassis keeps its existing +pi/2 fixed rotation. The
middle axle is offset from the `base_link` origin; the chassis mesh bounds were used
to place the camera housings outside its faces and the LiDAR above its roof.
The simple housings and LiDAR mounting post are visual approximations.

| Sensor | XYZ | Facing |
| --- | --- | --- |
| Front RGB-D | `0.03 -0.267839 0.25` | Forward |
| Left RGB | `-0.25 0.005 0.25` | Left (+pi/2 yaw) |
| Right RGB | `-0.25 -0.54 0.25` | Right (-pi/2 yaw) |
| Rear roof LiDAR | `-0.55 -0.267839 0.38` | Level, 360-degree sweep |

Adjust mount positions in `robot_description/urc-robot/urdf/robot.urdf.xacro`.
Sensor definitions are in the adjacent `sensors.gazebo.xacro`. Camera optical
frames use +Z forward, +X right, +Y down. The LiDAR cloud uses `rear_lidar_link`
with +X forward, +Y left, +Z up. Restart Gazebo after changing the model.

## Streams

| Topic | Type / settings |
| --- | --- |
| `/front_rgbd/color/image_raw` | Image, RGB8, 640x480 |
| `/front_rgbd/depth/image_raw` | Image, 32FC1 depth in meters, 640x480 |
| `/front_rgbd/camera_info` | CameraInfo for the shared color/depth viewpoint |
| `/left_camera/image_raw` | Image, RGB8, 640x480 |
| `/left_camera/camera_info` | CameraInfo |
| `/right_camera/image_raw` | Image, RGB8, 640x480 |
| `/right_camera/camera_info` | CameraInfo |
| `/unilidar/cloud` | PointCloud2, 360 horizontal samples x 32 vertical layers |

Camera update rates are 15 Hz and LiDAR is 10 Hz in simulation time; wall-clock
rates depend on rendering performance. RGB-D depth is limited to 0.1-10 m.
LiDAR covers +/-15 degrees vertically and ranges from 0.1 to 30 m. This is a
functional approximation, not a calibrated model of a specific hardware sensor.
All bridges use sensor-data QoS. Select Best Effort in custom RViz displays.

`enable_rgbd`, `enable_side_cameras`, and `enable_lidar` each default to true
and control both model generation and the corresponding bridges.
`rviz_config` defaults to `startup/config/sensors.rviz`, using `base_link` as
its fixed frame. A custom config can be passed with `rviz_config:=/path/to/file`.

The renderer defaults to OGRE for the software-rendering container and OGRE2
otherwise; `render_engine:=ogre2` overrides it. Both RGB-D and LiDAR message
contents were verified in the container with the default OGRE renderer.

## Verification

After building and sourcing the workspace:

```bash
python3 -m pytest src/startup/test/test_sensor_model.py
ros2 topic echo /front_rgbd/camera_info --once --qos-reliability best_effort
ros2 topic hz /left_camera/image_raw --qos-reliability best_effort
ros2 topic hz /right_camera/image_raw --qos-reliability best_effort
ros2 topic hz /unilidar/cloud --qos-reliability best_effort
```
