# URC startup

## Launch arguments

```bash
./gazebo_deploy.sh enable_rtabmap:=true enable_rviz:=true
```

| Argument | Default | Purpose |
| --- | --- | --- |
| `choose_map` | `empty.world` | World to load: `empty.world` or `rock.world` (28 sphere obstacles of varying sizes). |
| `use_sim_time` | `true` | Use Gazebo's simulation clock. |
| `render_engine` | `ogre` with `LIBGL_ALWAYS_SOFTWARE=1`; otherwise `ogre2` | Gazebo renderer. |
| `no_gui` | `false` | Hide Gazebo, RViz, and RTAB-Map windows. |
| `enable_rviz` | `false` | Open RViz. |
| `enable_rtabmap` | `true` | Build a map using ICP odometry and RGB-D/LiDAR. Requires LiDAR. |
| `enable_rtabmap_viz` | `true` | Open the RTAB-Map UI when mapping is enabled and `no_gui` is false. |
| `rviz_config` | `startup/config/mapping.rviz` when mapping; otherwise `startup/config/sensors.rviz` | RViz configuration file. |
| `rtabmap_database_path` | `~/.ros/rtabmap/urc_gazebo.db` | Map database; automatically cleared whenever RTAB-Map starts in the Gazebo launch. |
| `enable_rgbd` | `true` | Spawn and bridge the front RGB-D camera. |
| `enable_side_cameras` | `true` | Spawn and bridge the left/right RGB cameras. |
| `enable_point_lio` (hardware only) | `true` | Start real-sensor Point-LIO in `robot_launch.py`; sensor drivers must run separately. |
| `point_lio_config` (hardware only) | Point-LIO `config/unilidar_l2.yaml` | Real sensor profile; calibrate timing and LiDAR/IMU extrinsics for your hardware. |
| `point_lio_sensor_frame` (hardware only) | `unilidar_imu` | IMU frame; supply its calibrated mounting TF to `base_link`. |
| `enable_lidar` | `true` | Spawn and bridge the rear roof 360-degree LiDAR. |

## Packages and libraries used

| Package / library | Purpose |
| --- | --- |
| ROS 2 Jazzy, `launch`, `launch_ros` | Nodes, communication, and launch configuration. |
| `startup` | Rover launch files, sensor bridges, mapping configuration, and Gazebo process supervision. |
| `robot_description`, `xacro`, `robot_state_publisher` | Robot/sensor model generation and TF publication. |
| Gazebo Sim Harmonic, OGRE/OGRE2 | Simulation and sensor rendering. |
| `ros_gz_sim`, `ros_gz_bridge` | Spawn the rover and bridge Gazebo sensor data and clock into ROS. |
| `gz_ros2_control`, `controller_manager` | Connect simulated hardware to ROS 2 controllers. |
| `diff_drive_controller`, `joint_state_broadcaster` | Drive commands and joint states; wheel-odometry TF is disabled. |
| `rocker_bogie_control`, CTRE Phoenix 6 | Real Talon FX hardware driver and shared controller settings. |
| `point_lio` | LiDAR-inertial odometry from the Point-LIO ROS 2 submodule; Unmodified submodule; `deploy.sh` supplies C++17 and debug-log build settings. Our odometry adapter fills unknown covariance axes with configurable fallback variances. |
| NumPy, `tf2_ros_py` | IMU-pose/velocity conversion to `base_link`. |
| `rtabmap_odom`, `rtabmap_slam`, `rtabmap_viz` (`ros-jazzy-rtabmap-ros`) | Gazebo ICP odometry, SLAM, map publication, and RTAB-Map UI. |
| `rviz2` | Robot, sensor, and map visualization. |
| MoveIt 2, `arm_moveit_config`, `arm_description` | Arm model, motion planning, and visualization. |
| `ament_index_python`, `ament_cmake`, `ament_python`, setuptools, CMake, colcon | Package discovery, builds, and installation. |
| Python standard library | Gazebo process supervision and cleanup. |
| pytest, `ament_lint_auto`, `ament_lint_common`, `ament_copyright`, `ament_flake8`, `ament_pep257` | Tests and lint checks. |
