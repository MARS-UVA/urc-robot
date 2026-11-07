"""Check simulated sensor placement and frame conventions after Xacro expansion."""

import math
from pathlib import Path
import xml.etree.ElementTree as ET

import xacro


MODEL = (Path(__file__).resolve().parents[2] / 'robot_description' /
         'urc-robot' / 'urdf' / 'robot.urdf.xacro')


def model(**flags):
    """Expand the source model using the installed package resources."""
    mappings = {'use_sim': 'true', **flags}
    return ET.fromstring(xacro.process_file(str(MODEL), mappings=mappings).toxml())


def test_mount_directions_and_optical_frames():
    """Front, outward side cameras, and rear roof LiDAR share base_link axes."""
    robot = model()
    mounts = {}
    for name in ('front_rgbd', 'left_camera', 'right_camera', 'rear_lidar'):
        joint = robot.find(f"joint[@name='{name}_mount_joint']")
        assert joint.find('parent').get('link') == 'base_link'
        origin = joint.find('origin')
        mounts[name] = tuple(map(float, origin.get('xyz').split()))
        if name != 'rear_lidar':
            yaw = float(origin.get('rpy').split()[2])
            expected_yaw = {'front_rgbd': 0, 'left_camera': math.pi / 2,
                            'right_camera': -math.pi / 2}[name]
            assert math.isclose(yaw, expected_yaw, abs_tol=1e-8)
            optical = robot.find(f"joint[@name='{name}_optical_joint']")
            roll, pitch, yaw = map(float, optical.find('origin').get('rpy').split())
            # Rz(yaw) Ry(pitch) Rx(roll) must map optical +Z to camera +X.
            z_axis = (
                math.cos(yaw) * math.sin(pitch) * math.cos(roll) + math.sin(yaw) * math.sin(roll),
                math.sin(yaw) * math.sin(pitch) * math.cos(roll) - math.cos(yaw) * math.sin(roll),
                math.cos(pitch) * math.cos(roll),
            )
            assert all(math.isclose(a, b, abs_tol=1e-8)
                       for a, b in zip(z_axis, (1, 0, 0)))
    assert mounts['front_rgbd'][0] > mounts['left_camera'][0] > mounts['rear_lidar'][0]
    assert mounts['left_camera'][1] > mounts['front_rgbd'][1] > mounts['right_camera'][1]
    # The exported chassis roof is at z=0.33093 in base_link.
    assert mounts['rear_lidar'][2] - 0.03 > 0.33093


def test_sensor_types_and_lidar_coverage():
    """The payload contains RGB-D, two RGB cameras, and a full 3D LiDAR sweep."""
    robot = model()
    sensors = {s.get('name'): s for s in robot.findall('gazebo/sensor')}
    assert {name: sensor.get('type') for name, sensor in sensors.items()} == {
        'front_rgbd': 'rgbd_camera', 'left_camera': 'camera',
        'right_camera': 'camera', 'rear_lidar': 'gpu_lidar',
    }
    for name in ('front_rgbd', 'left_camera', 'right_camera'):
        assert sensors[name].findtext('camera/image/format') == 'R8G8B8'
    horizontal = sensors['rear_lidar'].find('lidar/scan/horizontal')
    span = float(horizontal.findtext('max_angle')) - float(horizontal.findtext('min_angle'))
    assert math.isclose(span, 2 * math.pi)
    assert int(sensors['rear_lidar'].findtext('lidar/scan/vertical/samples')) > 1


def test_sensors_can_be_disabled_and_do_not_spawn_on_hardware():
    """Flags gate the payload and hardware mode keeps the real control backend."""
    disabled = model(enable_rgbd='false', enable_side_cameras='false', enable_lidar='false')
    assert disabled.findall('gazebo/sensor') == []
    hardware = model(use_sim='false')
    assert hardware.findall('gazebo/sensor') == []
    assert hardware.findtext('ros2_control/hardware/plugin') == (
        'rocker_bogie_control/RockerBogieSystemHardware')
