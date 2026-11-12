"""Convert Point-LIO's IMU pose/velocity to the rover's base_link frame."""

import math

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener


def quaternion_matrix(quaternion):
    q = np.asarray(quaternion, dtype=float)
    norm = np.linalg.norm(q)
    if not np.isfinite(norm) or norm < 1e-12:
        raise ValueError('Invalid odometry quaternion')
    x, y, z, w = q / norm
    return np.array([
        [1 - 2 * (y*y + z*z), 2 * (x*y - z*w), 2 * (x*z + y*w)],
        [2 * (x*y + z*w), 1 - 2 * (x*x + z*z), 2 * (y*z - x*w)],
        [2 * (x*z - y*w), 2 * (y*z + x*w), 1 - 2 * (x*x + y*y)],
    ])


def matrix_quaternion(rotation):
    """Return XYZW without relying on an additional transforms package."""
    r = rotation
    trace = np.trace(r)
    if trace > 0:
        s = math.sqrt(trace + 1) * 2
        q = [(r[2, 1]-r[1, 2])/s, (r[0, 2]-r[2, 0])/s,
             (r[1, 0]-r[0, 1])/s, s/4]
    else:
        i = int(np.argmax(np.diag(r)))
        j, k = (i + 1) % 3, (i + 2) % 3
        s = math.sqrt(1 + r[i, i] - r[j, j] - r[k, k]) * 2
        q = [0.0] * 4
        q[i] = s/4
        q[j] = (r[j, i] + r[i, j])/s
        q[k] = (r[k, i] + r[i, k])/s
        q[3] = (r[k, j] - r[j, k])/s
    return np.asarray(q) / np.linalg.norm(q)


def rigid_transform(position, quaternion):
    result = np.eye(4)
    result[:3, :3] = quaternion_matrix(quaternion)
    result[:3, 3] = position
    if not np.isfinite(result).all():
        raise ValueError('Invalid odometry position')
    return result


def skew(vector):
    x, y, z = vector
    return np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])


def base_state(world_sensor, base_sensor, velocity_world, omega_sensor):
    """Apply the mount lever arm to pose and twist, not just the frame label."""
    world_base = world_sensor @ np.linalg.inv(base_sensor)
    lever_world = world_base[:3, 3] - world_sensor[:3, 3]
    rotation = world_base[:3, :3]
    omega_world = world_sensor[:3, :3] @ omega_sensor
    linear_base = rotation.T @ (velocity_world + np.cross(omega_world, lever_world))
    angular_base = rotation.T @ omega_world
    # Pose errors use world axes; Point-LIO's linear velocity uses world axes,
    # while its angular velocity uses IMU axes (see upstream set_twist).
    pose_jacobian = np.eye(6)
    pose_jacobian[:3, 3:] = -skew(lever_world)
    twist_jacobian = np.zeros((6, 6))
    twist_jacobian[:3, :3] = rotation.T
    twist_jacobian[:3, 3:] = -rotation.T @ skew(lever_world) @ world_sensor[:3, :3]
    twist_jacobian[3:, 3:] = rotation.T @ world_sensor[:3, :3]
    return world_base, linear_base, angular_base, pose_jacobian, twist_jacobian


def usable_covariance(values, fallback_diagonal):
    """Fill unknown axes locally; upstream exports zero variances on some axes.

    These are configurable fallback uncertainties, not recovered filter states.
    """
    covariance = np.asarray(values, dtype=float).reshape(6, 6)
    defaults = np.asarray(fallback_diagonal, dtype=float)
    if defaults.shape != (6,) or not np.isfinite(defaults).all() or (defaults <= 0).any():
        raise ValueError('Covariance defaults must contain six positive finite variances')
    if not np.isfinite(covariance).all():
        return np.diag(defaults)
    covariance = (covariance + covariance.T) / 2
    for axis in range(6):
        if covariance[axis, axis] <= 0:
            covariance[axis, :] = covariance[:, axis] = 0
            covariance[axis, axis] = defaults[axis]
    if np.linalg.eigvalsh(covariance).min() <= 0:
        return np.diag(np.maximum(np.diag(covariance), defaults))
    return covariance


class BaseOdometryAdapter(Node):
    def __init__(self):
        super().__init__('point_lio_base_odometry')
        self.buffer = Buffer(node=self)
        self.listener = TransformListener(self.buffer, self)
        self.broadcaster = TransformBroadcaster(self)
        self.publisher = self.create_publisher(Odometry, '/point_lio/odom', 10)
        self.subscription = self.create_subscription(
            Odometry, '/point_lio/raw_odom', self.on_odometry, 10)
        self.sensor_frame = self.declare_parameter('sensor_frame', 'unilidar_imu').value
        self.pose_variances = self.declare_parameter(
            'pose_variance_defaults', [0.01, 0.01, 0.01, 0.001, 0.001, 0.001]).value
        self.twist_variances = self.declare_parameter(
            'twist_variance_defaults', [0.1, 0.1, 0.1, 0.01, 0.01, 0.01]).value
        usable_covariance(np.zeros(36), self.pose_variances)
        usable_covariance(np.zeros(36), self.twist_variances)
        self.origin = None
        self.last_stamp = None

    def on_odometry(self, message):
        if message.header.frame_id != 'point_lio_odom' or message.child_frame_id != self.sensor_frame:
            self.get_logger().error('Unexpected Point-LIO odometry frames', throttle_duration_sec=5)
            return
        try:
            mount = self.buffer.lookup_transform('base_link', self.sensor_frame, Time())
            t, q = mount.transform.translation, mount.transform.rotation
            base_sensor = rigid_transform([t.x, t.y, t.z], [q.x, q.y, q.z, q.w])
            p, q = message.pose.pose.position, message.pose.pose.orientation
            world_sensor = rigid_transform([p.x, p.y, p.z], [q.x, q.y, q.z, q.w])
            v, w = message.twist.twist.linear, message.twist.twist.angular
            world_base, velocity, omega, pose_j, twist_j = base_state(
                world_sensor, base_sensor, np.array([v.x, v.y, v.z]), np.array([w.x, w.y, w.z]))
        except (TransformException, ValueError, np.linalg.LinAlgError) as error:
            self.get_logger().warning(str(error), throttle_duration_sec=5)
            return
        stamp = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
        if self.last_stamp is not None and stamp < self.last_stamp:
            self.origin = None
        self.last_stamp = stamp
        if self.origin is None:
            # Start at the initial base position/heading while preserving gravity Z.
            yaw = math.atan2(world_base[1, 0], world_base[0, 0])
            self.origin = rigid_transform(world_base[:3, 3], [0, 0, math.sin(yaw/2), math.cos(yaw/2)])
        alignment = np.linalg.inv(self.origin)
        pose = alignment @ world_base
        quaternion = matrix_quaternion(pose[:3, :3])
        output = Odometry()
        output.header.stamp = message.header.stamp
        output.header.frame_id = 'odom'
        output.child_frame_id = 'base_link'
        p, q = output.pose.pose.position, output.pose.pose.orientation
        p.x, p.y, p.z = map(float, pose[:3, 3])
        q.x, q.y, q.z, q.w = map(float, quaternion)
        v, w = output.twist.twist.linear, output.twist.twist.angular
        v.x, v.y, v.z = map(float, velocity)
        w.x, w.y, w.z = map(float, omega)
        axes = np.zeros((6, 6))
        axes[:3, :3] = axes[3:, 3:] = alignment[:3, :3]
        for target, source, jacobian, defaults in [
            (output.pose, message.pose, axes @ pose_j, self.pose_variances),
            (output.twist, message.twist, twist_j, self.twist_variances),
        ]:
            covariance = usable_covariance(source.covariance, defaults)
            target.covariance = (jacobian @ covariance @ jacobian.T).reshape(-1).tolist()
        self.publisher.publish(output)
        transform = TransformStamped()
        transform.header = output.header
        transform.child_frame_id = output.child_frame_id
        t = transform.transform.translation
        t.x, t.y, t.z = p.x, p.y, p.z
        transform.transform.rotation = q
        self.broadcaster.sendTransform(transform)


def main():
    rclpy.init()
    node = BaseOdometryAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
