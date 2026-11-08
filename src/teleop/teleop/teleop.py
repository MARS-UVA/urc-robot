import rclpy

from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from sensor_msgs.msg import Joy


class Teleop(Node):
    def __init__(self):
        super().__init__('teleop')

        # Subscribe to joy topic to receive joystick inputs
        self.subscription = self.create_subscription(Joy, '/joy', self.joy_callback, 10)

        # Publish to the stamped topic the controller is actively listening to
        self.publisher_ = self.create_publisher(TwistStamped, '/diff_drive_controller/cmd_vel', 10)
        
        # Publish exactly 100 times a second
        self.timer = self.create_timer(0.01, self.publish_loop)
        
        # Store current targets
        self.target_linear = 0.0
        self.target_angular = 0.0

    def joy_callback(self, msg):
        if len(msg.axes) < 2:
            self.target_linear = 0.0
            self.target_angular = 0.0
            return

        self.target_linear = 0.5 * msg.axes[1]
        self.target_angular = -0.5 * msg.axes[0]

    def publish_loop(self):
        # Create the stamped message
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x = self.target_linear
        msg.twist.angular.z = self.target_angular

        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    teleop_node = Teleop()
    try:
        rclpy.spin(teleop_node)
    except KeyboardInterrupt:
        pass
    finally:
        teleop_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()