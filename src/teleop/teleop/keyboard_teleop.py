import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
import sys
import termios
import tty
import select

class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('keyboard_teleop')
        # Publish to the stamped topic the controller is actively listening to
        self.publisher_ = self.create_publisher(TwistStamped, '/diff_drive_controller/cmd_vel', 10)
        self.get_logger().info('Teleop Node Started. W/A/S/D to drive, Space to stop. Ctrl+C to exit.')
        
        # Publish exactly 10 times a second
        self.timer = self.create_timer(0.1, self.publish_loop)
        self.settings = termios.tcgetattr(sys.stdin)
        
        # Store current targets
        self.target_linear = 0.0
        self.target_angular = 0.0

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.05)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def publish_loop(self):
        key = self.get_key()

        if key == 'w':
            self.target_linear = 0.5
            self.target_angular = 0.0
        elif key == 's':
            self.target_linear = -0.5
            self.target_angular = 0.0
        elif key == 'a':
            self.target_linear = 0.0
            self.target_angular = 0.5
        elif key == 'd':
            self.target_linear = 0.0
            self.target_angular = -0.5
        elif key == ' ':
            self.target_linear = 0.0
            self.target_angular = 0.0
        elif key == '\x03':  # Ctrl+C
            sys.exit(0)

        # Create the stamped message
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x = self.target_linear
        msg.twist.angular.z = self.target_angular
        
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    teleop_node = KeyboardTeleop()
    try:
        rclpy.spin(teleop_node)
    except KeyboardInterrupt:
        pass
    finally:
        teleop_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()