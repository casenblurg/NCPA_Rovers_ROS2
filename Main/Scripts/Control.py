import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import termios
import sys
import tty


class KeyboardControl(Node):
    def __init__(self):
        super().__init__('keyboard_control')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info("Node initialized. Use keys to control the rover. (w/a/s/d to move, q to quit)")

    def get_key(self):
        """Reads a single character from the terminal."""
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            key = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return key

    def run(self):
        try:
            while rclpy.ok():
                key = self.get_key()
                twist = Twist()

                if key == 'w':  # Move forward
                    twist.linear.x = 0.1
                elif key == 's':  # Move backward
                    twist.linear.x = -0.1
                elif key == 'a':  # Turn left
                    twist.angular.z = 0.5
                elif key == 'd':  # Turn right
                    twist.angular.z = -0.5
                elif key == 'q':  # Quit
                    self.get_logger().info("Exiting...")
                    break
                else:
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0

                # Publish the Twist message
                self.publisher.publish(twist)
                self.get_logger().info(f"Published: linear.x={twist.linear.x}, angular.z={twist.angular.z}")
        except Exception as e:
            self.get_logger().error(f"Error in run loop: {e}")


def main():
    rclpy.init()  # Initialize ROS 2 Python library
    node = KeyboardControl()

    try:
        node.run()  # Start the keyboard control loop
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt, shutting down...")
    finally:
        node.destroy_node()  # Clean up the node
        rclpy.shutdown()  # Shut down ROS 2


if __name__ == "__main__":
    main()

