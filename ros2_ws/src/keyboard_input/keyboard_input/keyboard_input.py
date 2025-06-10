import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class KeyboardInputPublisher(Node):
    """Some text."""

    def __init__(self):
        """Initialize the keyboard input publisher node."""
        super().__init__('keyboard_input_publisher')

        # Declare parameters
        self.declare_parameter('topic_name', 'keyboard_input')
        self.declare_parameter('queue_size', 10)

        # Get parameters
        topic_name = self.get_parameter(
            'topic_name').get_parameter_value().string_value
        queue_size = self.get_parameter(
            'queue_size').get_parameter_value().integer_value

        # Create publisher
        self.publisher = self.create_publisher(String, topic_name, queue_size)

        self.get_logger().info(
            f'Keyboard input publisher started on topic: {topic_name}')
        self.get_logger().info(
            'Type your messages. Press Enter to publish, Ctrl+C to exit.')

        # Start keyboard input thread
        self.input_thread = threading.Thread(target=self.keyboard_input_loop,
                                             daemon=True)
        self.input_thread.start()

    def keyboard_input_loop(self):
        """Handle keyboard input in a separate thread."""
        try:
            while rclpy.ok():
                try:
                    # Get input from user
                    user_input = input('Enter message: ').strip()

                    if user_input:
                        # Create and publish message
                        msg = String()
                        msg.data = user_input
                        self.publisher.publish(msg)

                        self.get_logger().info(f'Published: "{user_input}"')

                except EOFError:
                    # Handle Ctrl+D
                    self.get_logger().info('EOF received, shutting down...')
                    break
                except KeyboardInterrupt:
                    # Handle Ctrl+C
                    self.get_logger().info(
                        'Keyboard interrupt received, shutting down...')
                    break

        except Exception as e:
            self.get_logger().error(f'Error in keyboard input loop: {str(e)}')


def main(args=None):
    """Initialize the ROS 2 node."""
    # Initialize ROS 2
    rclpy.init(args=args)

    try:
        # Create node
        node = KeyboardInputPublisher()

        # Spin the node
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
    finally:
        # Cleanup
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
