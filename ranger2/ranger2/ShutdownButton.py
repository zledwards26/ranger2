#This node was originally for the shutdown button.
#However, I realized that the system could be configured to shutdown on a button press without writing more code
#This node is no longer used.

import subprocess

import rclpy
from rclpy.node import Node
import Jetson.GPIO as GPIO


class ShutdownButtonNode(Node):

    INPUT_PIN = 40  # Physical pin number (BOARD mode)

    def __init__(self):
        super().__init__('shutdown_button')

        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(
            self.INPUT_PIN,
            GPIO.IN,
            pull_up_down=GPIO.PUD_UP
        )

        GPIO.add_event_detect(
            self.INPUT_PIN,
            GPIO.FALLING,
            callback=self.shutdown_callback,
            bouncetime=500
        )

        self.get_logger().info(
            f'I will shutdown if you short {self.INPUT_PIN} to ground'
        )

    def shutdown_callback(self, channel):
        self.get_logger().warn('Shutdown button pressed')

        # Prevent additional callbacks
        GPIO.remove_event_detect(self.INPUT_PIN)

        # Asynchronous so the callback returns immediately
        subprocess.Popen(['shutdown', 'now'])

    def destroy_node(self):
        GPIO.cleanup()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = ShutdownButtonNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
