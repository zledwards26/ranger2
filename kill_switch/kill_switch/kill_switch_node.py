import rclpy
from rclpy.node import Node
from mavros_msgs.srv import CommandBool
import Jetson.GPIO as GPIO
import subprocess
import os
import signal
import time

KILL_PIN = 7

class KillSwitchNode(Node):
    def __init__(self):
        super().__init__('cube_kill_switch')

        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        while not self.arming_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('Waiting for /mavros/cmd/arming service...')

        self.waypoint_process = None
        self.current_state = None

        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(KILL_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.add_event_detect(KILL_PIN, GPIO.BOTH, callback=self.on_change, bouncetime=300)

        self.get_logger().info('Kill switch node ready, listening on pin 7...')

    def call_arming(self, value: bool):
        req = CommandBool.Request()
        req.value = value
        future = self.arming_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
        if future.done():
            result = future.result()
            self.get_logger().info(f'Arming call (value={value}) success={result.success}')
        else:
            self.get_logger().warn('Arming call did not complete in time')

    def stop_waypoint_process(self):
        if self.waypoint_process is not None and self.waypoint_process.poll() is None:
            os.killpg(os.getpgid(self.waypoint_process.pid), signal.SIGTERM)
            try:
                self.waypoint_process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(os.getpgid(self.waypoint_process.pid), signal.SIGKILL)
        self.waypoint_process = None
        subprocess.run(["pkill", "-9", "-f", "hardcoded_waypoint"])

    def start_waypoint_process(self):
        launch_cmd = (
            "source /opt/ros/humble/setup.bash && "
            "source /home/ocecengineering/ros2_ws/install/setup.bash && "
            "exec ros2 run robosub_waypoint hardcoded_waypoint"
        )
        self.waypoint_process = subprocess.Popen(
            ["bash", "-c", launch_cmd],
            preexec_fn=os.setsid
        )

    def on_change(self, channel):
        time.sleep(0.05)
        state = GPIO.input(KILL_PIN)
        new_state = "disarmed" if state == GPIO.LOW else "armed"

        if new_state == self.current_state:
            return

        self.current_state = new_state

        if new_state == "disarmed":
            self.get_logger().info('BUTTON PRESSED — disarming and stopping waypoint process')
            self.stop_waypoint_process()
            self.call_arming(False)
        else:
            self.get_logger().info('BUTTON RELEASED — arming and restarting waypoint process')
            self.call_arming(True)
            self.start_waypoint_process()

    def destroy_node(self):
        self.stop_waypoint_process()
        GPIO.cleanup()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = KillSwitchNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
