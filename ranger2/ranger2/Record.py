#!/usr/bin/env python3

from datetime import datetime

from pathlib import Path

import rclpy
from rclpy.node import Node

from zed_msgs.srv import StartSvoRec


class SvoRecorder(Node):

    def __init__(self):
        super().__init__('svo_recorder')

        #Create a client to use the SVO recording service
        self.client = self.create_client(
            StartSvoRec,
            '/zed/zed_node/start_svo_rec'
        )

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for SVO recording service...')

        
        #create request object to send to ZED Wrapper
        req = StartSvoRec.Request()

        #Create path to store SVO file
        timestamp = datetime.now().strftime('%Y-%b-%d_%H-%M-%S')

        recording_dir = Path.home() / "svo_recordings" #note: pathlib overwrites the / operator

        #make the directory if necesary
        recording_dir.mkdir(parents=True, exist_ok=True)

        req.svo_filename = str(
            recording_dir / f"{timestamp}.svo" 
        )
        
        
        
        #Configure request
        req.bitrate = 0
        req.compression_mode = 1
        req.target_framerate = 0
        req.input_transcode = False


        #send the request
        future = self.client.call_async(req)
        future.add_done_callback(self.service_response)

    
    def service_response(self, future):
        try:
            response = future.result()

            if response.success:
                self.get_logger().info("SVO recording started")
            else:
                self.get_logger().error(
                    f"Failed to start recording: {response.message}"
                )

        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")

        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)

    node = SvoRecorder()
    rclpy.spin(node)


if __name__ == '__main__':
    main()