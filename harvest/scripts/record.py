#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from harvest_interfaces.srv import RecordTopics
import os
from datetime import datetime
import subprocess
import signal
import time

class RecordTopicsNode(Node):
    def __init__(self):
        super().__init__('record_topics_node')
        self.bag_process = None
        self.record_service = self.create_service(RecordTopics, 'record_topics', self.start_recording_callback)
        self.stop_service = self.create_service(Trigger, 'stop_recording', self.stop_recording_callback)

        self.get_logger().info('Recording services are ready.')

    def start_recording_callback(self, request, response):
        topics = request.topics
        file_name_prefix = request.file_name_prefix
        if not topics:
            response.success = False
            self.get_logger().warn('No topics provided for recording.')
            return response

        self.bag_path = self.create_bag_file_path(file_name_prefix)
        command = f"ros2 bag record {' '.join(topics)} --output {self.bag_path}"

        # Start the recording process
        self.bag_process = subprocess.Popen(command, shell=True, preexec_fn=os.setsid)
        response.success = True
        self.get_logger().info(f'Started recording topics: {topics} to {self.bag_path}')
        return response

    def stop_recording_callback(self, request, response):
        if self.bag_process:
            os.killpg(os.getpgid(self.bag_process.pid), signal.SIGTERM)  # Terminate the process group
            time.sleep(1)  # Give it a moment to clean up

            # Check if the process is still alive
            if self.bag_process.poll() is None:  # Process is still running
                self.get_logger().warn('Bag process did not terminate immediately; forcing termination.')
                self.bag_process.kill()  # Force kill if not stopped
            self.bag_process = None
            response.success = True
            self.get_logger().info('Stopped recording and saved the bag file.')
        else:
            response.success = False
            self.get_logger().warn('No recording in progress.')
        return response

    def create_bag_file_path(self, file_name_prefix):
        # Generate a timestamped bag file name
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        bag_file_name = file_name_prefix + f"_{timestamp}.db3"
        return bag_file_name

def main(args=None):
    rclpy.init(args=args)
    record_topics_node = RecordTopicsNode()

    try:
        rclpy.spin(record_topics_node)
    except KeyboardInterrupt:
        record_topics_node.get_logger().info('Node interrupted.')
    finally:
        # Ensure that the recording process is stopped before shutting down
        if record_topics_node.bag_process:
            record_topics_node.stop_recording_callback(None, None)  # Call stop recording method

        record_topics_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
