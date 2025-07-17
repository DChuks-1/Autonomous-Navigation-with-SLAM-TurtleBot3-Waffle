#!/usr/bin/env python3

"""# URL of map resource
# Can be an absolute path to a file: file:///path/to/maps/floor1.yaml
# Or, relative to a ROS package: package://my_ros_package/maps/floor2.yaml
string map_topic
string map_url
# Constants for image_format. Supported formats: pgm, png, bmp
string image_format
# Map modes: trinary, scale or raw
string map_mode
# Thresholds. Values in range of [0.0 .. 1.0]
float32 free_thresh
float32 occupied_thresh
---
bool result """


import rclpy
from rclpy.node import Node

from nav2_msgs.srv import SaveMap

import argparse 
import time

import os

class MapSaverClient(Node):

    def __init__(self):
        super().__init__('map_saver_client')

        self.client = self.create_client(
            srv_type=SaveMap, 
            srv_name='map_saver/save_map'
        ) 

        cli = argparse.ArgumentParser() 
        cli.add_argument(
            "-f", "--filename", default=os.path.expanduser("~/ros2_ws/src/acs6121_team24_2025/maps/explore_map"), type=str
        ) 
        self.args, _ = cli.parse_known_args() 

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(
                "Waiting for service..."
            ) 

   

    def send_request(self, filename):
            """Send the request to save the map."""
            request = SaveMap.Request()
            request.map_url = filename  # Path to save the map
            request.image_format ='png'

            self.get_logger().info(f"Requesting to save map to: {filename}")

            """ if response is not None and response.success:
                self.get_logger().info(f"Map successfully saved to: {filename}")
            else:
                self.get_logger().error("Failed to save the map!") """

            return self.client.call_async(request)

def main():
    rclpy.init()
    client = MapSaverClient()

    interval = 10  # Time interval in seconds
    client.get_logger().info(f"Saving maps every {interval} seconds...")

    try:
        while rclpy.ok():
            future = client.send_request(client.args.filename)
            rclpy.spin_until_future_complete(client, future)
            response = future.result()
            time.sleep(interval)  # Wait before saving again
    except KeyboardInterrupt:
        client.get_logger().info("Shutting down map saver client...")

    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()