#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from nav2_msgs.action import FollowPath
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Quaternion
from tf_transformations import quaternion_from_euler
from nav_msgs.msg import Path
from rclpy.time import Time
from rclpy.action import ActionClient
from rclpy.duration import Duration
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException
from ament_index_python.packages import get_package_share_directory
import tf2_geometry_msgs
import math
import os
import json
import re

class FollowPathClient(Node):
    def __init__(self):
        super().__init__('send_follow_path_client')
        # tell ROS2 to use /clock for time
        #self.declare_parameter('use_sim_time', True)
        #self.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self._action_client = ActionClient(self, FollowPath, 'follow_path')
        # tf2 buffer / listener for transforms
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self._vis_pub = self.create_publisher(Path, 'plan', 10)


    def yaw_to_quaternion(self, yaw: float) -> Quaternion:
        """Converts a yaw angle (in radians) to a Quaternion message."""
        q = quaternion_from_euler(0, 0, yaw)  # Roll, Pitch, Yaw
        return Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])


    def send_path(self, poses_base_link):
        # 1) Wait until the FollowPath action server is ready
        self._action_client.wait_for_server()

        # 2) Wait for TF
        self.get_logger().info('Waiting for base_link→map transform…')
        while rclpy.ok():
            try:
                self.tf_buffer.lookup_transform(
                    'map', 'base_link',
                    rclpy.time.Time(),      # latest available
                    timeout=Duration(seconds=0.5)
                )
                break
            except (LookupException, ConnectivityException, ExtrapolationException):
                rclpy.spin_once(self, timeout_sec=0.1)

        # 3) Transform all positions to map frame
        self.get_logger().info('Transforming path from base_link to map frame...')
        map_poses = []
        for (x, y) in poses_base_link:
            ps_bl = PoseStamped()
            ps_bl.header.frame_id = 'base_link'
            ps_bl.header.stamp = Time().to_msg()
            ps_bl.pose.position.x = x
            ps_bl.pose.position.y = y
            try:
                ps_map = self.tf_buffer.transform(ps_bl, 'map', timeout=Duration(seconds=1.0))
                map_poses.append(ps_map)
            except (LookupException, ConnectivityException, ExtrapolationException) as ex:
                self.get_logger().error(f'Failed to transform point ({x:.2f},{y:.2f}): {ex}')
                return # Abort if any transform fails

        # 4) Calculate orientation for each waypoint
        self.get_logger().info('Calculating orientations for the path...')
        for i in range(len(map_poses)):
            if i < len(map_poses) - 1:
                # Point towards the next waypoint
                p_current = map_poses[i].pose.position
                p_next = map_poses[i+1].pose.position
                yaw = math.atan2(p_next.y - p_current.y, p_next.x - p_current.x)
            # For the last point, reuse the orientation of the second-to-last point
            # This ensures a smooth final heading. 'yaw' is already set from the previous iteration.
            map_poses[i].pose.orientation = self.yaw_to_quaternion(yaw)    

        # 5) Build the final Path message
        path = Path()
        path.header.frame_id = 'map'
        path.header.stamp = self.get_clock().now().to_msg()
        path.poses = map_poses

        # 6) Publish the Path for visualization in RViz
        self._vis_pub.publish(path)

        # 7) Send the FollowPath goal
        goal_msg = FollowPath.Goal()
        goal_msg.path = path
        #goal_msg.controller_id = "MPPI" # Specify controller if needed
        self.get_logger().info(f'Sending FollowPath goal with {len(path.poses)} waypoints.')
        return self._action_client.send_goal_async(goal_msg)


def main(args=None):
    rclpy.init(args=args)
    client = FollowPathClient()

    # The original parameter logic is commented out to preserve it, while the new logic is added below.
    # client.declare_parameter("path", "right_turn.json")
    # json_filename = client.get_parameter("path").get_parameter_value().string_value
    client.declare_parameter("path", "")
    json_filename_param = client.get_parameter("path").get_parameter_value().string_value

    share_dir = get_package_share_directory("test_nav2_controller")
    paths_dir = os.path.join(share_dir, "paths")
    json_path = ""

    if json_filename_param:
        # If the parameter is explicitly provided, use it.
        json_filename = json_filename_param
        json_path = os.path.join(paths_dir, json_filename)
        if not os.path.exists(json_path):
            client.get_logger().error(f"File not found: {json_path}")
            rclpy.shutdown()
            return
        client.get_logger().info(f"Using specified path file: {json_path}")
    else:
        client.get_logger().error("No path parameter specified. Please specify a path with -p path:=<filename.json>.")
        rclpy.shutdown()
        return

    # Load and send the path
    try:
        with open(json_path, "r") as f:
            path_points = json.load(f)
        
        # The JSON file contains (x, y, yaw) data, but the original send_path expects (x, y) tuples.
        # We extract just the (x, y) tuples to pass to the unmodified send_path function.
        path_in_base_link = [(pt["x"], pt["y"]) for pt in path_points]
        goal_future = client.send_path(path_in_base_link)

    except (IOError, json.JSONDecodeError) as e:
        client.get_logger().error(f"Failed to load or parse JSON file {json_path}: {e}")
        rclpy.shutdown()
        return

    # Spin until the action server accepts (or rejects) the goal, then exit.
    # The controller keeps executing the path independently after acceptance.
    rclpy.spin_until_future_complete(client, goal_future)
    goal_handle = goal_future.result()
    if goal_handle is None or not goal_handle.accepted:
        client.get_logger().error('Goal was REJECTED by the action server.')
    else:
        client.get_logger().info('Goal accepted. Controller is executing the path.')
    rclpy.shutdown()

if __name__ == '__main__':
    main()
            

    
