import rclpy
from rclpy.node import Node
from rclpy.time import Time
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, Point
from nav2_msgs.action import NavigateToPose, ComputePathToPose
from rclpy.action import ActionClient
import numpy as np
from action_msgs.msg import GoalStatus
from matplotlib import pyplot as plt
import datetime
import tf2_ros
import random
from std_msgs.msg import String, Bool
from rclpy.parameter import Parameter
from plansys2_support_py.ActionExecutorClient import ActionExecutorClient

from shared_interfaces.msg import ExplorationStatus
from .local_utilities import *


distance_delta = 0.5
action_prefix = "INV:"


class InvestigateNode(ActionExecutorClient):
	def __init__(self):
		super().__init__("investigate", 0.5)
		self.get_logger().info(f"{action_prefix}Action node started")

		self.create_subscription(OccupancyGrid, "/map", self.map_callback, 10)
		self.create_subscription(Point, "/anomaly", self.anomaly_callback, 10)
		self.navigation_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
		self.tf_buffer = tf2_ros.Buffer()
		self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

		# Map and position data
		self.map_data: OccupancyGrid = OccupancyGrid()
		self.waypoints: list[list[int]] = []
		self.current_goal_handle = None
		self.investigation_active = False
		self.node_active = False
		self.goal_pointer = 0
		self.anomaly: Point
		self.create_timer(1.0, self.check_proximity)


	def on_activate(self, state):
		self.node_active = True
		self.get_logger().info(f"{action_prefix}Node activated")
		return super().on_activate(state)

	def on_deactivate(self, state):
		self.investigation_active = False
		self.node_active = False
		self.get_logger().info(f"{action_prefix}Node deactivated")
		return super().on_deactivate(state)

	def do_work(self):
		if not self.anomaly:
			self.get_logger().warning(f"{action_prefix}No anomaly data available for inverstigation")
			return
		if (not self.investigation_active) and self.node_active:
			self.investigation_active = True
			self.investigate()


	def map_callback(self, msg: OccupancyGrid):
		if not self.node_active:
			return
		self.map_data = msg

	
	def anomaly_callback(self, msg: Point):
		self.get_logger().info(f"{action_prefix}anomaly_callback: {msg}")
		self.anomaly = msg


	def cancel_goal(self):
		if self.current_goal_handle:
			self.current_goal_handle.cancel_goal_async()


	def check_proximity(self):
		if not self.investigation_active or not self.node_active:
			return
		
		try:
			tf = self.tf_buffer.lookup_transform("map", "base_link", Time())
			robot_x, robot_y = tf.transform.translation.x, tf.transform.translation.y
		except Exception as e:
			self.get_logger().info(f"{action_prefix}robot position lookup failed (bruh) ({e})")
			return
		anomaly_x, anomaly_y = self.anomaly.x, self.anomaly.y
		proximity = ((robot_x - anomaly_x)**2 + (robot_y - anomaly_y)**2)**0.5
		self.anomaly.z += (5 - proximity**2) * (proximity <= 2.2)
		# self.get_logger().info(f"{action_prefix}Progress: {self.anomaly.z}")
		if (self.anomaly.z < 100):
			self.send_feedback(self.anomaly.z / 100, ExplorationStatus.INVESTIGATING)
		else:
			self.node_active = False
			self.waypoints = []
			self.cancel_goal()
			self.finish(True, self.anomaly.z / 100, ExplorationStatus.FINISHED_INVESTIGATING)


	def navigate_to(self, goal: list[int]):
		goal_pose = mappoint_to_coords(goal, self.map_data)
		goal_msg = PoseStamped()
		goal_msg.header.frame_id = "map"
		goal_msg.header.stamp = self.get_clock().now().to_msg()
		goal_msg.pose.position.x = goal_pose[0]
		goal_msg.pose.position.y = goal_pose[1]

		nav_goal = NavigateToPose.Goal()
		nav_goal.pose = goal_msg

		self.get_logger().info(f"{action_prefix}Navigating to goal: {goal_pose}")

		self.navigation_client.wait_for_server()

		try:
			send_goal_future = self.navigation_client.send_goal_async(nav_goal)
			send_goal_future.add_done_callback(self.goal_response_callback)
		except Exception as e:
			self.get_logger().error(f"{action_prefix}Exception during navigation: {e}")


	def goal_response_callback(self, future):
		goal_handle = future.result()
		self.current_goal_handle = goal_handle
		if not goal_handle.accepted:
			self.get_logger().warning(f"{action_prefix}Goal rejected!")
			return
		self.get_logger().info(f"{action_prefix}Goal accepted")
		goal_handle.get_result_async().add_done_callback(
			self.navigation_complete_callback
		)


	def navigation_complete_callback(self, future):
		try:
			result = future.result()
			status = result.status

			if status == GoalStatus.STATUS_ABORTED:
				self.get_logger().error(f"{action_prefix}Navigation to pose aborted: {result.result}")
				self.continue_investigation()
			elif status == GoalStatus.STATUS_CANCELED:
				self.get_logger().warning(
					f"{action_prefix}Navigation to pose canceled: {result.result}"
				)
				self.continue_investigation()
			elif status == GoalStatus.STATUS_SUCCEEDED:
				self.get_logger().info(f"{action_prefix}Navigation to pose succeeded")
				self.continue_investigation()
			else:
				self.get_logger().warning(
					f"{action_prefix}Navigation to pose status unknown: {result}"
				)
		except Exception as e:
			self.get_logger().error(f"{action_prefix}Navigation failed: {e}")
			self.continue_investigation()


	def continue_investigation(self):
		self.goal_pointer = (self.goal_pointer + 1) % 4
		self.investigate()

	
	def investigate(self):
		if not self.node_active:
			self.get_logger().info(f"{action_prefix}Node inactive - action canceled")
			return
		if self.map_data is None or not self.map_data.data:
			self.get_logger().warning(f"{action_prefix}No map data available for inverstigation")
			self.investigation_active = False
			return

		if not self.waypoints:
			self.waypoints = [
				coords_to_mappoint([self.anomaly.x, self.anomaly.y + distance_delta], self.map_data),
				coords_to_mappoint([self.anomaly.x + distance_delta, self.anomaly.y], self.map_data),
				coords_to_mappoint([self.anomaly.x, self.anomaly.y - distance_delta], self.map_data),
				coords_to_mappoint([self.anomaly.x - distance_delta, self.anomaly.y], self.map_data),
			]
		
		self.navigate_to(self.waypoints[self.goal_pointer])
		



def main(args=None):
	rclpy.init(args=args)

	explorer_node = InvestigateNode()
	explorer_node.set_parameters([Parameter(name='action_name', value='investigate')])
	
	explorer_node.trigger_configure()

	rclpy.spin(explorer_node)

	explorer_node.destroy_node()
	rclpy.shutdown()
	return
