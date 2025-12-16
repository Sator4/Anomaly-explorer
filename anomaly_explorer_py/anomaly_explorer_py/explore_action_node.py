import rclpy
from rclpy.node import Node
from rclpy.time import Time
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, Point
from nav2_msgs.action import NavigateToPose, ComputePathToPose
from rclpy.action import ActionClient
import numpy as np
from action_msgs.msg import GoalStatus
from matplotlib import pyplot as plt
import datetime
import tf2_ros
from rclpy.parameter import Parameter
from plansys2_support_py.ActionExecutorClient import ActionExecutorClient
from shared_interfaces.msg import ExplorationStatus
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from rcl_interfaces.msg import ParameterDescriptor
import os
from .local_utilities import *


files_explorer_py_dir = "/home/sator/ros2_program/ros2_ws/anomaly_explorer/src/files"

mock_anomalies: list[list[float]] = [[5.403, -8.064, 0], [2.57, -2.366, 0], [8.13, -6.52, 0]]   # x -1..9 y -9..1
# mock_anomalies: list[list[float]] = [[0, 0, 0], [1, 0, 0], [0, -1, 0]]   # x -1..9 y -9..1
distance_delta = 0.5  # check on every map update, if the anomaly is in the explored region, it is detected
    				  # the anomaly needs to get 100 points to be investigated, it floor(gets distance / distance_delta)*10 points per second

action_prefix = "EXP:"

# ARGUMENTS = [
#     DeclareLaunchArgument("x", default_value="0.0"),
#     DeclareLaunchArgument("y", default_value="0.0")
# ]


class ExploreNode(ActionExecutorClient):
	def __init__(self):
		super().__init__("explore", 0.5)
		self.declare_parameter("x", 0.0, ParameterDescriptor(dynamic_typing=True))
		self.declare_parameter("y", 0.0, ParameterDescriptor(dynamic_typing=True))
		x_offset = float(self.get_parameter("x").value) # type: ignore
		y_offset = float(self.get_parameter("y").value) # type: ignore
		for i in range(len(mock_anomalies)):
			mock_anomalies[i][0] -= x_offset
			mock_anomalies[i][1] -= y_offset

		self.get_logger().info(f"{action_prefix}Action node started")
		
		self.create_subscription(OccupancyGrid, "/map", self.map_callback, 10)
		self.create_subscription(Path, "/plan", self.plan_callback, 10)
		self.anomaly_pub = self.create_publisher(Point, "/anomaly", 10)
		self.navigation_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
		self.compute_path_client = ActionClient(self, ComputePathToPose, "compute_path_to_pose")
		self.tf_buffer = tf2_ros.Buffer()
		self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

		# Map and position data
		self.map_data: OccupancyGrid = OccupancyGrid()
		self.map_array: list[list[int]] = []
		self.robot_position = (0, 0)  # Placeholder, update from localization
		self.waypoints = []
		self.last_waypoints = []
		self.exploration_active = False
		self.node_active = False
		self.current_goal_handle = None
		self.recalculate_waypoints = False
		self.recheck_waypoints = False
		self.plan_callback_timeout = 2

	def on_activate(self, state):
		self.node_active = True
		self.get_logger().info(f"{action_prefix}Node activated")
		return super().on_activate(state)

	def on_deactivate(self, state):
		self.node_active = False
		self.exploration_active = False
		self.get_logger().info(f"{action_prefix}Node deactivated")
		return super().on_deactivate(state)

	def do_work(self):
		if (not self.exploration_active) and self.node_active:
			self.exploration_active = True
			self.explore()


	def map_callback(self, msg):
		if not self.node_active:
			return
		if self.map_data.data and (np.array(self.map_data.data).size != np.array(msg.data).size):
			self.recalculate_waypoints = True  # если карта другого размера - пересчитать вейпоинты
		else:
			self.recheck_waypoints = True  # если карта такого же размера, просто перепроверить их
		if (self.waypoints):
			self.recheck_goal_new_map(msg)   # здесь обновляется self.map_array и проверяется цель, потом на новом шаге запускается
		self.map_data = msg				 # перепроверка перед выбором новой цели. 


	def recheck_goal_new_map(self, map_data: OccupancyGrid): # при обновлении карты проверяется, не находится ли запланированная цель на неправильной территории
		goal = self.waypoints[0]
		self.map_array = list(np.array(map_data.data).reshape(
			(map_data.info.height, map_data.info.width))
		)
		self.check_anomalies()
		if (self.map_array[goal[0]][goal[1]] != -1 or self.recalculate_waypoints):
			self.cancel_goal()

	
	def plan_callback(self, msg: Path):   # при обновлении плана проверяется, не находится ли истинная цель на неправильной территории
		if not self.node_active or not self.exploration_active:
			return
		goal: Point = msg.poses[-1].pose.position  # pyright: ignore[reportIndexIssue]
		plan_goal = coords_to_mappoint([goal.x, goal.y], self.map_data)
		if self.map_array[plan_goal[0]][plan_goal[1]] != -1:
			self.cancel_goal()


	def check_reachability(self):
		goal_pose = mappoint_to_coords(self.waypoints[0], self.map_data)
		goal_msg = ComputePathToPose.Goal()
		goal_msg.goal.header.frame_id = "map"
		goal_msg.goal.header.stamp = self.get_clock().now().to_msg()
		goal_msg.goal.pose.position.x = goal_pose[0]
		goal_msg.goal.pose.position.y = goal_pose[1]
		self.compute_path_client.wait_for_server()
		send_goal_future = self.compute_path_client.send_goal_async(goal_msg)
		send_goal_future.add_done_callback(self.check_reachibility_callback)

	
	def check_reachibility_callback(self, future):
		goal_handle = future.result()
		if not goal_handle.accepted:
			self.remove_waypoint()
			return
		result_future = goal_handle.get_result_async()
		result_future.add_done_callback(self.check_reachability_result)

	
	def check_reachability_result(self, future):
		result = future.result().result
		if result.path.poses == []:
			self.remove_waypoint()
			return
		result_goal_point = coords_to_mappoint([result.path.poses[-1].pose.position.x, result.path.poses[-1].pose.position.y], self.map_data)
		if self.map_array[result_goal_point[0]][result_goal_point[1]] != -1:
			self.remove_waypoint()
			return
		self.navigate_to()


	def navigate_to(self):
		goal_pose = mappoint_to_coords(self.waypoints[0], self.map_data)
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
			send_goal_future = self.navigation_client.send_goal_async(nav_goal, feedback_callback=self.goal_feedback_callback)
			send_goal_future.add_done_callback(self.goal_response_callback)
		except Exception as e:
			self.get_logger().error(f"{action_prefix}Exception during navigation: {e}")


	def goal_feedback_callback(self, feedback_msg):
		if feedback_msg.feedback.number_of_recoveries > 0:
			self.cancel_goal()


	def goal_response_callback(self, future):
		goal_handle = future.result()
		self.current_goal_handle = goal_handle
		if not goal_handle.accepted:
			self.get_logger().warning(f"{action_prefix}Goal rejected!")
			self.remove_waypoint()
			return
		# self.get_logger().info(f"{action_prefix}Goal accepted")

		goal_handle.get_result_async().add_done_callback(
			self.navigation_complete_callback
		)


	def navigation_complete_callback(self, future):
		try:
			result = future.result()
			status = result.status

			if status == GoalStatus.STATUS_ABORTED:
				self.get_logger().error(f"{action_prefix}Navigation to pose aborted: {result.result}")
				self.remove_waypoint()
			elif status == GoalStatus.STATUS_CANCELED:
				self.get_logger().warning(
					f"{action_prefix}Navigation to pose canceled: {result.result}"
				)
				self.remove_waypoint()
			elif status == GoalStatus.STATUS_SUCCEEDED:
				self.get_logger().info(f"{action_prefix}Navigation to pose succeeded")
				self.remove_waypoint()
			else:
				self.get_logger().warning(
					f"{action_prefix}Navigation to pose status unknown: {result.status}"
				)
		except Exception as e:
			self.get_logger().error(f"{action_prefix}Navigation failed: {e}")
			self.remove_waypoint()


	def cancel_goal(self):
		if self.current_goal_handle:
			self.current_goal_handle.cancel_goal_async()


	def remove_waypoint(self):
		if self.waypoints == []:
			return
		self.waypoints.pop(0)
		self.get_logger().info(f"{action_prefix}Waypoint removed ({len(self.waypoints)} left)")
		self.explore()


	def generate_waypoints(self, interval=10):
		waypoints = []
		rows, cols = len(self.map_array), len(self.map_array[0])
		for r in range(1, rows - 1, interval):
			for c in range(1, cols - 1, interval):
				if self.map_array[r][c] == -1:
					waypoints.append([r, c])
		self.recalculate_waypoints = False
		self.get_logger().info(f"{action_prefix}Generated {len(waypoints)} waypoints")
		# self.show_picture(map=self.map_array, name_suffix="gen", points_regular=waypoints, points_important=[waypoints[0]])
		return waypoints
	

	def revise_waypoints(self, waypoints):
		new_waypoints = []
		for waypoint in waypoints:
			if self.map_array[waypoint[0]][waypoint[1]] == -1:
				new_waypoints.append(waypoint)

		# self.show_picture(map=self.map_array, name_suffix="1", points_regular=waypoints, points_important=[waypoints[0]])
		# if len(waypoints) - len(new_waypoints) > 0:
		# 	if waypoints != []:
		# 		self.show_picture(map=self.map_array, name_suffix="2", points_regular=new_waypoints, points_important=[new_waypoints[0]])
		# 	else:
		# 		self.show_picture(map=self.map_array, name_suffix="2", points_regular=new_waypoints)
	
		self.get_logger().info(f"{action_prefix}Revised waypoints: removed {len(waypoints) - len(new_waypoints)} waypoints ({len(new_waypoints)} left)")
		self.recheck_waypoints = False
		return new_waypoints
	
	
	def check_anomalies(self):
		if not mock_anomalies:
			return
		for i in range(len(mock_anomalies)):
			anomaly_crd = mock_anomalies[i]
			anomaly_mpp = coords_to_mappoint(anomaly_crd, self.map_data)
			try:
				if (self.map_array[anomaly_mpp[0]][anomaly_mpp[1]] == 0):
					self.node_active = False
					self.anomaly_pub.publish(Point(x = float(anomaly_crd[0]), y = float(anomaly_crd[1]), z = float(anomaly_crd[2])))
					mock_anomalies.remove(anomaly_crd)
					self.cancel_goal()
					self.send_feedback(0.0, ExplorationStatus.ANOMALY_FOUND)
					return
			except IndexError:
				continue
	

	def show_picture(self, map:list[list[int]], name_suffix="reg", points_regular:list[list[int]]=[], points_important:list[list[int]]=[]):
		show = [[(i+1)**0.4 for i in j] for j in map]
		for point in points_regular:
			show[point[0]][point[1]] = np.array(show).max() / 2
		for point in points_important:
			show[point[0]][point[1]] = np.array(show).max() * 2
		show = np.array(show[::-1]).transpose()[::-1]
		time = str(datetime.datetime.now().timestamp())
		imspath = os.path.join(files_explorer_py_dir, "images", time)
		plt.imsave(f"{imspath}-{name_suffix}.png", show, cmap = "cubehelix")
		# self.get_logger().info(f"{action_prefix}imvsave {time}")


	def explore(self):
		if not self.node_active:
			self.get_logger().info(f"{action_prefix}Node inactive - action canceled")
			return
		# if self.map_data is None or not self.map_data.data:
		if self.map_data is None or not self.map_data.data:
			self.get_logger().warning(f"{action_prefix}No map data available")
			self.exploration_active = False
			return
		
		if not self.map_array:
			self.map_array = list(np.array(self.map_data.data).reshape(
				(self.map_data.info.height, self.map_data.info.width))
			)

		if not self.waypoints or self.recalculate_waypoints:
			self.waypoints = self.generate_waypoints(10)
			if self.waypoints == self.last_waypoints:
				self.finish(True, 1.0, ExplorationStatus.FINISHED_EXPLORING)
				return
			else:
				self.last_waypoints = [i for i in self.waypoints]
		elif self.recheck_waypoints:
			self.waypoints = self.revise_waypoints(self.waypoints)

		if self.waypoints == []:
			self.finish(True, 1.0, ExplorationStatus.FINISHED_EXPLORING)
			return

		self.check_reachability()



def main(args=None):
	rclpy.init(args=args)

	explorer_node = ExploreNode()
	explorer_node.set_parameters([Parameter(name="action_name", value="explore")])
	
	explorer_node.trigger_configure()

	rclpy.spin(explorer_node)

	explorer_node.destroy_node()
	rclpy.shutdown()
	return
