import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
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
from ament_index_python.packages import get_package_share_directory
import os


files_explorer_py_dir = '/home/sator/ros2_program/ros2_ws/anomaly_explorer/src/files'


class ExploreNode(ActionExecutorClient):
	def __init__(self):
		super().__init__("explore", 0.5)
		self.get_logger().info("Explore action node started")

		self.create_subscription(OccupancyGrid, "/map", self.map_callback, 10)
		self.navigation_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
		self.compute_path_client = ActionClient(self, ComputePathToPose, 'compute_path_to_pose')
		self.tf_buffer = tf2_ros.Buffer()
		self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

		# Map and position data
		self.map_data = None
		self.robot_position = (0, 0)  # Placeholder, update from localization
		self.waypoints = []
		self.last_waypoints = []
		self.exploration_active = False
		self.current_goal_handle = None
		self.recalculate_waypoints = False
		self.recheck_waypoints = False

	def on_activate(self, state):
		return super().on_activate(state)

	def on_deactivate(self, state):
		return super().on_deactivate(state)

	def do_work(self):
		if not self.exploration_active:
			self.exploration_active = True
			self.explore()

	def map_callback(self, msg):
		if self.map_data and np.array(self.map_data.data).size != np.array(msg.data).size:
			self.recalculate_waypoints = True
		else:
			self.recheck_waypoints = True
			if (self.waypoints):
				self.recheck_goal(msg)
		if not self.waypoints:
			self.get_logger().info("Map received")
		self.map_data = msg


	
	def check_reachability(self):
		goal_pose = self.mappoint_to_coords(self.waypoints[0])
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
		result_goal_point = [result.path.poses[-1].pose.position.x, result.path.poses[-1].pose.position.y]
		actual_goal_point = self.mappoint_to_coords(self.waypoints[0])
		deviation = ((result_goal_point[0] - actual_goal_point[0])**2 + (result_goal_point[1] - actual_goal_point[1])**2)**0.5
		self.get_logger().info(f"Reachability: {result_goal_point}, {actual_goal_point}, {deviation}")
		if result.path.poses == [] or deviation > 10**-6:
			self.get_logger().warn("Waypoint is NOT reachable")
			self.remove_waypoint()
			return
		self.get_logger().info(F"Waypoint is reachable")
		self.navigate_to()


	def navigate_to(self):
		goal_pose = self.mappoint_to_coords(self.waypoints[0])
		goal_msg = PoseStamped()
		goal_msg.header.frame_id = "map"
		goal_msg.header.stamp = self.get_clock().now().to_msg()
		goal_msg.pose.position.x = goal_pose[0]
		goal_msg.pose.position.y = goal_pose[1]

		nav_goal = NavigateToPose.Goal()
		nav_goal.pose = goal_msg

		self.get_logger().info(f"Navigating to goal: {goal_pose}")

		self.navigation_client.wait_for_server()

		try:
			send_goal_future = self.navigation_client.send_goal_async(nav_goal, feedback_callback=self.goal_feedback_callback)
			send_goal_future.add_done_callback(self.goal_response_callback)
		except Exception as e:
			self.get_logger().error(f"Exception during navigation: {e}")


	def goal_feedback_callback(self, feedback_msg):
		if feedback_msg.feedback.number_of_recoveries > 0:
			self.cancel_goal()


	def goal_response_callback(self, future):
		goal_handle = future.result()
		self.current_goal_handle = goal_handle
		if not goal_handle.accepted:
			self.get_logger().warning("Goal rejected!")
			self.remove_waypoint()
			return
		self.get_logger().info("Goal accepted")

		goal_handle.get_result_async().add_done_callback(
			self.navigation_complete_callback
		)


	def navigation_complete_callback(self, future):
		try:
			result = future.result()
			status = result.status

			if status == GoalStatus.STATUS_ABORTED:
				self.get_logger().error(f"Navigation to pose aborted: {result.result}")
				self.remove_waypoint()
			elif status == GoalStatus.STATUS_CANCELED:
				self.get_logger().warning(
					f"Navigation to pose canceled: {result.result}"
				)
				self.remove_waypoint()
			elif status == GoalStatus.STATUS_SUCCEEDED:
				self.get_logger().info(f"Navigation to pose succeeded")
				self.remove_waypoint()
			else:
				self.get_logger().warning(
					f"Navigation to pose status unknown: {result.status}"
				)
		except Exception as e:
			self.get_logger().error(f"Navigation failed: {e}")
			self.remove_waypoint()


	def cancel_goal(self):
		if self.current_goal_handle:
			self.current_goal_handle.cancel_goal_async()


	def remove_waypoint(self):
		if self.waypoints == []:
			return
		self.waypoints.pop(0)
		self.get_logger().info(f"Waypoint removed ({len(self.waypoints)} left)")
		self.explore()


	def generate_waypoints(self, map_array, interval=10):
		waypoints = []
		rows, cols = map_array.shape
		for r in range(1, rows - 1, interval):
			for c in range(1, cols - 1, interval):
				if map_array[r, c] == -1:
					waypoints.append([r, c])
		self.recalculate_waypoints = False
		self.get_logger().info(f"Generated {len(waypoints)} waypoints")
		# self.show_picture(map=map_array, name_suffix='gen', points_regular=waypoints, points_important=[waypoints[0]])
		return waypoints
	

	def revise_waypoints(self, waypoints, map_array):
		new_waypoints = []
		for waypoint in waypoints:
			if map_array[waypoint[0], waypoint[1]] == -1:
				new_waypoints.append(waypoint)

		# self.show_picture(map=map_array, name_suffix='1', points_regular=waypoints, points_important=[waypoints[0]])
		# if len(waypoints) - len(new_waypoints) > 0:
		# 	if waypoints != []:
		# 		self.show_picture(map=map_array, name_suffix='2', points_regular=new_waypoints, points_important=[new_waypoints[0]])
		# 	else:
		# 		self.show_picture(map=map_array, name_suffix='2', points_regular=new_waypoints)
	
		self.get_logger().info(f"Revised waypoints: removed {len(waypoints) - len(new_waypoints)} waypoints ({len(new_waypoints)} left)")
		self.recheck_waypoints = False
		return new_waypoints
	

	def recheck_goal(self, map_data):
		map_array = np.array(map_data.data).reshape(
			(map_data.info.height, map_data.info.width)
		)
		goal = self.waypoints[0]
		if (map_array[goal[0], goal[1]] != -1 or self.recalculate_waypoints):
			self.cancel_goal()
	

	def mappoint_to_coords(self, mappoint):
		goal_x = mappoint[1] * self.map_data.info.resolution + self.map_data.info.origin.position.x
		goal_y = mappoint[0] * self.map_data.info.resolution + self.map_data.info.origin.position.y
		return [goal_x, goal_y]
	
	def coords_to_mappoint(self, coord):
		if len(coord) < 2:
			return
		coord_x = round((coord[0] - self.map_data.info.origin.position.x) / self.map_data.info.resolution)
		coord_y = round((coord[1] - self.map_data.info.origin.position.y) / self.map_data.info.resolution)
		return [coord_y, coord_x]
	

	def show_picture(self, map, name_suffix='reg', points_regular=[], points_important=[]):
		show = [[(i+1)**0.4 for i in j] for j in map]
		for point in points_regular:
			show[point[0]][point[1]] = np.array(show).max() / 2
		for point in points_important:
			show[point[0]][point[1]] = np.array(show).max() * 2
		show = np.array(show[::-1]).transpose()[::-1]
		imspath = os.path.join(files_explorer_py_dir, 'images', str(datetime.datetime.now().timestamp()))
		plt.imsave(f'{imspath}-{name_suffix}.png', show, cmap = 'cubehelix')


	def explore(self):
		if self.map_data is None:
			self.get_logger().warning("No map data available")
			self.exploration_active = False
			return

		map_array = np.array(self.map_data.data).reshape(
			(self.map_data.info.height, self.map_data.info.width)
		)

		if not self.waypoints or self.recalculate_waypoints:
			self.waypoints = self.generate_waypoints(map_array, 10)
			if self.waypoints == self.last_waypoints:
				self.finish(True, 1.0, ExplorationStatus.FINISHED_EXPLORING)
				return
			else:
				self.last_waypoints = [i for i in self.waypoints]
		elif self.recheck_waypoints:
			self.waypoints = self.revise_waypoints(self.waypoints, map_array)

		if self.waypoints == []:
			self.finish(True, 1.0, ExplorationStatus.FINISHED_EXPLORING)
			return

		# self.show_picture(map=map_array, name_suffix='ex', points_important=[self.waypoints[0]])
		self.check_reachability()



def main(args=None):
	rclpy.init(args=args)

	explorer_node = ExploreNode()
	explorer_node.set_parameters([Parameter(name='action_name', value='explore')])
	
	explorer_node.trigger_configure()

	rclpy.spin(explorer_node)

	explorer_node.destroy_node()
	rclpy.shutdown()
	return
