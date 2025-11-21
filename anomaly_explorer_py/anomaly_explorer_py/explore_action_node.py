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


class ExploreNode(ActionExecutorClient):
	def __init__(self):
		super().__init__("explore", 0.5)
		self.get_logger().info("Explore action node started")

		self.exploration_status = 0.0


	def do_work(self):
		self.exploration_status += 0.025
		anomaly = random.random() > 0.9
		if anomaly:
			self.send_feedback(self.exploration_status, ExplorationStatus.ANOMALY_FOUND)
			return
		if self.exploration_status >= 1:
			self.finish(True, self.exploration_status, ExplorationStatus.FINISHED_EXPLORING)
			return
		self.send_feedback(self.exploration_status, ExplorationStatus.EXPLORING)
	
	def on_activate(self, state):
		return super().on_activate(state)

	def on_deactivate(self, state):
		return super().on_deactivate(state)



def main(args=None):
	rclpy.init(args=args)

	explorer_node = ExploreNode()
	explorer_node.set_parameters([Parameter(name='action_name', value='explore')])
	
	explorer_node.trigger_configure()

	rclpy.spin(explorer_node)

	explorer_node.destroy_node()
	rclpy.shutdown()
	return
