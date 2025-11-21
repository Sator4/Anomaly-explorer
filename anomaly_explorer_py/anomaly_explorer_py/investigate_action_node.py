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


class InvestigateNode(ActionExecutorClient):
	def __init__(self):
		super().__init__("investigate", 0.5)
		self.get_logger().info("Investigate action node started")
		self.investigation_status = 0.0


	def on_activate(self, state):
		return super().on_activate(state)
	

	def do_work(self):
		self.get_logger().info(f"logging: {self.investigation_status}")
		self.investigation_status += 0.1
		if self.investigation_status >= 1:
			self.finish(True, self.investigation_status, ExplorationStatus.FINISHED_INVESTIGATING)
			return
		self.send_feedback(self.investigation_status, ExplorationStatus.INVESTIGATING)

	def on_deactivate(self, state):
		self.investigation_status = 0.0
		return super().on_deactivate(state)



def main(args=None):
	rclpy.init(args=args)

	explorer_node = InvestigateNode()
	explorer_node.set_parameters([Parameter(name='action_name', value='investigate')])
	
	explorer_node.trigger_configure()

	rclpy.spin(explorer_node)

	explorer_node.destroy_node()
	rclpy.shutdown()
	return
