# Detailed analysis of trajectory safety

# run bag file for a large number of (fixed) human trajectories
# for each beta strategy (fixed or adaptive), we want safety rates
# 	for each probability threshold, we want to compare this threshold with
# 		a) for each output plan, would collision have happened with no replans?
# 		b) for each executed trajectory, did a collision actually happen?
# aggregating bag files we have rates of (a) open- and (b) closed-loop safety
#
# hopefully (a) rates reasonably match the predictions made by the Bayesian
# approach, or at least do so to a greater degree than using naive betas
#
# can separate the analysis to show that conditioned on "rational" trajectories
# the fixed high beta is well calibrated and the fixed low beta is poorly
# calibrated (and vice-versa) whereas Bayesian beta is well calibrated across
# the board
#
# hopefully (b) rates are consistently lower than (a) rates due to replanning

import rospy


class SafetyRates(object):

	def __init__(self):

		# create ROS node
		rospy.init_node('safety_rates', anonymous=True)

		# load all the prediction params and setup subscriber/publishers
		self.load_parameters()
		self.register_callbacks()

		# initialize the open-loop (counterfactual) and closed-loop (actual)
		# collision rates (of generated plans and executed trajectories resp.)
		self.n_traj_total = 0
		self.n_traj_crashed = 0
		self.n_plans_total = 0
		self.n_plans_would_have_crashed = 0

		# initialize the robot's recorded state
		self.robot_state = None

		rate = rospy.Rate(100) 

		while not rospy.is_shutdown():
			# plot start/goal markers for visualization
			self.goal_pub.publish(marker_array)

			rate.sleep()

	# TODO: IMPLEMENT
	def load_parameters(self):
		"""
		Loads all the important parameters
		"""

	# TODO: SET UP THE REMAINING PUBLISHERS/SUBSCRIBERS (CF STATE AND *PLAN*)
	def register_callbacks(self):
		"""
		Sets up all the publishers/subscribers needed.
		"""
		# subscribe to the info of the human's current pose
		self.human_sub = rospy.Subscriber('/human_pose'+self.human_number,
											PoseStamped, 
											self.human_state_callback,
											queue_size=1)

	# TODO: IMPLEMENT
	def human_state_callback(self,msg):
		"""
		Checks current human state for collisions with current robot pose
		as well as every plan generated so far during this run that contains
		the current time stamp. Updates collision rates accordingly.
		"""

	# TODO: IMPLEMENT
	def robot_state_callback(self,msg):
		"""
		Updates the internal representation of the robot's current state.
		"""

	def traj_crash_rate(self):
		"""
		Returns current rate of actual crashes of closed-loop trajectories.
		"""
		return n_traj_crashed / float(n_traj_total)

	def plan_crash_rate(self):
		"""
		Returns current rate of hypothetical crashes of open-loop plans.
		"""
		return n_plans_would_have_crashed / float(n_plans_total)

if __name__ == '__main__':
	node = SafetyRates()