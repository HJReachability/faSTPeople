# Script to run all launch files and send takeoff command for software demo.
# Adapted from David Fridovich-Keil ( dfk@eecs.berkeley.edu )
import subprocess
import signal
import sys
import time

# Process handles.
robot_launch = None
human_launch = None
takeoff = None
data_logger = None

# Custom signal handler for clean shutdown
def sigint_handler(sig, frame):
	print "Cleaning up."

	# Register global variables in this scope.
	global robot_launch
	global human_launch

	# Send termination signals and wait for success.
	if robot_launch is not None:
		if robot_launch.poll() is None:
			robot_launch.terminate()
		else:
			robot_launch.wait()

	if human_launch is not None:
		if human_launch.poll() is None:
			human_launch.terminate()
		else:
			human_launch.wait()

	if takeoff is not None:
		if takeoff.poll() is None:
			takeoff.terminate()
		else:
			takeoff.wait()

	if data_logger is not None:
		if data_logger.poll() is None:
			data_logger.terminate()
		else:
			data_logger.wait()

	# Exit successfully.
	sys.exit(0)

# Register custom signal handler on SIGINT.
signal.signal(signal.SIGINT, sigint_handler)

# Set up processes and let it go.

# (1) Launch the crazyflies.
robot_launch = subprocess.Popen(["roslaunch", "fastpeople", "software_demo_single.launch"])

# (2) Wait a fixed amount of time (s) before calling the data logger.
TIME_BEFORE_LOGGER = 1.0
time.sleep(TIME_BEFORE_LOGGER)

# (3) Initiate data logger.
data_logger = subprocess.Popen(["roslaunch", "data_logger", "logger.launch"])

# (4) Wait a fixed amount of time (s) before calling takeoff.
TIME_BEFORE_TAKEOFF = 1.0
time.sleep(TIME_BEFORE_TAKEOFF)

# (5) Send takeoff signal.
takeoff = subprocess.Popen(["rosservice", "call", "takeoffHY4"])

# (6) Wait a fixed amount of time for takeoff to initiate before starting human motion predictor.
TIME_BEFORE_HUMAN = 1.0
time.sleep(TIME_BEFORE_HUMAN)

# (7) Initiate human predictor.
human_launch = subprocess.Popen(["roslaunch", "crazyflie_human", "simulated_demo_single.launch"])

signal.pause()

