# Script to run all launch files and send takeoff command for software demo.

import subprocess
import signal
import sys
import time

# Process handles.
robot_launch = None
human_launch = None
takeoff = None

# Custom signal handler for clean shutdown.
def sigint_handler(sig, frame):
    print "Cleaning up."

    # Register global variables in this scope.
    global robot_launch
    global human_launch

    # Send termination signals and wait for success.
    if robot_launch is not None and robot_launch.poll() is None:
        robot_launch.terminate()

    if human_launch is not None and human_launch.poll() is None:
        human_launch.terminate()

    if takeoff is not None and takeoff.poll() is None:
        takeoff.terminate()

    if robot_launch is not None:
        robot_launch.wait()

    if human_launch is not None:
        human_launch.wait()

    if takeoff is not None:
        takeoff.wait()

    # Exit successfully.
    sys.exit(0)

# Set up processes and let it go.
# (1) Launch the crazyflies.
robot_launch = subprocess.Popen(["roslaunch", "fastpeople", "software_demo.launch"])

# (2) Wait a fixed amount of time (s) before calling takeoff.
TIME_BEFORE_TAKEOFF = 3.0
time.sleep(TIME_BEFORE_TAKEOFF)

# (3) Send takeoff signal.
takeoff = subprocess.Popen(["./takeoff.sh"])

# (4) Wait a fixed amount of time for takeoff to initiate before starting
# human motion predictor.
TIME_BEFORE_HUMAN = 3.0
time.sleep(TIME_BEFORE_HUMAN)

# (5) Initiate human predictor.
human_launch = subprocess.Popen(["roslaunch", "crazyflie_human", "multi_sim.launch"])
