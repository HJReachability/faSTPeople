"""
Copyright (c) 2018, The Regents of the University of California (Regents).
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are
met:

   1. Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.

   2. Redistributions in binary form must reproduce the above
      copyright notice, this list of conditions and the following
      disclaimer in the documentation and/or other materials provided
      with the distribution.

   3. Neither the name of the copyright holder nor the names of its
      contributors may be used to endorse or promote products derived
      from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS AS IS
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.

Please contact the author(s) of this library if you have any questions.
Authors: David Fridovich-Keil   ( dfk@eecs.berkeley.edu )
         Andrea Bajcsy          ( abajcsy@eecs.berkeley.edu )
"""

################################################################################
#
# Class to listen for trajectory requests, human poses, and robot poses,
# compute metrics, and dump to disk.
#
################################################################################

import rospy
import geometry_msgs.msg
import fastrack_msgs.msg
import crazyflie_msgs.msg
import rospkg

import numpy as np
import cPickle as pickle

class DataLogger(object):
    def __init__(self):
        self._intialized = False

        # Logged data.
        self._metrics_file_name = None
        self._min_collision_times = []
        self._min_distances = []

        # Also log the raw data as list of lists (sublists are single trajs,
        # each element of which is a dictionary with keys
        # {"human_position", "robot_position", "time"}).
        self._current_traj_raw_data = []

        # Start and end time and current goal of current trajectory.
        self._start_time = None
        self._end_time = None
        self._current_goal = None
        self._reached_goal = False

        # Human position and velocity.
        self._human_position = None
        self._human_velocity = None
        self._human_time = None

        # Robot position and velocity.
        self._robot_position = None
        self._robot_velocity = None
        self._robot_time = None

    # Initialization and loading parameters.
    def Initialize(self):
        self._name = rospy.get_name() + "/data_logger"

        # Load parameters.
        if not self.LoadParameters():
            rospy.logerr("%s: Error loading parameters.", self._name)
            return False

        # Register callbacks.
        if not self.RegisterCallbacks():
            rospy.logerr("%s: Error registering callbacks.", self._name)
            return False

        self._initialized = True
        return True

    def LoadParameters(self):
        # Get the timer interval.
        if not rospy.has_param("~time_step"):
            return False
        self._time_step = rospy.get_param("~time_step")

        # Topics.
        if not rospy.has_param("~topics/human"):
            return False
        self._human_topic = rospy.get_param("~topics/human")

        if not rospy.has_param("~topics/robot"):
            return False
        self._robot_topic = rospy.get_param("~topics/robot")

        if not rospy.has_param("~topics/traj_request"):
            return False
        self._traj_request_topic = rospy.get_param("~topics/traj_request")

        # Decay factor for human velocity estimation.
        # Values closer to 1 forget the past faster than those near 0.
        if not rospy.has_param("~velocity_decay"):
            return False
        self._velocity_decay = rospy.get_param("~velocity_decay")

        # Maximum angle between human and robot to calculate time to collision.
        if not rospy.has_param("~max_angle"):
            return False
        self._max_angle = rospy.get_param("~max_angle")

        # File name where metrics data will be saved
        if not rospy.has_param("~metrics_file_name"):
            return False
        self._metrics_file_name = rospy.get_param("~metrics_file_name")

        return True

    def RegisterCallbacks(self):
        # Subscribers.
        self._human_sub = rospy.Subscriber(self._human_topic,
                                           geometry_msgs.msg.PoseStamped,
                                           self.HumanCallback)

        self._robot_sub = rospy.Subscriber(self._robot_topic,
                                           crazyflie_msgs.msg.PositionVelocityYawStateStamped,
                                           self.RobotCallback)

        self._traj_request_sub = rospy.Subscriber(self._traj_request_topic,
                                                  fastrack_msgs.msg.Trajectory,
                                                  self.TrajectoryRequestCallback)

        # Timer.
        self._timer = rospy.Timer(rospy.Duration(self._time_step), self.TimerCallback)

        return True

    # Update robot position.
    def RobotCallback(self, msg):
        if not self._initialized:
            print "In RobotCallback: not initialized."
            return

        position = np.array([msg.state.x, msg.state.y, msg.state.z])

        if self._robot_position is None:
            self._robot_position = position
            self._robot_velocity = np.array([0.0, 0.0, 0.0])
            self._robot_time = msg.header.stamp

            # Catch the first one.
            right_now = rospy.Time.now()
            if self._start_time is None:
                self._start_time = right_now
        else:
            dt = (msg.header.stamp - self._robot_time).to_sec()
            velocity = (position - self._robot_position) / dt

            self._robot_velocity = (velocity * self._velocity_decay + self._robot_velocity * (1.0 - self._velocity_decay))
            self._robot_position = position
            self._robot_time = msg.header.stamp

            # Check if we've reached the goal.
            if not self._reached_goal and self._current_goal is not None and np.linalg.norm(self._robot_position - self._current_goal) < 5e-2:
                self._reached_goal = True
                self._end_time = self._robot_time

    # Update human position and velocity.
    def HumanCallback(self, msg):
        if not self._initialized:
            print "In HumanCallback: not initialized."
            return

        position = np.array([msg.pose.position.x,
                             msg.pose.position.y,
                             msg.pose.position.z])

        if self._human_position is None:
            self._human_position = position
            self._human_velocity = np.array([0.0, 0.0, 0.0])
            self._human_time = msg.header.stamp
        else:
            dt = (msg.header.stamp - self._human_time).to_sec()
            velocity = (position - self._human_position) / dt

            # Update human velocity by a linear combination of the
            # new measured velocity and the old velocity. Weighting
            # is based on the velocity decay factor.
            self._human_velocity = (velocity * self._velocity_decay + self._human_velocity * (1.0 - self._velocity_decay))
            self._human_position = position
            self._human_time = msg.header.stamp

    # Trajectory request callback.
    def TrajectoryRequestCallback(self, msg):
        if not self._initialized:
            print "In TrajectoryRequestCallback: not initialized."
            return

        msg_goal = np.array([msg.states[-1].x[0], msg.states[-1].x[1], msg.states[-1].x[2]])
        self._current_goal = msg_goal

    # Timer callback.
    def TimerCallback(self, event):
        if not self._initialized:
            print "In TimerCallback: not initialized."
            return

        if self._human_position is None or self._robot_position is None or self._start_time is None:
            print "In TimerCallback: human or robot or start time is None."
            print "human: ", self._human_position
            print "robot: ", self._robot_position
            print "time: ", self._start_time
            return

        # Log collision time.
        self._min_collision_times.append(self.CollisionTime())

        # Log distance.
        print "Distance between human and robot: ", self.Distance()
        self._min_distances.append(self.Distance())

        # Log positions.
        self._current_traj_raw_data.append({"human_position" : self._human_position,
                                            "robot_position" : self._robot_position,
                                            "time" : (rospy.Time.now() - self._start_time).to_sec()})

    # Distance between human and robot.
    def Distance(self):
        if self._human_position is None or self._robot_position is None:
            return float("inf")
        return np.linalg.norm((self._human_position - self._robot_position)[:-1])

    # Compute human's time to collision with robot's plane.
    def CollisionTime(self):
        if self._human_position is None or self._robot_position is None:
            return float("inf")

        direction = self._human_position - self._robot_position
        velocity = self._human_velocity - self._robot_velocity

        angle = np.arccos(np.dot(direction[:-1], velocity[:-1]) /
                         (np.linalg.norm(velocity[:-1])* np.linalg.norm(direction[:-1])))

        if angle < self._max_angle:
            return np.linalg.norm(direction[:-1])**2 / np.dot(direction[:-1], velocity[:-1])
        else:
            return float("inf")

    # Save to disk.
    def Save(self):
        num_samples = len(self._min_distances)

        if (len(self._current_traj_raw_data) != len(self._min_collision_times) or 
            len(self._current_traj_raw_data) != len(self._min_distances)):
            rospy.logwarn("%s: Number of trajectories and collision times do not match.", self._name)

        # Columns will be as follows: min_collision_time | min_distance | traj_time.
        table = np.zeros((num_samples, 3))
        for ii in range(num_samples):
            table[ii, 0] = self._min_collision_times[ii]
            table[ii, 1] = self._min_distances[ii]
            table[ii, 2] = self._current_traj_raw_data[ii]["time"]
        print "table: ", table
        print "start, end time: ", self._start_time, self._end_time
        print "trajectory time: ", (self._end_time - self._start_time).to_sec()
        
        # Save.
        rospack = rospkg.RosPack()
        path = rospack.get_path('data_logger') + "/data/"
        np.savetxt(path + self._metrics_file_name + ".txt", table)
        pickle.dump((self._min_collision_times, self._min_distances, self._current_traj_raw_data), open(path + self._metrics_file_name + ".py", "wb"))
        rospy.loginfo("%s: Successfully saved data to disk.", self._name)
