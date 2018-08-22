/*
 * Copyright (c) 2018, The Regents of the University of California (Regents).
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 *    1. Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *
 *    2. Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials provided
 *       with the distribution.
 *
 *    3. Neither the name of the copyright holder nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS AS IS
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * Please contact the author(s) of this library if you have any questions.
 * Authors: David Fridovich-Keil   ( dfk@eecs.berkeley.edu )
 */

///////////////////////////////////////////////////////////////////////////////
//
// STPeopleEnvironment is derived from the Environment base class.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef FASTPEOPLE_ENVIRONMENT_STPEOPLE_ENVIRONMENT_H
#define FASTPEOPLE_ENVIRONMENT_STPEOPLE_ENVIRONMENT_H

#include <fastrack/environment/environment.h>
#include <fastrack/trajectory/trajectory.h>
#include <fastrack/utils/types.h>

#include <crazyflie_human/OccupancyGridTime.h>

#include <fastrack_msgs/Trajectory.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <vector>
#include <unordered_map>

#include <std_msgs/Float64.h>

namespace fastrack {
namespace environment {

using bound::Box;
using trajectory::Trajectory;

class STPeopleEnvironment
    : public Environment<crazyflie_human::OccupancyGridTime, Empty> {
public:
  ~STPeopleEnvironment() {}
  explicit STPeopleEnvironment() : Environment() {}

  // Derived classes must provide a collision checker which returns true if
  // and only if the provided position is a valid collision-free configuration.
  // Provide a separate collision check for each type of tracking error bound.
  bool IsValid(const Vector3d &position, const Box &bound,
      double time = std::numeric_limits<double>::quiet_NaN()) const;

  // Generate a sensor measurement.
  crazyflie_human::OccupancyGridTime SimulateSensor(const Empty &params) const {
    throw std::runtime_error("SimulateSensor is not implemented.");
    return crazyflie_human::OccupancyGridTime();
  }

  // Derived classes must have some sort of visualization through RViz.
  void Visualize() const;

private:
  // Load parameters. This should still call Environment::LoadParameters.
  bool LoadParameters(const ros::NodeHandle &n);

  // Register callbacks. This should still call Environment::RegisterCallbacks.
  bool RegisterCallbacks(const ros::NodeHandle &n);

  // Implement pure virtual sensor callback from base class to handle new
  // occupancy grid time msgs.
  void SensorCallback(const crazyflie_human::OccupancyGridTime::ConstPtr &msg);

  // Generic callback to handle a new trajectory msg coming from robot on
  // the given topic.
  void TrajectoryCallback(const fastrack_msgs::Trajectory::ConstPtr &msg,
                          const std::string &topic);

  // SensorCallback helper function that turns an OccupancyGridTime message into 
  // a map from time stamps to occupancy grids (arrays)
  std::unordered_map<double, float64[]> MsgToOccuGrid(
    const crazyflie_human::OccupancyGridTime::ConstPtr &msg);

  // Topics on which other robots' trajectories will be published.
  std::vector<std::string> topics_;

  // Map from topic to trajectory, and from topic to TEB.
  std::unordered_map<std::string, Trajectory<Vector3d>> traj_registry_;
  std::unordered_map<std::string, Box> bound_registry_;

  // One subscriber for each trajectory topic we're listening to.
  std::vector<ros::Subscriber> traj_subs_;

  // Map of occupancy grids received from sensor, time stamp to 2D array 
  std::unordered_map<double, std_msgs::Float64[]> occupancy_grids;
}; //\class STPeopleEnvironment

} //\namespace environment
} //\namespace fastrack

#endif
