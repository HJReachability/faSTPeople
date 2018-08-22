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

#include <fastpeople/environment/stpeople_environment.h>

#include <boost/bind.hpp>
#include <boost/function.hpp>

namespace fastrack {
namespace environment {

// Derived classes must provide a collision checker which returns true if
// and only if the provided position is a valid collision-free configuration.
// Provide a separate collision check for each type of tracking error bound.
bool STPeopleEnvironment::IsValid(const Vector3d &position,
                                  const Box &bound,
                                  double time) const {
    if (!initialized_) {
    ROS_WARN("%s: Tried to collision check an uninitialized BallsInBox.",
             name_.c_str());
    return false;
    }

  // check against the boundary of the occupancy grid
  if (position(0) < lower_(0) + bound.x ||
      position(0) > upper_(0) - bound.x ||
      position(1) < lower_(1) + bound.y ||
      position(1) > upper_(1) - bound.y ||
      position(2) < lower_(2) + bound.z ||
      position(2) > upper_(2) - bound.z)
    return false;

  std::vector<Vector3d> centers(traj_registry_.size());
  std::vector<Box> tebs(bound_registry_.size());
  int i = 0;
  for (auto it = traj_registry_.begin(); it != traj_registry_.end(); ++it) {
    Box teb = bound_registry_[it->first];
    Vector3d traj_pt = it->second.Interpolate(time);
    centers[i] = traj_pt;
    tebs[i] = teb;
    ++i;
  }

  // Check against each obstacle.
  // NOTE! Just using a linear search here for simplicity.
  //if (centers.size() > 100)
  //  ROS_WARN_THROTTLE(1.0, "%s: Caution! Linear search may be slowing you down.",
  //                    name_.c_str());

  //const Vector3d bound_vector(bound.x, bound.y, bound.z);
  for (size_t ii = 0; ii < centers.size(); ii++) {
    const Vector3d& p = centers[ii];
    const Vector3d bound_vector(bound.x + tebs[ii].x, bound.y + tebs[ii].y, bound.z + tebs[ii].z);

    // Find closest point in the tracking bound to the obstacle center.
    Vector3d closest_point;
    for (size_t jj = 0; jj < 3; jj++) {
      closest_point(jj) =
        std::min(position(jj) + bound_vector(jj),
                 std::max(position(jj) - bound_vector(jj), p(jj)));
    }

    // Check distance to closest point.
    // TODO: Vehicle size!
    if ((closest_point - p).norm() <= 0.0001)
      return false;
  }

  return true;
}

// Derived classes must have some sort of visualization through RViz.
void STPeopleEnvironment::Visualize() const {
  // TODO!
}

// Load parameters. This should still call Environment::LoadParameters.
bool STPeopleEnvironment::LoadParameters(const ros::NodeHandle &n) {
  ros::NodeHandle nl(n);
  if (!nl.getParam("topic/other_trajs", topics_)) return false;
  return Environment::LoadParameters(&n);
}

// Register callbacks. This should still call Environment::RegisterCallbacks.
bool STPeopleEnvironment::RegisterCallbacks(const ros::NodeHandle &n) {
  if (!Environment<crazyflie_human::OccupancyGridTime,
                   Empty>::RegisterCallbacks(n)) {
    ROS_ERROR("%s: Environment register callbacks failed.", name_.c_str());
    return false;
  }

  ros::NodeHandle nl(n);

  // Set up all the subscribers.
  for (const auto &topic : topics_) {
    // Generate a lambda function for this callback.
    boost::function<void(const fastrack_msgs::Trajectory::ConstPtr &,
                         const std::string &)>
        callback = [this](const fastrack_msgs::Trajectory::ConstPtr &msg,
                          const std::string &topic) {
          TrajectoryCallback(msg, topic);
        }; // callback

    // Create a new subscriber with this callback.
    traj_subs_.emplace_back(nl.subscribe<fastrack_msgs::Trajectory>(
        topic.c_str(), 1, boost::bind(callback, _1, topic)));
  }
}

// Implement pure virtual sensor callback from base class to handle new
// occupancy grid time msgs.
void STPeopleEnvironment::SensorCallback(
    const crazyflie_human::OccupancyGridTime::ConstPtr &msg) {

  occupancy_grids = MsgToOccuGrid(msg);
}

// SensorCallback helper function that turns an OccupancyGridTime message into 
// a map from time stamps to occupancy grids (arrays)
std::unordered_map<double, std_msgs::Float64[]> MsgToOccuGrid(
    const crazyflie_human::OccupancyGridTime::ConstPtr &msg){

  std::unordered_map<double, std_msgs::Float64[]> og;
  double tcount = 0;
  for(std::size_t ii = 0; ii < (sizeof(msg->gridarray)/sizeof(msg->gridarray[0])); ++ii) {
    if(ii > 0) {
      tcount += (msg->gridarray[ii].header.stamp - msg->gridarray[ii - 1].header.stamp).toSec();
    }
    og.insert({tcount, msg->gridarray[ii].data});
  }

  return og;

}

// Generic callback to handle a new trajectory msg coming from robot on
// the given topic.
void STPeopleEnvironment::TrajectoryCallback(
    const fastrack_msgs::Trajectory::ConstPtr &msg, const std::string &topic) {
/*
  if(traj_registry_.contains(topic)) {
    traj_registry_.erase(topic);
  }
  traj_registry_.insert({topic, Trajectory(msg)});
*/
  auto iter = traj_registry_.find(topic);
  if (iter == traj_registry_.end()) {
   // This is a topic we haven't seen before.
    traj_registry_.emplace(topic, Trajectory<Vector3d>(msg));
  } else {
   // We've already seen this topic, so just update the recorded trajectory.
    iter->second = Trajectory<Vector3d>(msg);
}


}

} //\namespace environment
} //\namespace fastrack