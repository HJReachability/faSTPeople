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

#include <boost/bind.hpp>
#include <boost/function.hpp>

#include <crazyflie_human/OccupancyGridTime.h>

#include <fastrack_msgs/Trajectory.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <math.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <unordered_map>
#include <vector>

namespace fastrack {
namespace environment {

using bound::Box;
using trajectory::Trajectory;

template <typename S>
class STPeopleEnvironment
    : public Environment<crazyflie_human::OccupancyGridTime, Empty> {
 public:
  ~STPeopleEnvironment() {}
  explicit STPeopleEnvironment() : Environment() {}

  // Derived classes must provide a collision checker which returns true if
  // and only if the provided position is a valid collision-free configuration.
  // Provide a separate collision check for each type of tracking error bound.
  bool IsValid(const Vector3d& position, const Box& bound, double time) const;

  // Generate a sensor measurement.
  crazyflie_human::OccupancyGridTime SimulateSensor(const Empty& params) const {
    throw std::runtime_error("SimulateSensor is not implemented.");
    return crazyflie_human::OccupancyGridTime();
  }

  // Derived classes must have some sort of visualization through RViz.
  void Visualize() const;

 private:
  // Load parameters. This should still call Environment::LoadParameters.
  bool LoadParameters(const ros::NodeHandle& n);

  // Register callbacks. This should still call Environment::RegisterCallbacks.
  bool RegisterCallbacks(const ros::NodeHandle& n);

  // Implement pure virtual sensor callback from base class to handle new
  // occupancy grid time msgs. (This may be replaced by OccupancyGridCallback.)
  void SensorCallback(const crazyflie_human::OccupancyGridTime::ConstPtr& msg);

  // Generic callback to handle a new trajectory msg on the given topic.
  void TrajectoryCallback(const fastrack_msgs::Trajectory::ConstPtr& msg,
                          const std::string& topic);

  // Generic callback to handle a new occupancy grid msg on the given topic.
  void OccupancyGridCallback(
      const fastrack_msgs::OccupancyGridTime::ConstPtr& msg,
      const std::string& topic);

  // Topics on which agent trajectories will be published.
  std::vector<std::string> traj_topics_;

  // Topics on which agent occupancy grids will be published
  std::vector<std::string> occupancy_grid_topics_;

  // Map from robot priority to trajectory. Each robot only stores the
  // trajectories of robots with higher priority number than themselves
  std::unordered_map<size_t, Trajectory<S>> traj_registry_;

  // List of tracking error bounds (TEB) corresponding to trajectories.
  std::vector<Box> bound_registry_;

  // Map from topic to occupancy grid.
  std::unordered_map<std::string, OccupancyGridTimeInterpolator>
      occupancy_grid_registry_;

  // One subscriber for each trajectory/occupancy grid topic we're listening to.
  std::vector<ros::Subscriber> traj_subs_;
  std::vector<ros::Subscriber> occupancy_grid_subs_;

  // Radius of the sphere representing the quadrotor using this environment.
  double vehicle_size_;

  // Threshold for probability of collision.
  double collision_threshold_;
};  //\class STPeopleEnvironment

// -------------------------------- IMPLEMENTATION
// ------------------------------------- //

// Derived classes must provide a collision checker which returns true if
// and only if the provided position is a valid collision-free configuration.
// Provide a separate collision check for each type of tracking error bound.
template <typename S>
bool STPeopleEnvironment<S>::IsValid(const Vector3d& position, const Box& bound,
                                     double time) const {
  if (!initialized_) {
    ROS_WARN(
        "%s: Tried to collision check an uninitialized STPeopleEnvironment.",
        name_.c_str());
    return false;
  }

  // check against the boundary of the occupancy grid
  if (position(0) < lower_(0) + bound.x || position(0) > upper_(0) - bound.x ||
      position(1) < lower_(1) + bound.y || position(1) > upper_(1) - bound.y ||
      position(2) < lower_(2) + bound.z || position(2) > upper_(2) - bound.z)
    return false;

  // Find which points on other robots' trajectories to collision check with
  // and their corresponding tracking error bounds.
  std::vector<S> traj_points(traj_registry_.size());
  std::vector<Box> tebs(bound_registry_.size());
  size_t i = 0;
  // TODO: Are bound topics in the same order as trajectory topics?
  for (auto it = bound_registry_.begin(); it != bound_registry_.end(); ++it) {
    Box teb = it->second;
    tebs[i] = teb;
    ++i;
  }

  i = 0;
  for (auto it = traj_registry_.begin(); it != traj_registry_.end(); ++it) {
    S traj_pt = it->second.Interpolate(time);
    traj_points[i] = traj_pt;
    ++i;
  }

  // Collision checking with other robots.
  for (size_t ii = 0; ii < traj_points.size(); ii++) {
    const S& p = traj_points[ii];
    const Vector3d bound_vector(bound.x + tebs[ii].x, bound.y + tebs[ii].y,
                                bound.z + tebs[ii].z);

    // Find closest point in the tracking bound to the trajectory point.
    Vector3d closest_point;
    for (size_t jj = 0; jj < 3; jj++) {
      closest_point(jj) = std::min(
          position(jj) + bound_vector(jj),
          std::max(position(jj) - bound_vector(jj), p.Configuration()(jj)));
    }

    // Check distance to closest point.
    if ((closest_point - p.Configuration()).norm() <= vehicle_size_)
      return false;
  }

  // Interpolation of occupancy grids.
  // TODO!

  return true;
}

// Derived classes must have some sort of visualization through RViz.
template <typename S>
void STPeopleEnvironment<S>::Visualize() const {
  // TODO!
}

// Load parameters. This should still call Environment::LoadParameters.
template <typename S>
bool STPeopleEnvironment<S>::LoadParameters(const ros::NodeHandle& n) {
  ros::NodeHandle nl(n);
  name_ = ros::names::append(n.getNamespace(), "STPeopleEnvironment");

  // Service for loading bound.
  std::vector<std::string> other_bound_srvs_name;

  if (!nl.getParam("topic/other_trajs", traj_topics_)) return false;
  if (!nl.getParam("topic/other_occupancy_grids", occupancy_grid_topics_))
    return false;
  if (!nl.getParam("vehicle_size", vehicle_size_)) return false;
  if (!nl.getParam("collision_threshold", collision_threshold_)) return false;
  if (!nl.getParam("other_bound_srvs", other_bound_srvs_name)) return false;

  // Load all the TEBs for the higher-priority robots.
  for (size_t ii = 0; ii < other_bound_srvs_name.size(); ii++) {
    std::string srv_name = other_bound_srvs_name_[ii];
    ros::service::waitForService(srv_name);
    ros::ServiceClient bound_srv = nl.serviceClient<SB>(srv_name.c_str(), true);

    SB b;
    if (!bound_srv.call(b)) {
      ROS_ERROR("%s: Bound server error.", name_.c_str());
      return false;
    }

    bound_registry_[ii].FromRos(b.response);
  }

  return Environment::LoadParameters(n);
}

// Register callbacks. This should still call Environment::RegisterCallbacks.
template <typename S>
bool STPeopleEnvironment<S>::RegisterCallbacks(const ros::NodeHandle& n) {
  if (!Environment<crazyflie_human::OccupancyGridTime,
                   Empty>::RegisterCallbacks(n)) {
    ROS_ERROR("%s: Environment register callbacks failed.", name_.c_str());
    return false;
  }

  ros::NodeHandle nl(n);

  // Set up all the trajectory subscribers.
  for (const auto& topic : traj_topics_) {
    // Generate a lambda function for this callback.
    boost::function<void(const fastrack_msgs::Trajectory::ConstPtr&,
                         const std::string&)>
        callback = [this](const fastrack_msgs::Trajectory::ConstPtr& msg,
                          const std::string& topic) {
          TrajectoryCallback(msg, topic);
        };  // callback

    // Create a new subscriber with this callback.
    traj_subs_.emplace_back(nl.subscribe<fastrack_msgs::Trajectory>(
        topic.c_str(), 1, boost::bind(callback, _1, topic)));
  }

  // Set up all the occupancy grid subscribers.
  for (const auto& topic : occupancy_grid_topics_) {
    // Generate a lambda function for this callback.
    boost::function<void(const fastrack_msgs::OccupancyGridTime::ConstPtr&,
                         const std::string&)>
        callback = [this](const fastrack_msgs::OccupancyGridTime::ConstPtr& msg,
                          const std::string& topic) {
          OccupancyGridCallback(msg, topic);
        };  // callback

    // Create a new subscriber with this callback.
    traj_subs_.emplace_back(nl.subscribe<fastrack_msgs::OccupancyGridTime>(
        topic.c_str(), 1, boost::bind(callback, _1, topic)));
  }
}

// Implement pure virtual sensor callback from base class to handle new
// occupancy grid time msgs.
template <typename S>
void STPeopleEnvironment<S>::SensorCallback(
    const crazyflie_human::OccupancyGridTime::ConstPtr& msg) {
  occupancy_grids_ = msg;
}

// Generic callback to handle a new trajectory msg coming from robot on
// the given topic.
template <typename S>
void STPeopleEnvironment<S>::TrajectoryCallback(
    const fastrack_msgs::Trajectory::ConstPtr& msg, const std::string& topic) {
  // Check if there's already a trajectory stored for this topic.
  auto iter = traj_registry_.find(topic);
  if (iter == traj_registry_.end()) {
    // This is a topic we haven't seen before.
    traj_registry_.emplace(topic, Trajectory<S>(msg));
  } else {
    // We've already seen this topic, so just update the recorded trajectory.
    iter->second = Trajectory<S>(msg);
  }
}

// Generic callback to handle a new occupancy grid msg on the given topic.
void STPeopleEnvironment::OccupancyGridCallback(
    const fastrack_msgs::OccupancyGridTime::ConstPtr& msg,
    const std::string& topic) {
  // Check if there's already an occupancy grid stored for this topic.
  auto iter = occupancy_grid_registry_.find(topic.c_str());
  if (iter == occupancy_grid_registry_.end()) {
    // Construct occupancy grid in place if doesn't exist yet.
    // NOTE: emplace() is c++ 17 standard, may need to set flag "-std=c++17".
    occupancy_grid_registry_.emplace(
        topic.c_str(),
        OccupancyGridTimeInterpolator(msg, this->lower_(0), this->upper_(0),
                                      this->lower_(1), this->upper_(1)));
  } else {
    // Update existing OccupancyGridTime data structure from incoming message.
    search->second =
        OccupancyGridTimeInterpolator(msg, this->lower_(0), this->upper_(0),
                                      this->lower_(1), this->upper_(1));
  }
}

}  //\namespace environment
}  //\namespace fastrack

#endif
