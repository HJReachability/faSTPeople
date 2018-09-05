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

#include <crazyflie_human/OccupancyGridTime.h>
#include <fastpeople/environment/occupancy_grid_time_interpolator.h>
#include <fastrack/environment/environment.h>
#include <fastrack/trajectory/trajectory.h>
#include <fastrack/utils/types.h>
#include <fastrack_msgs/Trajectory.h>
#include <fastrack_srvs/TrackingBoundBox.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <math.h>
#include <ros/assert.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <boost/bind.hpp>
#include <boost/function.hpp>
#include <mutex>
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
  explicit STPeopleEnvironment()
      : Environment<crazyflie_human::OccupancyGridTime, Empty>() {}

  // Derived classes must provide a collision checker which returns true if
  // and only if the provided position is a valid collision-free configuration.
  // Provide a separate collision check for each type of tracking error bound.
  bool IsValid(const Vector3d& position, const Box& bound, double time) const;

  // Returns just the noisy-OR'd collision probability with the humans.
  double HumanCollisionProbability(const Vector3d& position, 
    const Box& bound, double time) const;

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

  // Base class sensor callback. Not implemented in favor of custom callbacks
  // for each occupancy grid callback and trajectory callback.
  void SensorCallback(const crazyflie_human::OccupancyGridTime::ConstPtr& msg);

  // Generic callback to handle a new trajectory msg on the given topic.
  void TrajectoryCallback(const fastrack_msgs::Trajectory::ConstPtr& msg,
                          const std::string& topic);

  // Generic callback to handle a new occupancy grid msg on the given topic.
  void OccupancyGridCallback(
      const crazyflie_human::OccupancyGridTime::ConstPtr& msg,
      const std::string& topic);

  // Topics on which agent trajectories will be published.
  std::vector<std::string> traj_topics_;

  // Topics on which agent occupancy grids will be published
  std::vector<std::string> occupancy_grid_topics_;

  // Map from topic to trajectory, and topic to TEB. Each robot only stores the
  // trajectories of robots with higher priority number than themselves
  std::unordered_map<std::string, Trajectory<S>> traj_registry_;
  std::unordered_map<std::string, Box> bound_registry_;

  // Map from topic to occupancy grid, with mutex.
  std::unordered_map<std::string, OccupancyGridTimeInterpolator>
      occupancy_grid_registry_;
  mutable std::mutex occupancy_grid_mutex_;

  // One subscriber for each trajectory/occupancy grid topic we're listening to.
  std::vector<ros::Subscriber> traj_subs_;
  std::vector<ros::Subscriber> occupancy_grid_subs_;

  // Threshold for probability of collision.
  double collision_threshold_;
};  //\class STPeopleEnvironment

// ----------------------------- IMPLEMENTATION ----------------------------- //

// Derived classes must provide a collision checker which returns true if
// and only if the provided position is a valid collision-free configuration.
// Provide a separate collision check for each type of tracking error bound.
template <typename S>
bool STPeopleEnvironment<S>::IsValid(const Vector3d& position, const Box& bound,
                                     double time) const {
  // Lock mutex. When this goes out of scope it will unlock.
  std::lock_guard<std::mutex> lock(occupancy_grid_mutex_);

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

  // Collision checking with other robots.
  for (const auto& pair : traj_registry_) {
    const std::string& topic = pair.first;

    // Get another robot's trajectory and TEB.
    const Trajectory<S>& traj = pair.second;
    const Box& teb = bound_registry_.at(topic);

    // Interpolate the trajectory in time to find candidate point.
    const S& traj_pt = traj.Interpolate(time);
    const Vector3d other_position = traj_pt.Position();

    // Check if the TEBs do not intersect.
    const bool in_collision =
        (std::abs(position(0) - other_position(0)) < (bound.x + teb.x)) &&
        (std::abs(position(1) - other_position(1)) < (bound.y + teb.y)) &&
        (std::abs(position(2) - other_position(2)) < (bound.z + teb.z));

    if (in_collision) return false;
  }

  // Compute the total collision probability with each human and noisyOR
  // the probabilities togther to check if beneath collision threshold.
  double noisyOR_complement = 1.0;
  for (const auto& pair : occupancy_grid_registry_) {
    const OccupancyGridTimeInterpolator& interpolator = pair.second;

    // Interpolate the human's occupancy grid in time and then integrate the
    // probability mass inside the TEB.
    const double integrated_prob =
        interpolator.OccupancyProbability(position, bound, time);
    constexpr double kSmallNumber = 1e-8;
    if (integrated_prob > 1.0 + kSmallNumber ||
        integrated_prob < -kSmallNumber) {
      throw std::runtime_error("Invalid probability encountered: " +
                               std::to_string(integrated_prob));
    }

    noisyOR_complement *= 1.0 - integrated_prob;
    if (1.0 - noisyOR_complement > collision_threshold_) return false;
  }

  return true;
}

// Returns just the noisy-OR probability of collision. 
template <typename S>
double STPeopleEnvironment<S>::HumanCollisionProbability(const Vector3d& position, 
  const Box& bound, double time) const {

  // Compute the total collision probability with each human and noisyOR
  // the probabilities togther to check if beneath collision threshold.
  double noisyOR_complement = 1.0;
  for (const auto& pair : occupancy_grid_registry_) {
    const OccupancyGridTimeInterpolator& interpolator = pair.second;

    // Interpolate the human's occupancy grid in time and then integrate the
    // probability mass inside the TEB.
    const double integrated_prob =
        interpolator.OccupancyProbability(position, bound, time);
    constexpr double kSmallNumber = 1e-8;
    if (integrated_prob > 1.0 + kSmallNumber ||
        integrated_prob < -kSmallNumber) {
      throw std::runtime_error("Invalid probability encountered: " +
                               std::to_string(integrated_prob));
    }

    noisyOR_complement *= 1.0 - integrated_prob;
  }

  return 1.0 - noisyOR_complement;
}

// Load parameters. This should still call Environment::LoadParameters.
template <typename S>
bool STPeopleEnvironment<S>::LoadParameters(const ros::NodeHandle& n) {
  if (!Environment<crazyflie_human::OccupancyGridTime, Empty>::LoadParameters(
          n)) {
    ROS_WARN("%s: Base class Environment could not load parameters.",
             name_.c_str());
    return false;
  }

  ros::NodeHandle nl(n);

  // Load other traj/occupancy grid topics and collision probability threshold.
  if (!nl.getParam("topic/other_trajs", traj_topics_)) return false;
  if (!nl.getParam("topic/other_occupancy_grids", occupancy_grid_topics_))
    return false;
  if (!nl.getParam("collision_threshold", collision_threshold_)) return false;

  // Services for loading other tracking error bounds.
  std::vector<std::string> other_bound_srvs_name;
  if (!nl.getParam("srv/other_bounds", other_bound_srvs_name)) return false;

  // Load all the TEBs for the higher-priority robots.
  for (size_t ii = 0; ii < other_bound_srvs_name.size(); ii++) {
    const std::string& srv_name = other_bound_srvs_name[ii];
    ros::service::waitForService(srv_name);
    ros::ServiceClient bound_srv =
        nl.serviceClient<fastrack_srvs::TrackingBoundBox>(srv_name.c_str(),
                                                          true);

    fastrack_srvs::TrackingBoundBox b;
    if (!bound_srv.call(b)) {
      ROS_ERROR("%s: Bound server error.", name_.c_str());
      return false;
    }

    // Bound registry has traj_topic keys for easy TEB lookup
    // for a given robot's trajectory. Useful when collision checking.
    Box bound;
    bound.FromRos(b.response);
    bound_registry_.emplace(traj_topics_[ii], bound);
  }

  return true;
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
        callback = [=](const fastrack_msgs::Trajectory::ConstPtr& msg,
                       const std::string& topic) {
          TrajectoryCallback(msg, topic);
        };  // callback

    // Create a new subscriber with this callback.
    traj_subs_.emplace_back(nl.subscribe<fastrack_msgs::Trajectory>(
        topic, 1, boost::bind(callback, _1, topic)));
  }

  // Set up all the occupancy grid subscribers.
  for (const auto& topic : occupancy_grid_topics_) {
    // Generate a lambda function for this callback.
    boost::function<void(const crazyflie_human::OccupancyGridTime::ConstPtr&,
                         const std::string&)>
        callback = [=](const crazyflie_human::OccupancyGridTime::ConstPtr& msg,
                       const std::string& topic) {
          OccupancyGridCallback(msg, topic);
        };  // callback

    // Create a new subscriber with this callback.
    traj_subs_.emplace_back(nl.subscribe<crazyflie_human::OccupancyGridTime>(
        topic, 1, boost::bind(callback, _1, topic)));
  }

  return true;
}

// Base class sensor callback. Not implemented in favor of custom callbacks
// for each occupancy grid callback and trajectory callback.
template <typename S>
void STPeopleEnvironment<S>::SensorCallback(
    const crazyflie_human::OccupancyGridTime::ConstPtr& msg) {
  throw std::runtime_error("Nothing should ever be received on this topic.");
}

// Generic callback to handle a new trajectory msg coming from robot on
// the given topic.
template <typename S>
void STPeopleEnvironment<S>::TrajectoryCallback(
    const fastrack_msgs::Trajectory::ConstPtr& msg, const std::string& topic) {
  // Lock mutex. When this goes out of scope it will unlock.
  std::lock_guard<std::mutex> lock(occupancy_grid_mutex_);

  // Check if there's already a trajectory stored for this topic.
  auto iter = traj_registry_.find(topic);
  if (iter == traj_registry_.end()) {
    // This is a topic we haven't seen before.
    traj_registry_.emplace(topic, Trajectory<S>(msg));
  } else {
    // We've already seen this topic, so just update the recorded trajectory.
    iter->second = Trajectory<S>(msg);
  }

  // Let the system know this environment has been updated.
  this->updated_pub_.publish(std_msgs::Empty());
}

// Generic callback to handle a new occupancy grid msg on the given topic.
template <typename S>
void STPeopleEnvironment<S>::OccupancyGridCallback(
    const crazyflie_human::OccupancyGridTime::ConstPtr& msg,
    const std::string& topic) {
  // Lock mutex.
  std::lock_guard<std::mutex> lock(occupancy_grid_mutex_);

  // Check if there's already an occupancy grid stored for this topic.
  auto iter = occupancy_grid_registry_.find(topic);
  if (iter != occupancy_grid_registry_.end()) {
    // This topic already exists. Delete it and reinsert.
    occupancy_grid_registry_.erase(iter);
  }

  // Construct occupancy grid in place.
  // NOTE: emplace() is c++ 17 standard, may need to set flag "-std=c++17".
  occupancy_grid_registry_.emplace(
      topic,
      OccupancyGridTimeInterpolator(msg, this->lower_(0), this->upper_(0),
                                    this->lower_(1), this->upper_(1)));

  // Let the system know this environment has been updated.
  this->updated_pub_.publish(std_msgs::Empty());
}

// Derived classes must have some sort of visualization through RViz.
template <typename S>
void STPeopleEnvironment<S>::Visualize() const {
  // Set up box marker.
  visualization_msgs::Marker cube;
  cube.ns = "cube";
  cube.header.frame_id = this->fixed_frame_;
  cube.header.stamp = ros::Time::now();
  cube.id = 0;
  cube.type = visualization_msgs::Marker::CUBE;
  cube.action = visualization_msgs::Marker::ADD;
  cube.color.a = 0.5;
  cube.color.r = 0.3;
  cube.color.g = 0.7;
  cube.color.b = 0.7;

  geometry_msgs::Point center;

  // Fill in center and scale.
  cube.scale.x = upper_(0) - lower_(0);
  center.x = lower_(0) + 0.5 * cube.scale.x;

  cube.scale.y = upper_(1) - lower_(1);
  center.y = lower_(1) + 0.5 * cube.scale.y;

  cube.scale.z = upper_(2) - lower_(2);
  center.z = lower_(2) + 0.5 * cube.scale.z;

  cube.pose.position = center;
  cube.pose.orientation.x = 0.0;
  cube.pose.orientation.y = 0.0;
  cube.pose.orientation.z = 0.0;
  cube.pose.orientation.w = 1.0;

  // Publish cube marker.
  this->vis_pub_.publish(cube);
}

}  // namespace environment
}  // namespace fastrack

#endif
