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

namespace fastrack {
namespace environment {

// Derived classes must provide a collision checker which returns true if
// and only if the provided position is a valid collision-free configuration.
// Provide a separate collision check for each type of tracking error bound.
bool STPeopleEnvironment::IsValid(const Vector3d &position,
                                  const Box &bound) const {
  // TODO!
  return false;
}

// Derived classes must have some sort of visualization through RViz.
void STPeopleEnvironment::Visualize() const {
  // TODO!
}

// Load parameters. This should still call Environment::LoadParameters.
bool STPeopleEnvironment::LoadParameters(const ros::NodeHandle &n) {
  // TODO!
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
    traj_subs_.emplace_back(
        nl.subscribe(topic.c_str(), 1,
                     [&topic, this](const fastrack_msgs::Trajectory::ConstPtr &msg) {
                       this->TrajectoryCallback(topic, msg);
                     }));
  }
}

// Implement pure virtual sensor callback from base class to handle new
// occupancy grid time msgs.
void STPeopleEnvironment::SensorCallback(
    const fastpeople_msgs::OccupancyGridTime::ConstPtr &msg) {
  // TODO!
}

// Generic callback to handle a new trajectory msg coming from robot on
// the given topic.
void STPeopleEnvironment::TrajectoryCallback(
    const std::string &topic, const fastrack_msgs::Trajectory::ConstPtr &msg) {
  // TODO!
}

} //\namespace environment
} //\namespace fastrack
