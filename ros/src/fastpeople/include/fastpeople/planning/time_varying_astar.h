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
// Time varying A* planner, which inherits from FaSTrack's KinematicPlanner.
// Templated on state type (S), environment type (E), tracking error bound type
// (B), and a tracking error bound service type (SB).
//
///////////////////////////////////////////////////////////////////////////////

#ifndef FASTPEOPLE_PLANNING_TIME_VARYING_ASTAR_H
#define FASTPEOPLE_PLANNING_TIME_VARYING_ASTAR_H

#include <fastpeople/environment/stpeople_environment.h>
#include <fastrack/planning/kinematic_planner.h>

#include <boost/functional/hash.hpp>
#include <list>
#include <set>
#include <unordered_set>

namespace fastrack {
namespace planning {

using dynamics::Kinematics;

template <typename S, typename E, typename B, typename SB>
class TimeVaryingAStar : public KinematicPlanner<S, E, B, SB> {
 public:
  ~TimeVaryingAStar() {}
  explicit TimeVaryingAStar() : KinematicPlanner<S, E, B, SB>() {}

  // Plan a trajectory from the given start to goal states starting
  // at the given time.
  // NOTE! The states in the output trajectory are essentially configurations.
  Trajectory<S> Plan(const S& start, const S& goal,
                     double start_time = 0.0) const;

 private:
  // Load parameters. Be sure to call the base KinematicPlanner::LoadParameters.
  bool LoadParameters(const ros::NodeHandle& n);

  // Custom node struct.
  struct Node {
   private:
   public:
    typedef std::shared_ptr<const Node> ConstPtr;
    typedef std::shared_ptr<Node> Ptr;

    // Member variables.
    const S point_;
    const ConstPtr parent_;
    const double time_;
    const double cost_to_come_;
    const double heuristic_;
    const double priority_;

    // Factory method.
    static inline Ptr Create(const S& point, const ConstPtr& parent,
                             double time, double cost_to_come,
                             double heuristic) {
      Ptr ptr(new Node(point, parent, time, cost_to_come, heuristic));
      return ptr;
    }

    // Print function.
    inline void PrintNode(const double start_time = 0.0) const {
      std::cout << "---- Node Info: ----\n";
      std::cout << "  point: [" << point_.Configuration().transpose() << "]"
                << std::endl;
      std::cout << "  time: " << (time_ - start_time) << std::endl;
      std::cout << "  cost_to_come: " << cost_to_come_ << std::endl;
      std::cout << "  heuristic: " << heuristic_ << std::endl;
      std::cout << "  priority: " << priority_ << std::endl;
      if (parent_ != nullptr)
        std::cout << "  parent id: " << parent_->id_ << std::endl;
    }

    // Comparitor. Returns true if heuristic cost of Node 1 < for Node 2.
    struct NodeComparitor {
      inline bool operator()(const Node::Ptr& node1,
                             const Node::Ptr& node2) const {
        return node1->priority_ < node2->priority_;
      }
    };  // class NodeComparitor

    // Equality. Returns true if Node 1 space-time is same as Node 2 space-time.
    struct NodeEqual {
      inline bool operator()(const Node::Ptr& node1,
                             const Node::Ptr& node2) const {
        return (std::abs(node1->time_ - node2->time_) < 1e-8 &&
                node1->point_.Configuration().isApprox(
                    node2->point_.Configuration(), 1e-8));
      }
    };  // class NodeEqual

    // Custom hash functor.
    struct NodeHasher {
      inline bool operator()(const Node::Ptr& node) const {
        size_t seed = 0;

        // Hash this node's contents together.
        boost::hash_combine(seed, boost::hash_value(node->time_));
        for (size_t ii = 0; ii < node->point_.ConfigurationDimension(); ii++) {
          boost::hash_combine(
              seed, boost::hash_value(node->point_.Configuration()(ii)));
        }

        return seed;
      }
    };

   private:
    explicit Node(const S& point, const ConstPtr& parent, double time,
                  double cost_to_come, double heuristic)
        : point_(point),
          parent_(parent),
          time_(time),
          cost_to_come_(cost_to_come),
          heuristic_(heuristic),
          priority_(heuristic + cost_to_come) {}

  };  //\struct Node

  // Collision check a line segment between the two points with the given
  // start and stop times. Returns true if the path is collision free and
  // false otherwise.
  bool CollisionCheck(const S& start, const S& stop, double start_time,
                      double stop_time) const;

  // This function removes all instances of next with matching
  // point and time values from the given multiset (there should only be 1).
  static void RemoveFromMultiset(
      const typename Node::Ptr next,
      std::multiset<typename Node::Ptr, typename Node::NodeComparitor>& open);

  // Walk backward from the given node to the root to create a Trajectory.
  Trajectory<S> GenerateTrajectory(const typename Node::ConstPtr& node) const;

  // Helper function to find all neighboring nodes from the specified dimension
  // to the final dimension.
  std::list<VectorXd> FindNeighborsFromDimension(const VectorXd& config,
                                                 size_t dimension) const;

  // Get the neighbors of the given point on the implicit grid.
  // NOTE! Include the given point.
  std::list<VectorXd> Neighbors(const S& point) const;

  // Returns the total cost to get to point. best_time is the fastest time
  // it takes to go from the parent to the point.
  double ComputeCostToCome(
      const typename Node::ConstPtr& parent, const S& point,
      double best_time = -std::numeric_limits<double>::infinity()) const;

  // Returns the heuristic for the point.
  double ComputeHeuristic(const S& point, const S& stop) const;

  // Side length of virtual grid cells.
  double grid_resolution_;

  // Maximum distance between test points on line segments being
  // collision checked.
  double collision_check_resolution_;
};  //\class TimeVaryingAStar

// --------------------------- IMPLEMENTATION ------------------------------- //

// Plan a trajectory from the given start to goal states starting
// at the given time.
// NOTE! The states in the output trajectory are essentially configurations.
template <typename S, typename E, typename B, typename SB>
Trajectory<S> TimeVaryingAStar<S, E, B, SB>::Plan(const S& start, const S& end,
                                                  double start_time) const {
  const ros::Time plan_start_time = ros::Time::now();
  constexpr double kStayPutTime = 1.0;

  // Make a set for the open set. This stores the candidate "fringe" nodes
  // we might want to expand. This is sorted by priority and allows multiple
  // nodes with the same priority to be in the list.
  typename std::multiset<typename Node::Ptr, typename Node::NodeComparitor>
      open;

  // Make a hash set for the open set. The key in this hash table
  // is space-time for a node. This is used to find nodes with the same
  // space-time key in log time.
  typename std::unordered_set<typename Node::Ptr, typename Node::NodeHasher,
                              typename Node::NodeEqual>
      open_registry;

  // Make a hash set for the closed set. The key in this hash table
  // is space-time for a node. This is used to find nodes with the same
  // space-time key in constant time.
  typename std::unordered_set<typename Node::Ptr, typename Node::NodeHasher,
                              typename Node::NodeEqual>
      closed_registry;

  // Initialize the priority queue.
  constexpr double kStartCost = 0.0;
  const double start_heuristic = ComputeHeuristic(start, end);
  const typename Node::Ptr start_node =
      Node::Create(start, nullptr, start_time, kStartCost, start_heuristic);

  open.insert(start_node);
  open_registry.insert(start_node);

  // Main loop - repeatedly expand the top priority node and
  // insert neighbors that are not already in the closed list.
  while (true) {
    // Check if we have run out of planning time.
    if ((ros::Time::now() - plan_start_time).toSec() > this->max_runtime_) {
      ROS_ERROR("%s: Ran out of time.", this->name_.c_str());
      return Trajectory<S>();
    }

    // Checking open list size.
    if (open.size() != open_registry.size()) {
      throw std::runtime_error("Open and open_registry are not the same size!");
    }

    if (open.empty()) {
      ROS_ERROR_THROTTLE(1.0, "%s: Open list is empty.", this->name_.c_str());
      return Trajectory<S>();
    }

    const typename Node::Ptr next = *open.begin();

    // Pop the next node from the open_registry and open set.
    open_registry.erase(
        next);  // works because keys in open_registry are unique!
    RemoveFromMultiset(next, open);

    // Check if this guy is the goal.
    const double dist = (next->point_ - end).Configuration().norm();

    constexpr double kHalfSquareRootThree = 0.5 * std::sqrt(3.0);
    if (dist <= grid_resolution_ * kHalfSquareRootThree) {
      const typename Node::ConstPtr parent_node =
          (next->parent_ == nullptr) ? next : next->parent_;

      // Have to connect the goal point to the last sampled grid point.
      const double best_time =
          this->dynamics_.BestPossibleTime(parent_node->point_, end);
      const double terminus_time = parent_node->time_ + best_time;
      const double terminus_cost =
          ComputeCostToCome(parent_node, end, best_time);
      const double terminus_heuristic = 0.0;

      const typename Node::Ptr terminus = Node::Create(
          end, parent_node, terminus_time, terminus_cost, terminus_heuristic);

      // Only succeed if final line segment is collision free.
      if (!CollisionCheck(parent_node->point_, end, parent_node->time_, terminus_time)) {
        ROS_WARN("%s: Collision check failure adding terminus.",
                this->name_.c_str());
        break;
      }

      // Wait until planning time has elapsed before returning.
      const ros::Duration wait_time(
        this->max_runtime_ - (ros::Time::now() - plan_start_time).toSec());
      wait_time.sleep();

      return GenerateTrajectory(terminus);
    }

    // Add this to the closed list.
    closed_registry.insert(next);

    // Expand and add to the list.
    for (const VectorXd& neighbor : Neighbors(next->point_)) {
      const S neighbor_state(neighbor);

      // Compute the time at which we'll reach this neighbor.
      const double best_neigh_time =
          (neighbor.isApprox(next->point_.Configuration(), 1e-8))
              ? kStayPutTime
              : this->dynamics_.BestPossibleTime(next->point_, neighbor_state);

      // Gotta sanity check if we got a valid neighbor time.
      if (std::isinf(best_neigh_time))
        throw std::runtime_error("Neighbor time was infinity.");

      const double neighbor_time = next->time_ + best_neigh_time;

      // Compute cost to get to the neighbor.
      const double neighbor_cost =
          ComputeCostToCome(next, neighbor_state, best_neigh_time);

      // Compute heuristic from neighbor to stop.
      const double neighbor_heuristic = ComputeHeuristic(neighbor_state, end);

      // Discard if this is on the closed list.
      const typename Node::Ptr neighbor_node =
          Node::Create(neighbor_state, next, neighbor_time, neighbor_cost,
                       neighbor_heuristic);
      if (closed_registry.count(neighbor_node) > 0) {
        continue;
      }

      // Collision check this line segment (and store the collision probability)
      if (!CollisionCheck(next->point_, neighbor_state, next->time_, neighbor_time)) {
        continue;
      }

      // Check if we're in the open set.
      auto match = open_registry.find(neighbor_node);
      if (match != open_registry.end()) {
        // We found a match in the open set.
        if (neighbor_node->priority_ < (*match)->priority_) {
          // Remove the node that matches
          // and replace it with the new/updated one.
          RemoveFromMultiset(*match, open);
          open.insert(neighbor_node);

          open_registry.erase(*match);
          open_registry.insert(neighbor_node);
        }
      } else {
        // If the neighbor is not in the open set, add him to it.
        open.insert(neighbor_node);
        open_registry.insert(neighbor_node);
      }
    }
  }
}

// Returns the total cost to get to point. best_time is the fastest time
// it takes to go from the parent to the point.
template <typename S, typename E, typename B, typename SB>
double TimeVaryingAStar<S, E, B, SB>::ComputeCostToCome(
    const typename Node::ConstPtr& parent, const S& point,
    double best_time) const {
  if (parent == nullptr) {
    ROS_FATAL("Parent should never be null when computing cost to come!");
    return std::numeric_limits<double>::infinity();
  }

  // If best_time is not valid or provided by the call, compute it now.
  if (best_time < 0.0)
    best_time = this->dynamics_.BestPossibleTime(parent->point_, point);

  // Cost to get to the parent contains distance + time. Add to this
  // the distance from the parent to the current point and the time.
  // option 1: parent->cost_to_come_ + best_time
  // option 2: (doesn't work!): parent->cost_to_come_ + best_time +
  //           0.001*(parent->point_ - point).norm();
  // option 3: parent->cost_to_come_ + (parent->point_ - point).norm();
  return parent->cost_to_come_ + best_time +
         (parent->point_ - point).Configuration().norm();
}

// Returns the heuristic for the point.
template <typename S, typename E, typename B, typename SB>
double TimeVaryingAStar<S, E, B, SB>::ComputeHeuristic(const S& point,
                                                       const S& stop) const {
  // This heuristic is the best possible distance + best possible time.
  // option 1: ComputeBestTime(point, stop)
  // option 2 (doesn't work!): ComputeBestTime(point, stop) + (point -
  //                           stop).norm()*0.1
  // option 3: (point - stop).norm();
  return this->dynamics_.BestPossibleTime(point, stop) +
         (point - stop).Configuration().norm();
}

// Helper function to find all neighboring nodes from the specified dimension
// to the final dimension.
template <typename S, typename E, typename B, typename SB>
std::list<VectorXd> TimeVaryingAStar<S, E, B, SB>::FindNeighborsFromDimension(
    const VectorXd& config, size_t dimension) const {
  std::list<VectorXd> neighbors;

  // Catch base case.
  if (dimension == config.size()) {
    neighbors.emplace_back(config);
  } else {
    // For each possible value in the current dimension, populate lists of
    // neighbors in the next dimension (recursively) and concatenate.
    for (double value = config[dimension] - grid_resolution_;
         value <= config[dimension] + grid_resolution_ + 1e-4;
         value += grid_resolution_) {
      VectorXd next_config = config;
      next_config[dimension] = value;

      neighbors.splice(neighbors.end(),
                       FindNeighborsFromDimension(next_config, dimension + 1));
    }
  }

  return neighbors;
}

// Get the neighbors of the given point on the implicit grid.
// NOTE! Include the given point.
template <typename S, typename E, typename B, typename SB>
std::list<VectorXd> TimeVaryingAStar<S, E, B, SB>::Neighbors(
    const S& point) const {
  const VectorXd config = point.Configuration();
  return FindNeighborsFromDimension(config, 0);
}

// Collision check a line segment between the two points with the given
// initial start time. Returns true if the path is collision free and
// false otherwise.
template <typename S, typename E, typename B, typename SB>
bool TimeVaryingAStar<S, E, B, SB>::CollisionCheck(const S& start,
                                                   const S& stop,
                                                   double start_time,
                                                   double stop_time) const {
  // Unpack configurations.
  const VectorXd start_config = start.Configuration();
  const VectorXd stop_config = stop.Configuration();

  // Need to check if collision checking identical configurations at different
  // times. In this case, we will have to avoid divide by zero issues in
  // computing the unit direction from start to stop.
  const bool same_pt = start_config.isApprox(stop_config, 1e-8);

  // Compute the vector pointing from start to stop, whose norm is the speed.
  constexpr double kSmallNumber = 1e-8;
  const VectorXd velocity =
    (stop_config - start_config) / std::max(kSmallNumber, stop_time - start_time);

  // Compute the dt between query points.
  constexpr double kStayPutIntervalCollisionCheckFraction = 0.1;
  const double dt =
      (same_pt)
          ? (stop_time - start_time) * kStayPutIntervalCollisionCheckFraction
    : collision_check_resolution_ / velocity.norm();

  // Start at the start point and walk until we get past the stop point.
  for (double time_offset = 0.0; time_offset < stop_time - start_time; time_offset += dt) {
    const Vector3d query = S(start_config + velocity * time_offset).Position();
    if (!this->env_.IsValid(query, this->bound_, start_time + time_offset)) return false;
  }

  return true;
}

// This function removes all instances of next with matching
// point and time values from the given multiset (there should only be 1).
template <typename S, typename E, typename B, typename SB>
void TimeVaryingAStar<S, E, B, SB>::RemoveFromMultiset(
    const typename Node::Ptr next,
    std::multiset<typename Node::Ptr, typename Node::NodeComparitor>& open) {
  // Get the range of nodes that have equal priority to next
  std::pair<typename std::multiset<typename Node::Ptr,
                                   typename Node::NodeComparitor>::iterator,
            typename std::multiset<typename Node::Ptr,
                                   typename Node::NodeComparitor>::iterator>
      matches = open.equal_range(next);

  // This guy lets us check the equality of two Nodes.
  typename Node::NodeEqual equality_checker;

  for (auto it = matches.first; it != matches.second; ++it) {
    // If the current iterate has the same time and point as next
    // remove the iterate from the open set
    if (equality_checker(*it, next)) {
      open.erase(it);
      return;
    }
  }
}

// Walk backward from the given node to the root to create a Trajectory.
template <typename S, typename E, typename B, typename SB>
Trajectory<S> TimeVaryingAStar<S, E, B, SB>::GenerateTrajectory(
    const typename Node::ConstPtr& node) const {
  // Start with an empty list of positions and times.
  std::vector<S> positions;
  std::vector<double> times;

  // Populate these lists by walking backward, then reverse.
  for (typename Node::ConstPtr n = node; n != nullptr; n = n->parent_) {
    positions.push_back(n->point_);
    times.push_back(n->time_);
  }

  std::reverse(positions.begin(), positions.end());
  std::reverse(times.begin(), times.end());

  // Create a trajectory.
  return Trajectory<S>(positions, times);
}

// Load parameters. Be sure to call the base KinematicPlanner::LoadParameters.
template <typename S, typename E, typename B, typename SB>
bool TimeVaryingAStar<S, E, B, SB>::LoadParameters(const ros::NodeHandle& n) {
  if (!KinematicPlanner<S, E, B, SB>::LoadParameters(n)) {
    ROS_ERROR("%s: Planner could not load parameters.", this->name_.c_str());
    return false;
  }

  ros::NodeHandle nl(n);

  if (!nl.getParam("grid_resolution", grid_resolution_)) return false;
  if (!nl.getParam("collision_check_resolution", collision_check_resolution_))
    return false;

  return true;
}
}  //\namespace planning
}  //\namespace fastrack

#endif
