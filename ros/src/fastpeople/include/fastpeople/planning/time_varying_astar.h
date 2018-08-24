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

#include <set>
#include <unordered_set>

namespace fastrack {
namespace planning {

using dynamics::Kinematics;

template <typename S, typename E, typename B, typename SB>
class TimeVaryingAStar : public KinematicPlanner<S, E, B, SB> {
public:
  ~TimeVaryingAStar() {}
  /*
  explicit TimeVaryingAStar(ValueFunctionId incoming_value,
                   ValueFunctionId outgoing_value,
                   const E::ConstPtr& space,
                   const Dynamics::ConstPtr& dynamics,      
                   double grid_resolution,
                   double collision_check_resolution)
    : incoming_value_(incoming_value),
      outgoing_value_(outgoing_value),
      space_(space),
      dynamics_(dynamics)
      grid_resolution_(grid_resolution),
      collision_check_resolution_(collision_check_resolution) {
    if (incoming_value_ + 1 != outgoing_value_)
      ROS_ERROR("Outgoing value function not successor to incoming one.");
  }
  */

  explicit TimeVaryingAStar() : KinematicPlanner<S, E, B, SB>() {}


  // Plan a trajectory from the given start to goal states starting
  // at the given time.
  // NOTE! The states in the output trajectory are essentially configurations.
  Trajectory<S> Plan(const S &start, const S &goal,
                     double start_time = 0.0) const;

private:
  // Load parameters. Be sure to call the base KinematicPlanner::LoadParameters.
  bool LoadParameters(const ros::NodeHandle &n);


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
    double collision_prob_;

    // Factory method.
    static inline Ptr Create(const S& point,
                             const ConstPtr& parent,
                             double time,
                             double cost_to_come,
                             double heuristic) {
      Ptr ptr(new Node(point, parent, time, cost_to_come, heuristic));
      return ptr;
    }
/*
    inline void PrintNode(const double start_time=0.0) const {
      std::cout << "---- Node Info: ----\n";
      std::cout << "  id: " << id_ << std::endl;
      std::cout << "  point: [" << point_.Configuration().transpose() << "]" << std::endl;
      std::cout << "  time: " << (time_ - start_time) << std::endl;
      std::cout << "  cost_to_come: " << cost_to_come_ << std::endl;
      std::cout << "  heuristic: " << heuristic_ << std::endl;
      std::cout << "  priority: " << priority_ << std::endl;
      std::cout << "  collision prob: " << collision_prob_ << std::endl;
      if (parent_ != nullptr)
        std::cout << "  parent id: " << parent_->id_ << std::endl;
    }
*/
    // Comparitor. Returns true if heuristic cost of Node 1 < for Node 2.
    struct NodeComparitor {
      inline bool operator()(const Node::Ptr& node1,
                             const Node::Ptr& node2) const {
        return node1->priority_ < node2->priority_;
      }
    }; // class NodeComparitor

    // Equality. Returns true if Node 1 space-time is same as Node 2 space-time.
    struct NodeEqual {
      inline bool operator()(const Node::Ptr& node1,
                             const Node::Ptr& node2) const {
         return (std::abs(node1->time_ - node2->time_) < 1e-8 &&
              node1->point_.Configuration().isApprox(node2->point_.Configuration(), 1e-8));
      }
    }; // class NodeEqual

    // Custom hash functor.
    struct NodeHasher {
      inline bool operator()(const Node::Ptr& node) const {
        size_t seed = 0;

        // Hash this node's contents together.
        //   boost::hash_combine(seed, boost::hash_value(node->priority_));
        boost::hash_combine(seed, boost::hash_value(node->time_));
        for (int d = 0; d < node->point_.Configuration().size(); d += 1) {
          boost::hash_combine(seed, boost::hash_value(node->point_.ToVector()(d)));
        }
        //        boost::hash_combine(seed, boost::hash_value(node->parent_));
        return seed;
      }
    };

  private:
    explicit Node(const S& point, const ConstPtr& parent,
                  double time, double cost_to_come, double heuristic)
      : 
        point_(point), 
        parent_(parent), 
        time_(time), 
        cost_to_come_(cost_to_come),
        heuristic_(heuristic),
        priority_(heuristic + cost_to_come),
        collision_prob_(0.0) {}

  }; //\struct Node


  // Collision check a line segment between the two points with the given
  // start and stop times. Returns true if the path is collision free and
  // false otherwise.
  bool CollisionCheck(const S& start, const S& stop,
                      double start_time, double stop_time, 
                      double& collision_prob) const;

  // This function removes all instances of next with matching
  // point and time values from the given multiset (there should only be 1). 
  static void RemoveFromMultiset(const typename Node::Ptr next, 
    std::multiset<typename Node::Ptr, typename Node::NodeComparitor>& open);


  // Walk backward from the given node to the root to create a Trajectory.
  Trajectory<S> GenerateTrajectory(const typename Node::ConstPtr& node) const;

  // Helper function populating neighbors in each dimension
  std::vector<S> NeighborsFinder(const S& point, VectorXd neighbor, int dimension) const;

  // Get the neighbors of the given point on the implicit grid.
  // NOTE! Include the given point.
  std::vector<S> Neighbors(const S& point) const;

  // Returns the total cost to get to point.
  double ComputeCostToCome(const typename Node::ConstPtr& parent, 
    const S& point, 
    double dt=-std::numeric_limits<double>::infinity()) const;

  // Returns the heuristic for the point.
  double ComputeHeuristic(const S& point, 
    const S& stop) const;

/*
  // Returns the best possible time from point to stop.
  static double ComputeBestTime(const S& point, 
    const S& stop) const;
*/
  // Keep our own notion of the environment because base class doesn't assume
  // ProbabilisticBox class. 

  // Side length of virtual grid cells.
  double grid_resolution_;

  // Maximum distance between test points on line segments being
  // collision checked.
  double collision_check_resolution_;

  // Stores max speed that quad can go (in all directions)


}; //\class TimeVaryingAStar



// --------------------------- IMPLEMENTATION ------------------------------- //

// Plan a trajectory from the given start to goal states starting
// at the given time.
// NOTE! The states in the output trajectory are essentially configurations.
template <typename S, typename E, typename B, typename SB>
Trajectory<S> TimeVaryingAStar<S, E, B, SB>::Plan(const S &start, const S &end,
                                                  double start_time) const {
  const ros::Time plan_start_time = ros::Time::now();
  constexpr double kStayPutTime = 1.0;

  // Make a set for the open set. This stores the candidate "fringe" nodes 
  // we might want to expand. This is sorted by priority and allows multiple 
  // nodes with the same priority to be in the list. 
  typename std::multiset<typename Node::Ptr, typename Node::NodeComparitor> open;

  // Make a hash set for the open set. The key in this hash table
  // is space-time for a node. This is used to find nodes with the same 
  // space-time key in log time.
  typename std::unordered_set<typename Node::Ptr, 
    typename Node::NodeHasher, typename Node::NodeEqual> open_registry;

  // Make a hash set for the closed set. The key in this hash table
  // is space-time for a node. This is used to find nodes with the same 
  // space-time key in constant time.
  typename std::unordered_set<typename Node::Ptr, 
         typename Node::NodeHasher, typename Node::NodeEqual> closed_registry;

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
    if (open.size() != open_registry.size()){
      throw std::runtime_error("open and open_registry are not the same size!");
    }

    //if ((ros::Time::now() - plan_start_time).toSec() > budget)
    //  return nullptr;

    if (open.empty()){
      ROS_ERROR_THROTTLE(1.0, "%s: Open list is empty.", KinematicPlanner<S, E, B, SB>::name_.c_str());
      return Trajectory<S>();
    }

    const typename Node::Ptr next = *open.begin();

    // TODO this is for debugging!
    // next->PrintNode(start_time);
    
    ROS_INFO("%s: Open list size: %zu, Next priority: %f", 
       KinematicPlanner<S, E, B, SB>::name_.c_str(), open.size(), next->priority_);

    // Pop the next node from the open_registry and open set.
    open_registry.erase(next); // works because keys in open_registry are unique!    
    RemoveFromMultiset(next, open); 

    // Check if this guy is the goal.
    if (std::sqrt((next->point_.X() - end.X())*(next->point_.X() - end.X())
      + (next->point_.Y() - end.Y())*(next->point_.Y() - end.Y()) 
      + (next->point_.Z() - end.Z())*(next->point_.Z() - end.Z())) 
      < grid_resolution_*0.5) {
      const typename Node::ConstPtr parent_node = (next->parent_ == nullptr) ? 
      next : next->parent_;

      // Have to connect the goal point to the last sampled grid point.
      const double best_time = KinematicPlanner<S, E, B, SB>::dynamics_.BestPossibleTime(parent_node->point_, end);
      const double terminus_time = parent_node->time_ + best_time;
      const double terminus_cost = 
        ComputeCostToCome(parent_node, end, best_time);
      const double terminus_heuristic = 0.0;

      const typename Node::Ptr terminus = 
      Node::Create(end, parent_node, terminus_time, terminus_cost, 
         terminus_heuristic);
      return GenerateTrajectory(terminus);
    }

    // Add this to the closed list.
    closed_registry.insert(next);

    // Expand and add to the list.
    for (const S& neighbor : Neighbors(next->point_)) {
      // Compute the time at which we'll reach this neighbor.
      const double best_neigh_time = (neighbor.Configuration().isApprox(next->point_.Configuration(), 1e-8)) ? 
        kStayPutTime : KinematicPlanner<S, E, B, SB>::dynamics_.BestPossibleTime(next->point_, neighbor);
      //BestPossibleTime(next->point_, neighbor);

      // Gotta sanity check if we got a valid neighbor time.
      if (std::isinf(best_neigh_time)) 
        throw std::runtime_error("neighbor time infinity");

      const double neighbor_time = next->time_ + best_neigh_time;      

      // Compute cost to get to the neighbor.
      const double neighbor_cost = 
        ComputeCostToCome(next, neighbor, best_neigh_time);

      // Compute heuristic from neighbor to stop.
      const double neighbor_heuristic = ComputeHeuristic(neighbor, end);

      // Discard if this is on the closed list.
      const typename Node::Ptr neighbor_node =
        Node::Create(neighbor, next, neighbor_time, neighbor_cost, neighbor_heuristic);      
      if (closed_registry.count(neighbor_node) > 0) {
        //neighbor_node->PrintNode(start_time);
        ROS_WARN("Did not add neighbor_node because its already on the closed list.");
        continue;
      }

      // Collision check this line segment (and store the collision probability)
      if (!CollisionCheck(next->point_, neighbor, next->time_, 
        neighbor_time, neighbor_node->collision_prob_)) {
        //neighbor_node->PrintNode(start_time);
        ROS_WARN("Did not add neighbor_node because its in collision!");
        continue;
      }

      ROS_INFO("Adding neighbor node...");
      //neighbor_node->PrintNode(start_time);

      // Check if we're in the open set.
      auto match = open_registry.find(neighbor_node);
      if (match != open_registry.end()) {
 
#if 0       
        std::cout << "I found a match between candidate: \n";
        neighbor_node->PrintNode(start_time);
        std::cout << " and open list node: \n";
        (*match)->PrintNode(start_time);
#endif

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

// Returns the total cost to get to point.
template <typename S, typename E, typename B, typename SB>
double TimeVaryingAStar<S, E, B, SB>::ComputeCostToCome(const typename Node::ConstPtr& parent, 
  const S& point, double dt) const{

  if(parent == nullptr){
    ROS_FATAL("Parent should never be null when computing cost to come!");
    return std::numeric_limits<double>::infinity();
  }

  if (dt < 0.0)
    dt = KinematicPlanner<S, E, B, SB>::dynamics_.BestPossibleTime(parent->point_, point); //BestPossibleTime(parent->point_, point);  

  // Cost to get to the parent contains distance + time. Add to this
  // the distance from the parent to the current point and the time

  // option 1: parent->cost_to_come_ + dt
  // option 2 (doesn't work!): parent->cost_to_come_ + dt + 0.001*(parent->point_ - point).norm();
  // option 3: parent->cost_to_come_ + (parent->point_ - point).norm();
  return parent->cost_to_come_ + dt + (parent->point_ - point).Configuration().norm();
}

// Returns the heuristic for the point.
template <typename S, typename E, typename B, typename SB>
double TimeVaryingAStar<S, E, B, SB>::ComputeHeuristic(const S& point, 
  const S& stop) const{

  // This heuristic is the best possible distance + best possible time. 
  // option 1: ComputeBestTime(point, stop)
  // option 2 (doesn't work!): ComputeBestTime(point, stop) + (point - stop).norm()*0.1
  // option 3: (point - stop).norm();
  return KinematicPlanner<S, E, B, SB>::dynamics_.BestPossibleTime(point, stop) + (point - stop).Configuration().norm();
}

/*
// Computes the best possible time by looking at the largest coordinate diff.
template <typename S, typename E, typename B, typename SB>
double TimeVaryingAStar<S, E, B, SB>::ComputeBestTime(const S& point, 
    const S& stop) const {
  S diff = (point - stop).cwiseAbs();
  double max_diff = std::max(diff[0], std::max(diff[1], diff[2]));
  return max_diff/max_speed_;
}
*/

// helper function populating the neighbors of a given point in all dimension
template <typename S, typename E, typename B, typename SB>
std::vector<S> TimeVaryingAStar<S, E, B, SB>::NeighborsFinder(const S& point, VectorXd neighbor, int dimension) const {
  if (dimension == -1) {
    std::vector<S> neighbors;
    S s;
    s.FromVector(neighbor);
    neighbors.push_back(s);
    return neighbors;
  } else {
    VectorXd config = point.Configuration();
    for (double d = config(dimension) - grid_resolution_; 
        d < config(dimension) + grid_resolution_ + 1e-8;
        d += grid_resolution_) {
      neighbor(dimension) = d;
    return NeighborsFinder(point, neighbor, dimension - 1);
    }
  }
}

// Get the neighbors of the given point on the implicit grid.
// NOTE! Include the given point.
template <typename S, typename E, typename B, typename SB>
std::vector<S> TimeVaryingAStar<S, E, B, SB>::Neighbors(const S& point) const {
  VectorXd config = point.Configuration();
  int dimension = config.size();
  VectorXd new_config = point.Configuration();
  return NeighborsFinder(point, new_config, dimension - 1);
}

// Collision check a line segment between the two points with the given
// initial start time. Returns true if the path is collision free and
// false otherwise.
template <typename S, typename E, typename B, typename SB>
bool TimeVaryingAStar<S, E, B, SB>::CollisionCheck(const S& start, const S& stop,
                                    double start_time, double stop_time, 
                                    double& max_collision_prob) const {

  // Need to check if collision checking against yourself
  const bool same_pt = start.Configuration().isApprox(stop, 1e-8);

  // Compute the unit vector pointing from start to stop.
  const S direction = (same_pt) ? S::Zero() : 
    static_cast<S>((stop - start) / (stop - start).Configuration().norm());

  // Compute the dt between query points.
  const double dt = (same_pt) ? (stop_time-start_time) * 0.1 : 
    (stop_time - start_time) * collision_check_resolution_ / 
    (stop - start).Configuration().norm();

  double collision_prob = 0.0;
  max_collision_prob = 0.0;
  // Start at the start point and walk until we get past the stop point.
  S query(start);
  for (double time = start_time; time < stop_time; time += dt) {
    const bool valid_pt = 
      KinematicPlanner<S, E, B, SB>::space_->IsValid(query, KinematicPlanner<S, E, B, SB>::bound_); //*****************************************
      //space_->IsValid(query, incoming_value_, outgoing_value_, collision_prob, time);

    if (collision_prob > max_collision_prob)
      max_collision_prob = collision_prob;

    if (!valid_pt)
      return false;

    // Take a step.
    query += collision_check_resolution_ * direction;
  }

  return true;
}


// This function removes all instances of next with matching
// point and time values from the given multiset (there should only be 1).
template <typename S, typename E, typename B, typename SB>
void TimeVaryingAStar<S, E, B, SB>::RemoveFromMultiset(const typename Node::Ptr next, 
  std::multiset<typename Node::Ptr, typename Node::NodeComparitor>& open) {
  // Get the range of nodes that have equal priority to next
  std::pair<
    typename std::multiset<typename Node::Ptr, typename Node::NodeComparitor>::iterator,
    typename std::multiset<typename Node::Ptr, typename Node::NodeComparitor>::iterator> matches =
     open.equal_range(next);

  // This guy lets us check the equality of two Nodes.
  typename Node::NodeEqual equality_checker;

  for (auto it=matches.first; it!=matches.second; ++it) {
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
  std::vector<double> collision_probs;

  //  std::cout << "Collision probabilities for generated trajectory:\n";
  // Populate these lists by walking backward, then reverse.
  for (typename Node::ConstPtr n = node; n != nullptr; n = n->parent_) {
    positions.push_back(n->point_);
    times.push_back(n->time_);
    collision_probs.push_back(n->collision_prob_);
  }
  //  std::cout << std::endl;

  std::reverse(positions.begin(), positions.end());
  std::reverse(times.begin(), times.end());
  // std::reverse(collision_probs.begin(), collision_probs.end());

  // Lift positions into states.
  //const std::vector<S> states =
  //  KinematicPlanner<S, E, B, SB>::dynamics_.LiftGeometricTrajectory(positions, times);

  // Create dummy list containing value function IDs.
  //const std::vector<ValueFunctionId> values(states.size(), incoming_value_); 
  //ROS_INFO("Returning Trajectory of length %zu.", positions.size());
  
  // Create a trajectory.
  return Trajectory<S>(positions, times);
}



// Load parameters. Be sure to call the base KinematicPlanner::LoadParameters.
template <typename S, typename E, typename B, typename SB>
bool TimeVaryingAStar<S, E, B, SB>::LoadParameters(const ros::NodeHandle &n) {
  if (!KinematicPlanner<S, E, B, SB>::LoadParameters(n)) {
    ROS_ERROR("%s: Planner could not load parameters.", this->name_.c_str());
    return false;
  }

  ros::NodeHandle nl(n);

  // Load extra parameters from the parameter server.
  // TODO!
  return true;
}
} //\namespace planning
} //\namespace fastrack
#endif
