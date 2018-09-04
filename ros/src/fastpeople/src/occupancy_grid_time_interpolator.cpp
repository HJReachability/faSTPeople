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
// Utility for accessing the time-indexed occupancy grid msg.
//
///////////////////////////////////////////////////////////////////////////////

#include <crazyflie_human/OccupancyGridTime.h>
#include <fastpeople/environment/occupancy_grid_time_interpolator.h>
#include <fastrack/bound/box.h>
#include <fastrack/utils/types.h>

#include <ros/ros.h>

namespace fastrack {
namespace environment {

OccupancyGridTimeInterpolator::OccupancyGridTimeInterpolator(
    const crazyflie_human::OccupancyGridTime::ConstPtr& msg, double lower_x,
    double upper_x, double lower_y, double upper_y)
    : lower_x_(lower_x),
      upper_x_(upper_x),
      lower_y_(lower_y),
      upper_y_(upper_y) {
  // Make sure upper bounds are above lower bounds.
  if (lower_x_ > upper_x_ || lower_y_ > upper_y_)
    throw std::runtime_error("Invalid lower/upper bounds.");

  // Step through each grid in msg and populate our local copy.
  for (const auto& grid : msg->gridarray) {
    occupancy_grids_.emplace(
        grid.header.stamp.toSec(),
        OccupancyGrid{grid.data, static_cast<double>(grid.resolution),
                      static_cast<size_t>(grid.width),
                      static_cast<size_t>(grid.height)});
  }

  // Make sure we have at least one occupancy grid.
  if (occupancy_grids_.empty())
    throw std::runtime_error("Empty OccupancyGridTime msg.");
}

// Get occupancy probability at the given position at the given time.
double OccupancyGridTimeInterpolator::OccupancyProbability(
    const Vector3d& position, double time) const {
  // Find the grid with time just past the given time.
  auto hi = occupancy_grids_.upper_bound(time);
  auto lo = hi;
  if (hi == occupancy_grids_.begin()) {
    // The query time is before all occupancy grids we have.
    ROS_WARN("OccupancyGridTimeInterpolator: interpolation time is too early.");
  } else {
    lo--;
  }

  // 'lo' is definitely a valid iterator, so find the probability there.
  const size_t lo_idx = PointToIndex(position(0), position(1), lo->second);
  const double lo_prob = (lo_idx < lo->second.data.size()) ? lo->second.data[lo_idx] : 0.0;
  constexpr double kSmallNumber = 1e-4;
  if (lo_prob < -kSmallNumber || lo_prob > 1.0 + kSmallNumber);
    throw std::runtime_error("Invalid probability encountered: 1 " + std::to_string(lo_prob));

  // If 'hi' is not a valid iterator, then just return what we have.
  if (hi == occupancy_grids_.end()) {
    ROS_WARN_THROTTLE(2.0, "OccupancyGridTimeInterpolator: interpolation time is too late.");
    return lo_prob;
  }

  // Extract probability at 'hi' and interpolate.
  const size_t hi_idx = PointToIndex(position(0), position(1), hi->second);
  const double hi_prob = (hi_idx < hi->second.data.size()) ? hi->second.data[hi_idx] : 0.0;
  if (hi_prob < -kSmallNumber || hi_prob > 1.0 + kSmallNumber);
    throw std::runtime_error("Invalid probability encountered: 2 " + std::to_string(hi_prob));;

  const double hi_fraction = (time - lo->first) / (hi->first - lo->first);
  return hi_fraction * hi_prob + (1.0 - hi_fraction) * lo_prob;
}

// Get total occupancy probability within the given tracking error bound,
// centered at the given position, at the given time.
double OccupancyGridTimeInterpolator::OccupancyProbability(
    const Vector3d& position, const Box& bound, double time) const {
  // Find the grid with time just past the given time.
  auto hi = occupancy_grids_.upper_bound(time);
  auto lo = hi;
  if (hi == occupancy_grids_.begin()) {
    // The query time is before all occupancy grids we have.
    ROS_WARN("OccupancyGridTimeInterpolator: interpolation time is too early.");
  } else {
    lo--;
  }

  // 'lo' is definitely a valid iterator, so find the probability there.
  double lo_prob = 0.0;
  constexpr double kSmallNumber = 1e-4;
  for (double x = position(0) - bound.x; x < position(0) + bound.x;
       x += lo->second.resolution) {
    for (double y = position(1) - bound.y; y < position(1) + bound.y;
         y += lo->second.resolution) {
      const size_t idx = PointToIndex(x, y, lo->second);
      const double prob = (idx < lo->second.data.size()) ? lo->second.data[idx] : 0.0;

      if (prob < -kSmallNumber || prob > 1.0 + kSmallNumber)
         throw std::runtime_error("Invalid probability encountered: 3 " + std::to_string(prob));
      lo_prob += prob;
    }
  }

  if (lo_prob < -kSmallNumber || lo_prob > 1.0 + kSmallNumber) {
    throw std::runtime_error("Invalid accumulated pre probability: " + 
      std::to_string(lo_prob));
  }

  // If 'hi' is not a valid iterator, then just return what we have.
  if (hi == occupancy_grids_.end()) {
    ROS_WARN_THROTTLE(1.0, "OccupancyGridTimeInterpolator: interpolation time is too late.");
    return lo_prob;
  }

  // Extract probability at 'hi' and interpolate.
  double hi_prob = 0.0;
  for (double x = position(0) - bound.x; x < position(0) + bound.x;
       x += hi->second.resolution) {
    for (double y = position(1) - bound.y; y < position(1) + bound.y;
         y += hi->second.resolution) {
      const size_t idx = PointToIndex(x, y, hi->second);
      const double prob = (idx < hi->second.data.size()) ? hi->second.data[idx] : 0.0;

      if (prob < -kSmallNumber || prob > 1.0 + kSmallNumber)
         throw std::runtime_error("Invalid probability encountered: 4 " + std::to_string(prob));
      hi_prob += prob;
    }
  }

  if (hi_prob < -kSmallNumber || hi_prob > 1.0 + kSmallNumber)
    throw std::runtime_error("Invalid accumulated post probability: " + 
      std::to_string(hi_prob));

  const double hi_fraction = (time - lo->first) / (hi->first - lo->first);
  if (hi_fraction < 0.0 || hi_fraction > 1.0)
    throw std::runtime_error("Invalid interpolation fraction: " + std::to_string(hi_fraction));

  return hi_fraction * hi_prob + (1.0 - hi_fraction) * lo_prob;
}

// Get the 1D grid index of the cell which includes the given point.
size_t OccupancyGridTimeInterpolator::PointToIndex(
    double x, double y, const OccupancyGrid& grid) const {
  // If out of bounds, return max index and handle later. This may happen if we
  // attempt to query near the edges.
  if (x < lower_x_ || x >= upper_x_ || y < lower_y_ || y >= upper_y_)
    return std::numeric_limits<size_t>::max();

  // NOTE: Assuming layout is
  //                       |-:-|-:-|...|-:-|-:-|
  //                      xmin                xmax
  // and that upper-left corner in the 'data' grid is lower-left corner in
  // Cartesian space.
  const size_t ii = static_cast<size_t>((x - lower_x_) / grid.resolution);
  const size_t jj = static_cast<size_t>((y - lower_y_) / grid.resolution);

  return jj + grid.num_cells_y * ii;
}

}  //\namespace environment
}  //\namespace fastrack
