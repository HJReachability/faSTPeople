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

#ifndef FASTPEOPLE_ENVIRONMENT_OCCUPANCY_GRID_TIME_INTEPOLATOR_H
#define FASTPEOPLE_ENVIRONMENT_OCCUPANCY_GRID_TIME_INTEPOLATOR_H

#include <crazyflie_human/OccupancyGridTime.h>
#include <fastrack/bound/box.h>
#include <fastrack/utils/types.h>
#include <fastrack/utils/uncopyable.h>

#include <ros/ros.h>

namespace fastrack {
namespace environment {

using bound::Box;

class OccupancyGridTimeInterpolator {
 public:
  ~OccupancyGridTimeInterpolator() {}
  explicit OccupancyGridTimeInterpolator(
      const crazyflie_human::OccupancyGridTime::ConstPtr& msg, double lower_x,
      double upper_x, double lower_y, double upper_y);

  // Get occupancy probability at the given position at the given time.
  double OccupancyProbability(const Vector3d& position, double time) const;

  // Get total occupancy probability within the given tracking error bound,
  // centered at the given position, at the given time.
  double OccupancyProbability(const Vector3d& position, const Box& bound,
                              double time) const;

 private:
  struct OccupancyGrid {
    std::vector<double> data;
    double resolution;
    size_t num_cells_x;
    size_t num_cells_y;
  };  //\struct OccupancyGrid

  // Get the 1D grid index of the cell which includes the given point.
  size_t PointToIndex(double x, double y, const OccupancyGrid& grid) const;

  // Lower and upper bounds in x/y dimensions.
  const double lower_x_, upper_x_;
  const double lower_y_, upper_y_;

  // Store msg as map from time to flattened occupancy grid.
  std::map<double, OccupancyGrid> occupancy_grids_;
};  //\class OccupancyGridTimeInterpolator

}  // namespace environment
}  // namespace fastrack

#endif
