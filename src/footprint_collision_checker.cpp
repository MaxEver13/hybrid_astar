// Copyright (c) 2019 Intel Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Modified by: Shivang Patel (shivaang14@gmail.com)
// Modified by: Bartosz Meglicki (meglickib@gmail.com>)



#include <algorithm>
#include <cmath>


#include "smac_planner/footprint_collision_checker.hpp"
#include "smac_planner/line_iterator.hpp"
#include "smac_planner/constants.hpp"

namespace smac_planner
{


FootprintCollisionChecker::FootprintCollisionChecker()
: costmap_(nullptr)
{
}


FootprintCollisionChecker::FootprintCollisionChecker(
  costmap_2d::Costmap2D *costmap)
: costmap_(costmap)
{
}

double FootprintCollisionChecker::footprintCost(const Footprint footprint)
{
  // now we really have to lay down the footprint in the costmap_ grid
  unsigned int x0, x1, y0, y1;
  double footprint_cost = 0.0;

  // we need to rasterize each line in the footprint
  for (unsigned int i = 0; i < footprint.size() - 1; ++i) {
    // get the cell coord of the first point
    if (!worldToMap(footprint[i].x, footprint[i].y, x0, y0)) {
      return static_cast<double>(OCCUPIED);
    }

    // get the cell coord of the second point
    if (!worldToMap(footprint[i + 1].x, footprint[i + 1].y, x1, y1)) {
      return static_cast<double>(OCCUPIED);
    }

    footprint_cost = std::max(lineCost(x0, x1, y0, y1), footprint_cost);
  }

  // we also need to connect the first point in the footprint to the last point
  // get the cell coord of the last point
  if (!worldToMap(footprint.back().x, footprint.back().y, x0, y0)) {
    return static_cast<double>(OCCUPIED);
  }

  // get the cell coord of the first point
  if (!worldToMap(footprint.front().x, footprint.front().y, x1, y1)) {
    return static_cast<double>(OCCUPIED);
  }

  footprint_cost = std::max(lineCost(x0, x1, y0, y1), footprint_cost);

  // if all line costs are legal... then we can return that the footprint is legal
  return footprint_cost;
}

double FootprintCollisionChecker::lineCost(int x0, int x1, int y0, int y1) const
{
  double line_cost = 0.0;
  double point_cost = -1.0;

  for (LineIterator line(x0, y0, x1, y1); line.isValid(); line.advance()) {
    point_cost = pointCost(line.getX(), line.getY());   // Score the current point

    if (line_cost < point_cost) {
      line_cost = point_cost;
    }
  }

  return line_cost;
}

bool FootprintCollisionChecker::worldToMap(
  double wx, double wy, unsigned int & mx, unsigned int & my)
{
  return costmap_->worldToMap(wx, wy, mx, my);
}

double FootprintCollisionChecker::pointCost(int x, int y) const
{
  return costmap_->getCost(x, y);
}

void FootprintCollisionChecker::setCostmap(costmap_2d::Costmap2D *costmap)
{
  costmap_ = costmap;
}

double FootprintCollisionChecker::footprintCostAtPose(
  double x, double y, double theta, const Footprint footprint)
{
  double cos_th = cos(theta);
  double sin_th = sin(theta);
  Footprint oriented_footprint;
  for (unsigned int i = 0; i < footprint.size(); ++i) {
	  geometry_msgs::Point new_pt;
    new_pt.x = x + (footprint[i].x * cos_th - footprint[i].y * sin_th);
    new_pt.y = y + (footprint[i].x * sin_th + footprint[i].y * cos_th);
    oriented_footprint.push_back(new_pt);
  }

  return footprintCost(oriented_footprint);
}

}  // namespace smac_planner
