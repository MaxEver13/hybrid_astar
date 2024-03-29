// Copyright (c) 2020, Samsung Research America
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
// limitations under the License. Reserved.

#ifndef SMAC_PLANNER__SMAC_PLANNER_HPP_
#define SMAC_PLANNER__SMAC_PLANNER_HPP_

#include <memory>
#include <vector>
#include <string>

#include "smac_planner/a_star.hpp"
#include "smac_planner/smoother.hpp"
#include "smac_planner/costmap_downsampler.hpp"

#include <ros/ros.h>

#include "nav_core/base_global_planner.h"
#include "nav_msgs/OccupancyGrid.h"
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include "costmap_2d/costmap_2d_ros.h"
#include "costmap_2d/costmap_2d.h"
#include <geometry_msgs/PoseStamped.h>

#include "tf2/utils.h"

namespace smac_planner
{

class SmacPlanner : public nav_core::BaseGlobalPlanner
{
public:
  /**
   * @brief constructor
   */
  SmacPlanner() = default;

  SmacPlanner(std::string name, costmap_2d::Costmap2D * costmap);

  void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros) override;

  bool makePlan(const geometry_msgs::PoseStamped& start,
                const geometry_msgs::PoseStamped& goal,
                std::vector<geometry_msgs::PoseStamped>& plan) override;


  /**
   * @brief Create an Eigen Vector2D of world poses from continuous map coords
   * @param mx float of map X coordinate
   * @param my float of map Y coordinate
   * @param costmap Costmap pointer
   * @return Eigen::Vector2d eigen vector of the generated path
   */
  Eigen::Vector2d getWorldCoords(
    const float & mx, const float & my, const costmap_2d::Costmap2D * costmap);

  /**
   * @brief Create quaternion from A* coord bins
   * @param theta continuous bin coordinates angle
   * @return quaternion orientation in map frame
   */
  geometry_msgs::Quaternion getWorldOrientation(const float & theta);

  /**
   * @brief Remove hooking at end of paths
   * @param path Path to remove hooking from
   */
  void removeHook(std::vector<Eigen::Vector2d> & path);

protected:
  // Node Handle
  ros::NodeHandle _nh;
  ros::NodeHandle _private_nh;
  std::unique_ptr<AStarAlgorithm> _a_star{nullptr};
  std::unique_ptr<Smoother> _smoother{nullptr};
  costmap_2d::Costmap2D * _costmap{nullptr};
  std::unique_ptr<CostmapDownsampler> _costmap_downsampler{nullptr};
  std::string _global_frame, _name;
  double _tolerance;
  int _downsampling_factor;
  unsigned int _angle_quantizations;
  double _angle_bin_size;
  bool _downsample_costmap;
  ros::Publisher _raw_plan_publisher;
  SmootherParams _smoother_params;
  OptimizerParams _optimizer_params;
  double _max_planning_time;
};

}  // namespace smac_planner

#endif  // SMAC_PLANNER__SMAC_PLANNER_HPP_
