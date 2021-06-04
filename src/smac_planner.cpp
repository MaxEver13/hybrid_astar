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

#include <string>
#include <memory>
#include <vector>
#include <algorithm>
#include <limits>
#include <chrono>
#include <mutex>

#include "Eigen/Core"
#include "smac_planner/smac_planner.hpp"

#define BENCHMARK_TESTING

using namespace ros;

using namespace std::chrono;

namespace smac_planner
{

SmacPlanner::SmacPlanner(std::string name, costmap_2d::Costmap2D * costmap)
: _nh(""), _private_nh("~")
{
  _costmap = costmap;
  _name = name;
  _global_frame = "map";

  bool allow_unknown;
  int max_iterations;
  int max_on_approach_iterations = std::numeric_limits<int>::max();
  int angle_quantizations;
  SearchInfo search_info;
  bool smooth_path;
  std::string motion_model_for_search;

  // General planner params
  _private_nh.param<double>("tolerance", _tolerance, 0.125);
  _private_nh.param("downsample_costmap", _downsample_costmap, false);
  _private_nh.param<int>("downsampling_factor", _downsampling_factor, 1); 


  _private_nh.param<int>("angle_quantization_bins", angle_quantizations, 72); 
  _angle_bin_size = 2.0 * M_PI / angle_quantizations;
  _angle_quantizations = static_cast<unsigned int>(angle_quantizations);
  
  _private_nh.param("allow_unknown", allow_unknown, true); 
  _private_nh.param<int>("max_iterations", max_iterations, -1);
  _private_nh.param<int>("max_on_approach_iterations", max_on_approach_iterations, 1000);  
  _private_nh.param("smooth_path", smooth_path, false); 

  _private_nh.param<float>("minimum_turning_radius", search_info.minimum_turning_radius, 0.2); 
  _private_nh.param<float>("reverse_penalty", search_info.reverse_penalty, 2.0); 
  _private_nh.param<float>("change_penalty", search_info.change_penalty, 0.5);   
  _private_nh.param<float>("non_straight_penalty", search_info.non_straight_penalty, 1.05); 
  _private_nh.param<float>("cost_penalty", search_info.cost_penalty, 1.2); 
  _private_nh.param<float>("analytic_expansion_ratio", search_info.analytic_expansion_ratio, 2.0); 


  _private_nh.param<double>("max_planning_time", _max_planning_time, 5.0); 

  _private_nh.param<std::string>("motion_model_for_search", motion_model_for_search, "MOORE"); 
  MotionModel motion_model = fromString(motion_model_for_search);
  if (motion_model == MotionModel::UNKNOWN) {
    ROS_WARN(    
      "Unable to get MotionModel search type. Given '%s', valid options are MOORE, VON_NEUMANN, DUBIN, REEDS_SHEPP.",
      motion_model_for_search.c_str());
  }

  if (max_on_approach_iterations <= 0) {
    ROS_INFO("On approach iteration selected as <= 0, disabling tolerance and on approach iterations.");
    max_on_approach_iterations = std::numeric_limits<int>::max();
  }

  if (max_iterations <= 0) {
    ROS_INFO("maximum iteration selected as <= 0, disabling maximum iterations.");
    max_iterations = std::numeric_limits<int>::max();
  }

  // convert to grid coordinates
  const double minimum_turning_radius_global_coords = search_info.minimum_turning_radius;
  search_info.minimum_turning_radius =
    search_info.minimum_turning_radius / (_costmap->getResolution() * _downsampling_factor);

  _a_star = std::make_unique<AStarAlgorithm>(motion_model, search_info);
  _a_star->initialize(
    allow_unknown,
    max_iterations,
    max_on_approach_iterations);

  std::vector<geometry_msgs::Point> footprint;
  geometry_msgs::Point p0, p1, p2, p3;
  p0.x = -0.3; p0.y = -0.3; p0.z = 0.0;
  p1.x = 0.3; p1.y = -0.3; p1.z = 0.0;
  p2.x = 0.3; p2.y = 0.3; p2.z = 0.0;
  p3.x = -0.3; p3.y = 0.3; p3.z = 0.0;
  footprint.push_back(p0);
  footprint.push_back(p1);
  footprint.push_back(p2);
  footprint.push_back(p3);

  _a_star->setFootprint(footprint, false);

  // _a_star->setFootprint(costmap_ros->getRobotFootprint(), false);

  // Smoother Params
  _private_nh.param<double>("curvature_weight", _smoother_params.curvature_weight, 1.5);
  _private_nh.param<double>("costmap_weight", _smoother_params.costmap_weight, 0.0);
  _private_nh.param<double>("distance_weight", _smoother_params.distance_weight, 0.0);
  _private_nh.param<double>("smooth_weight", _smoother_params.smooth_weight, 15000.0);
  _private_nh.param<double>("costmap_factor", _smoother_params.costmap_factor, 10.0);
  _smoother_params.max_curvature = 1.0f / minimum_turning_radius_global_coords;

  // Optimizer Params
  _private_nh.param<double>("param_tol", _optimizer_params.param_tol, 1e-15);
  _private_nh.param<double>("fn_tol", _optimizer_params.fn_tol, 1e-7); 
  _private_nh.param<double>("gradient_tol", _optimizer_params.gradient_tol, 1e-10);
  _private_nh.param<int>("opt_max_iterations", _optimizer_params.max_iterations, 500);
  _private_nh.param<double>("max_time", _optimizer_params.max_time, 0.100);  
  _private_nh.param("debug_optimizer", _optimizer_params.debug, false); 
  // Advanced Params
  _private_nh.param<double>("min_line_search_step_size", _optimizer_params.advanced.min_line_search_step_size, 1e-20);
  _private_nh.param<int>("max_num_line_search_step_size_iterations", _optimizer_params.advanced.max_num_line_search_step_size_iterations, 50);
  _private_nh.param<double>("line_search_sufficient_function_decrease", _optimizer_params.advanced.line_search_sufficient_function_decrease, 1e-20);  
  _private_nh.param<int>("max_num_line_search_direction_restarts", _optimizer_params.advanced.max_num_line_search_direction_restarts, 10);
  _private_nh.param<int>("max_line_search_step_expansion", _optimizer_params.advanced.max_line_search_step_expansion, 50);
  

  if (smooth_path) {
    _smoother = std::make_unique<Smoother>();  
    _smoother->initialize(_optimizer_params);
  }

  if (_downsample_costmap && _downsampling_factor > 1) {
    std::string topic_name = "downsampled_costmap";
    _costmap_downsampler = std::make_unique<CostmapDownsampler>(&_nh, _global_frame, topic_name, _costmap, _downsampling_factor);
  }

  _raw_plan_publisher = _nh.advertise<nav_msgs::Path>("plan", 1);

  ROS_INFO("Configured plugin %s of type SmacPlanner with tolerance %.2f, maximum iterations %i, max on approach iterations %i, and allow_unknown: %d, Using motion model: %s.",
    _name.c_str(), _tolerance, max_iterations, max_on_approach_iterations, allow_unknown, toString(motion_model).c_str());

}


void SmacPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
{
  
}


bool SmacPlanner::makePlan(const geometry_msgs::PoseStamped& start,
                const geometry_msgs::PoseStamped& goal,
                std::vector<geometry_msgs::PoseStamped>& plan)
{
  steady_clock::time_point a = steady_clock::now();

  std::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(_costmap->getMutex()));

  // Downsample costmap, if required
  costmap_2d::Costmap2D * costmap = _costmap;
  if (_costmap_downsampler) {
    costmap = _costmap_downsampler->downsample(_downsampling_factor);
  }

  ROS_INFO("X: %f, Y:%f", costmap->getSizeInMetersX(), costmap->getSizeInMetersY());

  ROS_INFO("size_x: %d, size_y: %d, resolution: %f", costmap->getSizeInCellsX(), costmap->getSizeInCellsY(), costmap->getResolution());
  // Set Costmap
  _a_star->createGraph(
    costmap->getSizeInCellsX(),
    costmap->getSizeInCellsY(),
    _angle_quantizations,
    costmap);

  // Set starting point, in A* bin search coordinates
  unsigned int mx, my;
  costmap->worldToMap(start.pose.position.x, start.pose.position.y, mx, my);
  double orientation_bin = tf2::getYaw(start.pose.orientation) / _angle_bin_size;
  while (orientation_bin < 0.0) {
    orientation_bin += static_cast<float>(_angle_quantizations);
  }
  unsigned int orientation_bin_id = static_cast<unsigned int>(floor(orientation_bin));
  _a_star->setStart(mx, my, orientation_bin_id);

  // Set goal point, in A* bin search coordinates
  costmap->worldToMap(goal.pose.position.x, goal.pose.position.y, mx, my);
  orientation_bin = tf2::getYaw(goal.pose.orientation) / _angle_bin_size;
  while (orientation_bin < 0.0) {
    orientation_bin += static_cast<float>(_angle_quantizations);
  }
  orientation_bin_id = static_cast<unsigned int>(floor(orientation_bin));
  _a_star->setGoal(mx, my, orientation_bin_id);

  // Setup message
  nav_msgs::Path plan_pub;
  plan_pub.header.stamp = ros::Time::now();
  plan_pub.header.frame_id = _global_frame;
  geometry_msgs::PoseStamped pose;
  pose.header = plan_pub.header;
  pose.pose.position.z = 0.0;
  pose.pose.orientation.x = 0.0;
  pose.pose.orientation.y = 0.0;
  pose.pose.orientation.z = 0.0;
  pose.pose.orientation.w = 1.0;

  // Compute plan
  NodeSE2::CoordinateVector path;
  int num_iterations = 0;
  std::string error;
  try {
    if (!_a_star->createPath(
        path, num_iterations, _tolerance / static_cast<float>(costmap->getResolution())))
    {
      if (num_iterations < _a_star->getMaxIterations()) {
        error = std::string("no valid path found");
      } else {
        ROS_WARN("exceeded maximum iterations: %d", _a_star->getMaxIterations());
        error = std::string("exceeded maximum iterations");
      }
    }
  } catch (const std::runtime_error & e) {
    error = "invalid use: ";
    error += e.what();
  }

  if (!error.empty()) {
    ROS_WARN("%s: failed to create plan, %s.",
      _name.c_str(), error.c_str());
    return false;
  }

  // Convert to world coordinates and downsample path for smoothing if necesssary
  // We're going to downsample by 4x to give terms room to move.
  const int downsample_ratio = 4;
  std::vector<Eigen::Vector2d> path_world;
  path_world.reserve(path.size());
  plan.reserve(path.size());
  plan_pub.poses.reserve(path.size());

  for (int i = path.size() - 1; i >= 0; --i) {
    path_world.push_back(getWorldCoords(path[i].x, path[i].y, costmap));
    pose.pose.position.x = path_world.back().x();
    pose.pose.position.y = path_world.back().y();
    pose.pose.orientation = getWorldOrientation(path[i].theta);
    plan_pub.poses.push_back(pose);
    plan.push_back(pose);
  }

  // Publish raw path for debug
  if (_raw_plan_publisher.getNumSubscribers() > 0) {
    ROS_INFO("Publish raw path ...");
    _raw_plan_publisher.publish(plan_pub);
  }

  // If not smoothing or too short to smooth, return path
  if (!_smoother || path_world.size() < 4) {
#ifdef BENCHMARK_TESTING
    steady_clock::time_point b = steady_clock::now();
    duration<double> time_span = duration_cast<duration<double>>(b - a);
    std::cout << "It took " << time_span.count() * 1000 <<
      " milliseconds with " << num_iterations << " iterations." << std::endl;
#endif
    return true;
  }

  // Find how much time we have left to do smoothing
  steady_clock::time_point b = steady_clock::now();
  duration<double> time_span = duration_cast<duration<double>>(b - a);
  double time_remaining = _max_planning_time - static_cast<double>(time_span.count());
  _smoother_params.max_time = std::min(time_remaining, _optimizer_params.max_time);

  // Smooth plan
  if (!_smoother->smooth(path_world, costmap, _smoother_params)) {
    ROS_WARN("%s: failed to smooth plan, Ceres could not find a usable solution to optimize.",
      _name.c_str());
    return false;
  }

  removeHook(path_world);

  // populate final path
  // TODO(stevemacenski): set orientation to tangent of path
  for (uint i = 0; i != path_world.size(); i++) {
    pose.pose.position.x = path_world[i][0];
    pose.pose.position.y = path_world[i][1];
    plan[i] = pose;
  }

  return true;
}

void SmacPlanner::removeHook(std::vector<Eigen::Vector2d> & path)
{
  // Removes the end "hooking" since goal is locked in place
  Eigen::Vector2d interpolated_second_to_last_point;
  interpolated_second_to_last_point = (path.end()[-3] + path.end()[-1]) / 2.0;
  if (
    squaredDistance(path.end()[-2], path.end()[-1]) >
    squaredDistance(interpolated_second_to_last_point, path.end()[-1]))
  {
    path.end()[-2] = interpolated_second_to_last_point;
  }
}

Eigen::Vector2d SmacPlanner::getWorldCoords(
  const float & mx, const float & my, const costmap_2d::Costmap2D * costmap)
{
  // mx, my are in continuous grid coordinates, must convert to world coordinates
  double world_x =
    static_cast<double>(costmap->getOriginX()) + (mx + 0.5) * costmap->getResolution();
  double world_y =
    static_cast<double>(costmap->getOriginY()) + (my + 0.5) * costmap->getResolution();
  return Eigen::Vector2d(world_x, world_y);
}

geometry_msgs::Quaternion SmacPlanner::getWorldOrientation(const float & theta)
{
  // theta is in continuous bin coordinates, must convert to world orientation
  tf2::Quaternion q;
  q.setEuler(0.0, 0.0, theta * static_cast<double>(_angle_bin_size));
  return tf2::toMsg(q);
}

}  // namespace smac_planner

