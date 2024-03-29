// Copyright (c) 2020, Samsung Research America
// Copyright (c) 2020, Applied Electric Vehicles Pty Ltd
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

#ifndef SMAC_PLANNER__A_STAR_HPP_
#define SMAC_PLANNER__A_STAR_HPP_

#include <vector>
#include <iostream>
#include <unordered_map>
#include <memory>
#include <queue>
#include <utility>
#include "Eigen/Core"

#include "costmap_2d/costmap_2d.h"

#include "smac_planner/node_se2.hpp"
#include "smac_planner/node_basic.hpp"
#include "smac_planner/types.hpp"
#include "smac_planner/constants.hpp"

namespace smac_planner
{

inline double squaredDistance(
  const Eigen::Vector2d & p1,
  const Eigen::Vector2d & p2)
{
  const double & dx = p1[0] - p2[0];
  const double & dy = p1[1] - p2[1];
  return hypot(dx, dy);
}

/**
 * @class smac_planner::AStarAlgorithm
 * @brief An A* implementation for planning in a costmap. */
class AStarAlgorithm
{
public:
  typedef NodeSE2 * NodePtr;
  typedef std::unordered_map<unsigned int, NodeSE2> Graph;
  typedef std::vector<NodePtr> NodeVector;
  typedef std::pair<float, NodeBasic<NodeSE2>> NodeElement;
  typedef typename NodeSE2::Coordinates Coordinates;
  typedef typename NodeSE2::CoordinateVector CoordinateVector;
  typedef typename NodeVector::iterator NeighborIterator;
  typedef std::function<bool (const unsigned int &, NodeSE2 * &)> NodeGetter;

  /**
   * @struct smac_planner::NodeComparator
   * @brief Node comparison for priority queue sorting
   */
  struct NodeComparator
  {
    bool operator()(const NodeElement & a, const NodeElement & b) const
    {
      return a.first > b.first;
    }
  };

  typedef std::priority_queue<NodeElement, std::vector<NodeElement>, NodeComparator> NodeQueue;

  /**
   * @brief A constructor for smac_planner::PlannerServer
   * @param neighborhood The type of neighborhood to use for search (4 or 8 connected)
   */
  explicit AStarAlgorithm(const MotionModel & motion_model, const SearchInfo & search_info);

  /**
   * @brief A destructor for smac_planner::AStarAlgorithm
   */
  ~AStarAlgorithm();

  /**
   * @brief Initialization of the planner with defaults
   * @param allow_unknown Allow search in unknown space, good for navigation while mapping
   * @param max_iterations Maximum number of iterations to use while expanding search
   * @param max_on_approach_iterations Maximum number of iterations before returning a valid
   * path once within thresholds to refine path
   * comes at more compute time but smoother paths.
   */
  void initialize(
    const bool & allow_unknown,
    int & max_iterations,
    const int & max_on_approach_iterations);

  /**
   * @brief Creating path from given costmap, start, and goal
   * @param path Reference to a vector of indicies of generated path
   * @param num_iterations Reference to number of iterations to create plan
   * @param tolerance Reference to tolerance in costmap nodes
   * @return if plan was successful
   */
  bool createPath(CoordinateVector & path, int & num_iterations, const float & tolerance);

  /**
   * @brief Create the graph based on the node type. For 2D nodes, a cost grid.
   *   For 3D nodes, a SE2 grid without cost info as needs collision detector for footprint.
   * @param x The total number of nodes in the X direction
   * @param y The total number of nodes in the X direction
   * @param dim_3 The total number of nodes in the theta or Z direction
   * @param costmap Costmap to convert into the graph
   */
  void createGraph(
    const unsigned int & x,
    const unsigned int & y,
    const unsigned int & dim_3,
    costmap_2d::Costmap2D * & costmap);

  /**
   * @brief Set the goal for planning, as a node index
   * @param mx The node X index of the goal
   * @param my The node Y index of the goal
   * @param dim_3 The node dim_3 index of the goal
   */
  void setGoal(
    const unsigned int & mx,
    const unsigned int & my,
    const unsigned int & dim_3);

  /**
   * @brief Set the starting pose for planning, as a node index
   * @param mx The node X index of the goal
   * @param my The node Y index of the goal
   * @param dim_3 The node dim_3 index of the goal
   */
  void setStart(
    const unsigned int & mx,
    const unsigned int & my,
    const unsigned int & dim_3);

  /**
   * @brief Set the footprint
   * @param footprint footprint of robot
   * @param use_radius Whether this footprint is a circle with radius
   */
  void setFootprint(const std::vector<geometry_msgs::Point>& footprint, bool use_radius);

  /**
   * @brief Perform an analytic path expansion to the goal
   * @param node The node to start the analytic path from
   * @param getter The function object that gets valid nodes from the graph
   * @return Node pointer to goal node if successful, else return nullptr
   */
  NodePtr getAnalyticPath(const NodePtr & node, const NodeGetter & getter);

  /**
   * @brief Set the starting pose for planning, as a node index
   * @param node Node pointer to the goal node to backtrace
   * @param path Reference to a vector of indicies of generated path
   * @return whether the path was able to be backtraced
   */
  bool backtracePath(NodePtr & node, CoordinateVector & path);

  /**
   * @brief Get maximum number of iterations to plan
   * @return Reference to Maximum iterations parameter
   */
  int & getMaxIterations();

  /**
   * @brief Get pointer reference to starting node
   * @return Node pointer reference to starting node
   */
  NodePtr & getStart();

  /**
   * @brief Get pointer reference to goal node
   * @return Node pointer reference to goal node
   */
  NodePtr & getGoal();

  /**
   * @brief Get maximum number of on-approach iterations after within threshold
   * @return Reference to Maximum on-appraoch iterations parameter
   */
  int & getOnApproachMaxIterations();

  /**
   * @brief Get tolerance, in node nodes
   * @return Reference to tolerance parameter
   */
  float & getToleranceHeuristic();

  /**
   * @brief Get size of graph in X
   * @return Size in X
   */
  unsigned int & getSizeX();

  /**
   * @brief Get size of graph in Y
   * @return Size in Y
   */
  unsigned int & getSizeY();

  /**
   * @brief Get number of angle quantization bins (SE2) or Z coordinate  (XYZ)
   * @return Number of angle bins / Z dimension
   */
  unsigned int & getSizeDim3();

protected:
  /**
   * @brief Get pointer to next goal in open set
   * @return Node pointer reference to next heuristically scored node
   */
  inline NodePtr getNextNode();

  /**
   * @brief Get pointer to next goal in open set
   * @param cost The cost to sort into the open set of the node
   * @param node Node pointer reference to add to open set
   */
  inline void addNode(const float cost, NodePtr & node);

  /**
   * @brief Adds node to graph
   * @param cost The cost to sort into the open set of the node
   * @param node Node pointer reference to add to open set
   */
  inline NodePtr addToGraph(const unsigned int & index);

  /**
   * @brief Check if this node is the goal node
   * @param node Node pointer to check if its the goal node
   * @return if node is goal
   */
  inline bool isGoal(NodePtr & node);

  /**
   * @brief Get cost of traversal between nodes
   * @param current_node Pointer to current node
   * @param new_node Pointer to new node
   * @return Reference traversal cost between the nodes
   */
  inline float getTraversalCost(NodePtr & current_node, NodePtr & new_node);

  /**
   * @brief Get total cost of traversal for a node
   * @param node Pointer to current node
   * @return Reference accumulated cost between the nodes
   */
  inline float & getAccumulatedCost(NodePtr & node);

  /**
   * @brief Get cost of heuristic of node
   * @param node Node index current
   * @param node Node index of new
   * @return Heuristic cost between the nodes
   */
  inline float getHeuristicCost(const NodePtr & node);

  /**
   * @brief Check if inputs to planner are valid
   * @return Are valid
   */
  inline bool areInputsValid();

  /**
   * @brief Clear hueristic queue of nodes to search
   */
  inline void clearQueue();

  /**
   * @brief Clear graph of nodes searched
   */
  inline void clearGraph();

  /**
   * @brief Attempt an analytic path completion
   * @return Node pointer reference to goal node if successful, else
   * return nullptr
   */
  inline NodePtr tryAnalyticExpansion(
    const NodePtr & current_node,
    const NodeGetter & getter, int & iterations, int & best_cost);

  bool _traverse_unknown;
  int _max_iterations;
  int _max_on_approach_iterations;
  float _tolerance;
  unsigned int _x_size;
  unsigned int _y_size;
  unsigned int _dim3_size;
  SearchInfo _search_info;

  Coordinates _goal_coordinates;
  NodePtr _start;
  NodePtr _goal;

  Graph _graph;
  NodeQueue _queue;

  MotionModel _motion_model;
  NodeHeuristicPair _best_heuristic_node;

  GridCollisionChecker _collision_checker;
  std::vector<geometry_msgs::Point> _footprint;
  bool _is_radius_footprint;
  costmap_2d::Costmap2D * _costmap;
};

}  // namespace smac_planner

#endif  // SMAC_PLANNER__A_STAR_HPP_
