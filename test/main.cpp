#include <iostream>
#include "smac_planner/smac_planner.hpp"
#include "utils/tf_utils.h"

#include <tf/transform_listener.h>

#include <nav_msgs/OccupancyGrid.h>

#include "grid_map_core/GridMap.hpp"
#include "grid_map_core/iterators/GridMapIterator.hpp"
#include "grid_map_core/gtest_eigen.hpp"
#include "grid_map_ros/GridMapRosConverter.hpp"
#include "grid_map_msgs/GridMap.h"
#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_costmap_2d/grid_map_costmap_2d.hpp>
#include "grid_map_costmap_2d/Costmap2DConverter.hpp"




// Costmap
std::shared_ptr<costmap_2d::Costmap2D> costmap = nullptr;

// Global planner
std::shared_ptr<smac_planner::SmacPlanner> planner = nullptr;      


// TF
// tf2_ros::Buffer tf_buffer;
std::string map_frame_id = "map";
std::string robot_frame_id = "base_link";


// Planner star and goal
nav_msgs::OccupancyGrid occupancyGrid;
geometry_msgs::PoseStamped start;
geometry_msgs::PoseStamped goal;
bool is_start_set = false;
bool is_costmap_set = false;


// Publisher
ros::Publisher goal_pub;

void mapCallBack(const nav_msgs::OccupancyGrid::ConstPtr& msg);
void goalCallBack(const geometry_msgs::PoseStamped::ConstPtr& msg);
void startCallBack(const nav_msgs::Odometry::ConstPtr& msg);

void mapCallBack(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    occupancyGrid = *msg.get();
    occupancyGrid.data.resize(occupancyGrid.info.width * occupancyGrid.info.height);

    ROS_WARN("width: %d, height: %d", occupancyGrid.info.width, occupancyGrid.info.height);

    // for (auto& cell : occupancyGrid.data) {
    //     cell = rand() % 102 - 1; // [-1, 100]
    // }

    // Convert to grid map.
    const std::string layer("layer");
    grid_map::GridMap gridMap;
    grid_map::GridMapRosConverter::fromOccupancyGrid(occupancyGrid, layer, gridMap);

    ROS_WARN_THROTTLE(5.0, "gridmap size: %lld", gridMap.getSize());

    // Create costmap2d.
    grid_map::Costmap2DConverter<grid_map::GridMap, grid_map::Costmap2DCenturyTranslationTable> costmap2dConverter;
    // costmap_2d::Costmap2D my_costmap;
    costmap2dConverter.initializeFromGridMap(gridMap, *(costmap.get()));

    ROS_WARN("X: %f, Y:%f", costmap->getSizeInMetersX(), costmap->getSizeInMetersY());

    // Copy data.
    costmap2dConverter.setCostmap2DFromGridMap(gridMap, layer, *(costmap.get()));

    is_costmap_set = true;
}

// start point callback
void startCallBack(const nav_msgs::Odometry::ConstPtr& msg)
{
    // ROS_WARN_THROTTLE(5.0, "curr linear vel: %f, angular vel: %f", msg->twist.twist.linear.x, msg->twist.twist.angular.z);

    // 设置当前位姿
    try {
        tf::TransformListener tf_listener;
        tf::StampedTransform transform;
        tf_listener.waitForTransform(map_frame_id, robot_frame_id, ros::Time(0), ros::Duration(1.0));
        tf_listener.lookupTransform(map_frame_id, robot_frame_id, ros::Time(0), transform);
        start = tf_utils::transform2pose(transform);
        is_start_set = true;

        // ROS_WARN_THROTTLE(5.0, "start pose:[%f, %f]", start.pose.position.x, start.pose.position.y);
    }
    catch (tf2::TransformException &ex) {
        ROS_WARN_STREAM(ex.what());
    }
}

// goal point callback and path plan
void goalCallBack(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    ROS_WARN("frame_id: %s, map_frame_id: %s", msg->header.frame_id.c_str(), map_frame_id.c_str());
  
    if (msg->header.frame_id == map_frame_id)
    {
        goal = *msg;
        ROS_WARN("start:[%f, %f]", start.pose.position.x, start.pose.position.y);
        ROS_WARN("Goal:[%f, %f]", goal.pose.position.x, goal.pose.position.y);
        // if start point , new goal and costmap are settled
        if(is_start_set && is_costmap_set) {
            std::vector<geometry_msgs::PoseStamped> plan;
            // SmacPlanner
            planner.reset(new smac_planner::SmacPlanner("smac_planner", costmap.get()));
            auto res = planner->makePlan(start, goal, plan);
            ROS_WARN("res: %d, path size: %d", res, plan.size());
            if (res) {
                goal_pub.publish(plan.back());
            }
        }
    }
    else
    {
        ROS_WARN_STREAM("The path must be published in the " << map_frame_id
                        << " frame! Ignoring path in " << msg->header.frame_id
                        << " frame!");
    }  

    // reset
    is_start_set = false;
    // is_costmap_set = false;
}





int main(int argc, char** argv)
{
    // Handle
    ros::init(argc, argv, "smac_planner");
    ros::NodeHandle n;
    ros::NodeHandle nh("~");

    // Initializer for costmap
    costmap = std::make_shared<costmap_2d::Costmap2D>();

    // Subscriber for receiving map
    ros::Subscriber map_sub = n.subscribe("map", 1, mapCallBack);
    // ROS_WARN("X: %f, Y:%f", costmap->getSizeInMetersX(), costmap->getSizeInMetersY());

    // RosParam 
    nh.param<std::string>("global_frame", map_frame_id, "map");
    nh.param<std::string>("robot_base_frame", robot_frame_id, "base_link");
    ROS_WARN("robot_frame_id: %s, map_frame_id: %s", robot_frame_id.c_str(), map_frame_id.c_str());
    // Subscriber for getting planner start 
    ros::Subscriber start_sub = n.subscribe("odometry", 1, startCallBack);

    // Subscriber for receiving planner goal
    ros::Subscriber goal_sub = n.subscribe("goal", 1, goalCallBack);

    // Smac planner
    planner =  std::make_shared<smac_planner::SmacPlanner>("smac_planner", costmap.get());


    // Publisher 
    goal_pub = n.advertise<geometry_msgs::PoseStamped>("planned_goal", 1);

    

    ros::spin();
    
    return 0;
}