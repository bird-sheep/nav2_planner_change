/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020 Shivang Patel
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Shivang Patel
 *
 * Reference tutorial:
 * https://navigation.ros.org/tutorials/docs/writing_new_nav2planner_plugin.html
 *********************************************************************/

#include <cmath>
#include <string>
#include <memory>
#include "nav2_util/node_utils.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include "nav2_straightline_planner/straight_line_planner.hpp"
#include "lib_route.h"
#include "fstream"

namespace nav2_straightline_planner
{

void StraightLine::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent.lock();
  name_ = name;
  tf_ = tf;
  costmap_ = costmap_ros->getCostmap();
  global_frame_ = costmap_ros->getGlobalFrameID();

  // Parameter initialization
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".interpolation_resolution", rclcpp::ParameterValue(
      0.1));
  node_->get_parameter(name_ + ".interpolation_resolution", interpolation_resolution_);

}

bool
StraightLine::worldToMap(double wx, double wy, unsigned int & mx, unsigned int & my)
{
  if (wx < costmap_->getOriginX() || wy < costmap_->getOriginY()) {
    return false;
  }

  mx = static_cast<int>(
    std::round((wx - costmap_->getOriginX()) / costmap_->getResolution()));
  my = static_cast<int>(
    std::round((wy - costmap_->getOriginY()) / costmap_->getResolution()));

  if (mx < costmap_->getSizeInCellsX() && my < costmap_->getSizeInCellsY()) {
    return true;
  }

  RCLCPP_ERROR(
    node_->get_logger(),
    "worldToMap failed: mx,my: %d,%d, size_x,size_y: %d,%d", mx, my,
    costmap_->getSizeInCellsX(), costmap_->getSizeInCellsY());

  return false;
}

void
StraightLine::mapToWorld(double mx, double my, double & wx, double & wy)
{
  wx = costmap_->getOriginX() + mx * costmap_->getResolution();
  wy = costmap_->getOriginY() + my * costmap_->getResolution();
}
void StraightLine::cleanup()
{
  RCLCPP_INFO(
    node_->get_logger(), "CleaningUp plugin %s of type NavfnPlanner",
    name_.c_str());
}

void StraightLine::activate()
{
  RCLCPP_INFO(
    node_->get_logger(), "Activating plugin %s of type NavfnPlanner",
    name_.c_str());
}

void StraightLine::deactivate()
{
  RCLCPP_INFO(
    node_->get_logger(), "Deactivating plugin %s of type NavfnPlanner",
    name_.c_str());
}



std::vector<std::vector<int>> getCostmapAsVector(nav2_costmap_2d::Costmap2D* costmap_)
{

    int width = costmap_->getSizeInCellsX();
  int height = costmap_->getSizeInCellsY();

  std::vector<std::vector<int>> map;
  map.resize(height, std::vector<int>(width));

  // std::cout<<"here is map size: "<<height<<", "<<width<<std::endl;
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      unsigned char cost = costmap_->getCost(y, x);
      int tmp_cost = static_cast<int>(cost);
      if(tmp_cost >=235)
          map[y][x] = 1;
      else
           map[y][x] = 0;
      // std::cout<<map[y][x]<<" ";
    }
    // std::cout<<std::endl;
  }


  // map.resize(width, std::vector<int>(height));

  // for (int x = 0; x < width; ++x) {
  //   for (int y = 0; y < height; ++y) {
  //     unsigned char cost = costmap_->getCost(x, y);
  //     map[x][y] = static_cast<int>(cost);
  //     // map[x][y] = 0;
  //   }
  // }
 
  return map;
}

nav_msgs::msg::Path StraightLine::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  nav_msgs::msg::Path global_path;

  // Checking if the goal and start state is in the global frame
  if (start.header.frame_id != global_frame_) {
    RCLCPP_ERROR(
      node_->get_logger(), "Planner will only except start position from %s frame",
      global_frame_.c_str());
    return global_path;
  }

  if (goal.header.frame_id != global_frame_) {
    RCLCPP_INFO(
      node_->get_logger(), "Planner will only except goal position from %s frame",
      global_frame_.c_str());
    return global_path;
  }

  global_path.poses.clear();
  global_path.header.stamp = node_->now();
  global_path.header.frame_id = global_frame_;

  // set the start goal map
  std::ofstream out("path.txt");
  std::streambuf *coutbuf = std::cout.rdbuf(); // 保存 cout 原始的缓冲区指针
  std::cout.rdbuf(out.rdbuf()); // 将 cout 的输出重定向到文件流 out

  int width = costmap_->getSizeInCellsX();
  int height = costmap_->getSizeInCellsY();
  std::cout<<"here is map size, witdh: "<<width<<", height: "<<height<<std::endl;

  unsigned int start_mx, start_my, goal_mx, goal_my;
  worldToMap(start.pose.position.x, start.pose.position.y, start_mx, start_my);
  worldToMap(goal.pose.position.x, goal.pose.position.y, goal_mx, goal_my);
  Point vec_start(start_mx, start_my);
  Point vec_goal(goal_mx, goal_my);
  // Point vec_start(start.pose.position.y/0.05 + height/2, start.pose.position.x/0.05 + width/2);
  // Point vec_goal(goal.pose.position.y/0.05 + height/2, goal.pose.position.x/0.05 + width/2);

  std::cout<<"pose_start:"<<start.pose.position.x<<", "<<start.pose.position.y<<std::endl;
  std::cout<<"vec_start: "<<vec_start.x<<", "<<vec_start.y<<std::endl;
   std::cout<<"pose_goal:"<<goal.pose.position.x<<", "<<goal.pose.position.y<<std::endl;
  std::cout<<"vec_goal: "<<vec_goal.x<<", "<<vec_goal.y<<std::endl;
  std::vector<std::vector<int>> vec_map = getCostmapAsVector(costmap_);
  // get the path and yaw
  Pathdata vec_path_yaw = findPath(vec_map, vec_start, vec_goal);


  for (int i = 0; i < vec_path_yaw.path.size(); ++i) {
      geometry_msgs::msg::PoseStamped pose;
      mapToWorld(vec_path_yaw.path[i].x, vec_path_yaw.path[i].y, pose.pose.position.x, pose.pose.position.y);
      // pose.pose.position.x = 0.05*(vec_path_yaw.path[i].y -width/2);
      // pose.pose.position.y = 0.05*( vec_path_yaw.path[i].x -height/2);
      pose.pose.position.z = 0.0;
      double yaw = vec_path_yaw.yaw[i];

      std::cout<<"pose_x: "<<pose.pose.position.x<<" pose_y: "<<pose.pose.position.y<<" yaw: "<<yaw<<std::endl;

      tf2::Quaternion quat;
      quat.setRPY(0, 0, yaw); // Assuming roll and pitch are 0
      pose.pose.orientation.x = quat.x();
      pose.pose.orientation.y = quat.y();
      pose.pose.orientation.z = quat.z();
      pose.pose.orientation.w = quat.w();

      pose.header.stamp = node_->now();
      pose.header.frame_id = global_frame_;
      global_path.poses.push_back(pose);
    }
     std::cout.rdbuf(coutbuf);
  // calculating the number of loops for current value of interpolation_resolution_

  // int total_number_of_loop = std::hypot(
  //   goal.pose.position.x - start.pose.position.x,
  //   goal.pose.position.y - start.pose.position.y) /
  //   interpolation_resolution_;
  // double x_increment = (goal.pose.position.x - start.pose.position.x) / total_number_of_loop;
  // double y_increment = (goal.pose.position.y - start.pose.position.y) / total_number_of_loop;

  // for (int i = 0; i < total_number_of_loop; ++i) {
  //   geometry_msgs::msg::PoseStamped pose;
  //   pose.pose.position.x = start.pose.position.x + x_increment * i;
  //   pose.pose.position.y = start.pose.position.y + y_increment * i;
  //   pose.pose.position.z = 0.0;
  //   pose.pose.orientation.x = 0.0;
  //   pose.pose.orientation.y = 0.0;
  //   pose.pose.orientation.z = 0.0;
  //   pose.pose.orientation.w = 1.0;
  //   pose.header.stamp = node_->now();
  //   pose.header.frame_id = global_frame_;
  //   global_path.poses.push_back(pose);
  // }

  geometry_msgs::msg::PoseStamped goal_pose = goal;
  goal_pose.header.stamp = node_->now();
  goal_pose.header.frame_id = global_frame_;
  global_path.poses.push_back(goal_pose);

  return global_path;
}

}  // namespace nav2_straightline_planner

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_straightline_planner::StraightLine, nav2_core::GlobalPlanner)
