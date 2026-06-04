// Copyright 2020 Anshumaan Singh
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <vector>
#include <memory>
#include <string>
#include <limits>
#include "nav2_theta_star_oriented_planner/theta_star_planner.hpp"
#include "nav2_theta_star_oriented_planner/theta_star.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/utils.h"

namespace nav2_theta_star_oriented_planner
{
void ThetaStarOrientedPlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  planner_ = std::make_unique<theta_star::ThetaStar>();
  parent_node_ = parent;
  auto node = parent_node_.lock();
  logger_ = node->get_logger();
  clock_ = node->get_clock();
  name_ = name;
  tf_ = tf;
  planner_->costmap_ = costmap_ros->getCostmap();
  global_frame_ = costmap_ros->getGlobalFrameID();

  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".how_many_corners", rclcpp::ParameterValue(8));

  node->get_parameter(name_ + ".how_many_corners", planner_->how_many_corners_);

  if (planner_->how_many_corners_ != 8 && planner_->how_many_corners_ != 4) {
    planner_->how_many_corners_ = 8;
    RCLCPP_WARN(logger_, "Your value for - .how_many_corners  was overridden, and is now set to 8");
  }

  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".allow_unknown", rclcpp::ParameterValue(true));
  node->get_parameter(name_ + ".allow_unknown", planner_->allow_unknown_);

  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".w_euc_cost", rclcpp::ParameterValue(1.0));
  node->get_parameter(name_ + ".w_euc_cost", planner_->w_euc_cost_);

  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".w_traversal_cost", rclcpp::ParameterValue(2.0));
  node->get_parameter(name_ + ".w_traversal_cost", planner_->w_traversal_cost_);

  planner_->w_heuristic_cost_ = planner_->w_euc_cost_ < 1.0 ? planner_->w_euc_cost_ : 1.0;

  nav2_util::declare_parameter_if_not_declared(
    node, name + ".use_final_approach_orientation", rclcpp::ParameterValue(false));
  node->get_parameter(name + ".use_final_approach_orientation", use_final_approach_orientation_);

  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".proximity_threshold", rclcpp::ParameterValue(1.0));
  node->get_parameter(name_ + ".proximity_threshold", proximity_threshold_);

  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".orientation_delta", rclcpp::ParameterValue(0.2));
  node->get_parameter(name_ + ".orientation_delta", orientation_delta_);

  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".escape_distance", rclcpp::ParameterValue(0.4));
  node->get_parameter(name_ + ".escape_distance", escape_distance_);

  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".escape_lateral_range", rclcpp::ParameterValue(0.1));
  node->get_parameter(name_ + ".escape_lateral_range", escape_lateral_range_);
}

void ThetaStarOrientedPlanner::cleanup()
{
  RCLCPP_INFO(logger_, "CleaningUp plugin %s of type nav2_theta_star_planner", name_.c_str());
  planner_.reset();
}

void ThetaStarOrientedPlanner::activate()
{
  RCLCPP_INFO(logger_, "Activating plugin %s of type nav2_theta_star_planner", name_.c_str());
  // Add callback for dynamic parameters
  auto node = parent_node_.lock();
  dyn_params_handler_ = node->add_on_set_parameters_callback(
    std::bind(&ThetaStarOrientedPlanner::dynamicParametersCallback, this, std::placeholders::_1));
}

void ThetaStarOrientedPlanner::deactivate()
{
  RCLCPP_INFO(logger_, "Deactivating plugin %s of type nav2_theta_star_planner", name_.c_str());
}

nav_msgs::msg::Path ThetaStarOrientedPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  std::function<bool()> cancel_checker)
{
  nav_msgs::msg::Path global_path;
  auto start_time = std::chrono::steady_clock::now();

  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(planner_->costmap_->getMutex()));

  // Corner case of start and goal beeing on the same cell
  unsigned int mx_start, my_start, mx_goal, my_goal;
  if (!planner_->costmap_->worldToMap(
      start.pose.position.x, start.pose.position.y, mx_start, my_start))
  {
    throw nav2_core::StartOutsideMapBounds(
            "Start Coordinates of(" + std::to_string(start.pose.position.x) + ", " +
            std::to_string(start.pose.position.y) + ") was outside bounds");
  }

  if (!planner_->costmap_->worldToMap(
      goal.pose.position.x, goal.pose.position.y, mx_goal, my_goal))
  {
    throw nav2_core::GoalOutsideMapBounds(
            "Goal Coordinates of(" + std::to_string(goal.pose.position.x) + ", " +
            std::to_string(goal.pose.position.y) + ") was outside bounds");
  }

  if (planner_->costmap_->getCost(mx_start, my_start) == nav2_costmap_2d::LETHAL_OBSTACLE) {
    throw nav2_core::StartOccupied(
            "Start Coordinates of(" + std::to_string(start.pose.position.x) + ", " +
            std::to_string(start.pose.position.y) + ") was in lethal cost");
  }

  if (planner_->costmap_->getCost(mx_goal, my_goal) == nav2_costmap_2d::LETHAL_OBSTACLE) {
    throw nav2_core::GoalOccupied(
            "Goal Coordinates of(" + std::to_string(goal.pose.position.x) + ", " +
            std::to_string(goal.pose.position.y) + ") was in lethal cost");
  }

  if (mx_start == mx_goal && my_start == my_goal) {
    global_path.header.stamp = clock_->now();
    global_path.header.frame_id = global_frame_;
    geometry_msgs::msg::PoseStamped pose;
    pose.header = global_path.header;
    pose.pose.position.z = 0.0;

    pose.pose = start.pose;
    // if we have a different start and goal orientation, set the unique path pose to the goal
    // orientation, unless use_final_approach_orientation=true where we need it to be the start
    // orientation to avoid movement from the local planner
    if (start.pose.orientation != goal.pose.orientation && !use_final_approach_orientation_) {
      pose.pose.orientation = goal.pose.orientation;
    }
    global_path.poses.push_back(pose);
    return global_path;
  }

  // If an obstacle is within escape_distance_, compute a repulsion-based escape waypoint,
  // plan start to escape then escape to goal and concatenate.
  // Fall back to direct planning if the escape point is invalid or either sub-plan fails.
  bool escape_planned = false;
  if (escape_distance_ > 0.0) {
    double escape_x, escape_y;
    double dummy_x, dummy_y;
    bool goal_near_obstacle = computeEscapePoint(goal, dummy_x, dummy_y);
    if (!goal_near_obstacle && computeEscapePoint(start, escape_x, escape_y)) {
      unsigned int mx_esc, my_esc;
      bool escape_free =
        planner_->costmap_->worldToMap(escape_x, escape_y, mx_esc, my_esc) &&
        planner_->costmap_->getCost(mx_esc, my_esc) < nav2_costmap_2d::LETHAL_OBSTACLE;

      if (escape_free) {
        geometry_msgs::msg::PoseStamped escape_pose;
        escape_pose.header = start.header;
        escape_pose.pose.position.x = escape_x;
        escape_pose.pose.position.y = escape_y;
        escape_pose.pose.position.z = 0.0;
        // Face toward goal from the escape point
        tf2::Quaternion q_esc;
        q_esc.setRPY(
          0.0, 0.0,
          std::atan2(
            goal.pose.position.y - escape_y,
            goal.pose.position.x - escape_x));
        escape_pose.pose.orientation = tf2::toMsg(q_esc);

        try {
          nav_msgs::msg::Path escape_path;
          planner_->setStartAndGoal(start, escape_pose);
          getPlan(escape_path);
          for (auto & pose : escape_path.poses) {
            pose.pose.orientation = start.pose.orientation;
          }

          planner_->setStartAndGoal(escape_pose, goal);
          getPlan(global_path);
          if (!global_path.poses.empty()) {
            global_path.poses.front().pose.orientation = start.pose.orientation;
          }

          // Prepend escape segment
          escape_path.poses.insert(
            escape_path.poses.end(),
            global_path.poses.begin(),
            global_path.poses.end());
          global_path = escape_path;
          escape_planned = true;
          RCLCPP_DEBUG(
            logger_, "Escape path planned via (%.2f, %.2f)", escape_x, escape_y);
        } catch (const std::exception & e) {
          RCLCPP_WARN(
            logger_,
            "Escape path planning failed (%s), falling back to direct planning.", e.what());
          global_path.poses.clear();
        }
      }
    }
  }

  if (!escape_planned) {
    planner_->setStartAndGoal(start, goal);
    RCLCPP_DEBUG(
      logger_, "Got the src and dst... (%i, %i) && (%i, %i)",
      planner_->src_.x, planner_->src_.y, planner_->dst_.x, planner_->dst_.y);
    getPlan(global_path);
  }

  size_t plan_size = global_path.poses.size();

  // When start and goal are close enough and orientation difference is within orientation_delta,
  // assign the average of start and goal yaw to every pose in the path.
  double dist_sg = std::hypot(
    goal.pose.position.x - start.pose.position.x,
    goal.pose.position.y - start.pose.position.y);
  tf2::Quaternion q_start, q_goal;
  tf2::fromMsg(start.pose.orientation, q_start);
  tf2::fromMsg(goal.pose.orientation, q_goal);
  double yaw_start = tf2::getYaw(q_start);
  double yaw_goal = tf2::getYaw(q_goal);
  double angle_diff = std::abs(
    std::atan2(std::sin(yaw_goal - yaw_start), std::cos(yaw_goal - yaw_start)));

  if (dist_sg <= proximity_threshold_ && angle_diff <= orientation_delta_) {
    double avg_yaw = std::atan2(
      std::sin(yaw_start) + std::sin(yaw_goal),
      std::cos(yaw_start) + std::cos(yaw_goal));
    tf2::Quaternion q_avg;
    q_avg.setRPY(0.0, 0.0, avg_yaw);
    auto avg_orientation = tf2::toMsg(q_avg);
    for (auto & pose : global_path.poses) {
      pose.pose.orientation = avg_orientation;
    }
  } else {
    if (plan_size > 0) {
      global_path.poses.back().pose.orientation = goal.pose.orientation;
    }

    // If use_final_approach_orientation=true, interpolate the last pose orientation from the
    // previous pose to set the orientation to the 'final approach' orientation of the robot so
    // it does not rotate.
    // And deal with corner case of plan of length 1
    if (use_final_approach_orientation_) {
      if (plan_size == 1) {
        global_path.poses.back().pose.orientation = start.pose.orientation;
      } else if (plan_size > 1) {
        double dx, dy, theta;
        auto last_pose = global_path.poses.back().pose.position;
        auto approach_pose = global_path.poses[plan_size - 2].pose.position;
        dx = last_pose.x - approach_pose.x;
        dy = last_pose.y - approach_pose.y;
        theta = atan2(dy, dx);
        global_path.poses.back().pose.orientation =
          nav2_util::geometry_utils::orientationAroundZAxis(theta);
      }
    }
  }

  // Prepend the robot's actual start pose so the path always begins at the robot position.
  // linearInterpolation skips the first raw waypoint (inner loop starts at k=1).
  if (!global_path.poses.empty()) {
    geometry_msgs::msg::PoseStamped start_pose;
    start_pose.header = global_path.header;
    start_pose.pose = start.pose;
    global_path.poses.insert(global_path.poses.begin(), start_pose);
  }

  auto stop_time = std::chrono::steady_clock::now();
  auto dur = std::chrono::duration_cast<std::chrono::microseconds>(stop_time - start_time);
  RCLCPP_DEBUG(logger_, "the time taken is : %i", static_cast<int>(dur.count()));
  RCLCPP_DEBUG(logger_, "the nodes_opened are:  %i", planner_->nodes_opened);
  return global_path;
}

void ThetaStarOrientedPlanner::getPlan(nav_msgs::msg::Path & global_path)
{
  std::vector<coordsW> path;
  if (planner_->isUnsafeToPlan()) {
    global_path.poses.clear();
    throw nav2_core::PlannerException("Either of the start or goal pose are an obstacle! ");
  } else if (planner_->generatePath(path)) {
    global_path = linearInterpolation(path, planner_->costmap_->getResolution());
  } else {
    global_path.poses.clear();
    throw nav2_core::NoValidPathCouldBeFound("Could not generate path between the given poses");
  }
  global_path.header.stamp = clock_->now();
  global_path.header.frame_id = global_frame_;
}

nav_msgs::msg::Path ThetaStarOrientedPlanner::linearInterpolation(
  const std::vector<coordsW> & raw_path,
  const double & dist_bw_points)
{
  nav_msgs::msg::Path pa;

  geometry_msgs::msg::PoseStamped p1;
  for (unsigned int j = 0; j < raw_path.size() - 1; j++) {
    coordsW pt1 = raw_path[j];
    p1.pose.position.x = pt1.x;
    p1.pose.position.y = pt1.y;
    //pa.poses.push_back(p1);

    coordsW pt2 = raw_path[j + 1];
    double theta1 = atan2(pt2.y - pt1.y, pt2.x - pt1.x);
    tf2::Quaternion q1;
    q1.setRPY(0.0, 0.0, theta1);
    p1.pose.orientation = tf2::toMsg(q1);
    double distance = std::hypot(pt2.x - pt1.x, pt2.y - pt1.y);
    int loops = static_cast<int>(distance / dist_bw_points);
    double sin_alpha = (pt2.y - pt1.y) / distance;
    double cos_alpha = (pt2.x - pt1.x) / distance;
    for (int k = 1; k < loops; k++) {
      p1.pose.position.x = pt1.x + k * dist_bw_points * cos_alpha;
      p1.pose.position.y = pt1.y + k * dist_bw_points * sin_alpha;
      double theta = atan2(pt2.y - pt1.y, pt2.x - pt1.x);
      tf2::Quaternion q;
      q.setRPY(0.0, 0.0, theta);
      p1.pose.orientation = tf2::toMsg(q);
      pa.poses.push_back(p1);
    }
  }

  return pa;
}

rcl_interfaces::msg::SetParametersResult
ThetaStarOrientedPlanner::dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  for (auto parameter : parameters) {
    const auto & type = parameter.get_type();
    const auto & name = parameter.get_name();

    if (type == ParameterType::PARAMETER_INTEGER) {
      if (name == name_ + ".how_many_corners") {
        planner_->how_many_corners_ = parameter.as_int();
      }
    } else if (type == ParameterType::PARAMETER_DOUBLE) {
      if (name == name_ + ".w_euc_cost") {
        planner_->w_euc_cost_ = parameter.as_double();
      } else if (name == name_ + ".w_traversal_cost") {
        planner_->w_traversal_cost_ = parameter.as_double();
      } else if (name == name_ + ".proximity_threshold") {
        proximity_threshold_ = parameter.as_double();
      } else if (name == name_ + ".orientation_delta") {
        orientation_delta_ = parameter.as_double();
      } else if (name == name_ + ".escape_distance") {
        escape_distance_ = parameter.as_double();
      } else if (name == name_ + ".escape_lateral_range") {
        escape_lateral_range_ = parameter.as_double();
      }
    } else if (type == ParameterType::PARAMETER_BOOL) {
      if (name == name_ + ".use_final_approach_orientation") {
        use_final_approach_orientation_ = parameter.as_bool();
      } else if (name == name_ + ".allow_unknown") {
        planner_->allow_unknown_ = parameter.as_bool();
      }
    }
  }

  result.successful = true;
  return result;
}

bool ThetaStarOrientedPlanner::computeEscapePoint(
  const geometry_msgs::msg::PoseStamped & start,
  double & escape_x, double & escape_y)
{
  unsigned int mx_start, my_start;
  if (!planner_->costmap_->worldToMap(
      start.pose.position.x, start.pose.position.y, mx_start, my_start))
  {
    return false;
  }

  tf2::Quaternion q;
  tf2::fromMsg(start.pose.orientation, q);
  const double yaw = tf2::getYaw(q);
  const double cos_yaw = std::cos(yaw);
  const double sin_yaw = std::sin(yaw);

  const double resolution = planner_->costmap_->getResolution();
  const int radius_cells = static_cast<int>(std::ceil(escape_distance_ / resolution));
  const int size_x = static_cast<int>(planner_->costmap_->getSizeInCellsX());
  const int size_y = static_cast<int>(planner_->costmap_->getSizeInCellsY());

  double nearest_obs_x = 0.0, nearest_obs_y = 0.0;
  double nearest_dist = std::numeric_limits<double>::max();
  bool found_obstacle = false;

  for (int dy = -radius_cells; dy <= radius_cells; dy++) {
    for (int dx = -radius_cells; dx <= radius_cells; dx++) {
      const double world_dx = dx * resolution;
      const double world_dy = dy * resolution;

      // Project offset into robot frame: forward (local_x) and lateral (local_y)
      const double local_x =  world_dx * cos_yaw + world_dy * sin_yaw;
      const double local_y = -world_dx * sin_yaw + world_dy * cos_yaw;

      // Only obstacles strictly ahead and within the lateral corridor
      if (local_x <= 0.0 || std::abs(local_y) > escape_lateral_range_) {
        continue;
      }

      const double dist = std::hypot(world_dx, world_dy);
      if (dist > escape_distance_) {
        continue;
      }

      const int nx = static_cast<int>(mx_start) + dx;
      const int ny = static_cast<int>(my_start) + dy;
      if (nx < 0 || ny < 0 || nx >= size_x || ny >= size_y) {
        continue;
      }

      if (planner_->costmap_->getCost(nx, ny) >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
        if (dist < nearest_dist) {
          nearest_dist = dist;
          nearest_obs_x = start.pose.position.x + world_dx;
          nearest_obs_y = start.pose.position.y + world_dy;
          found_obstacle = true;
        }
      }
    }
  }

  if (!found_obstacle) {
    return false;
  }

  // Direction from the nearest obstacle back toward the robot
  double dir_x = start.pose.position.x - nearest_obs_x;
  double dir_y = start.pose.position.y - nearest_obs_y;
  const double norm = std::hypot(dir_x, dir_y);
  if (norm < 1e-6) {
    dir_x = -cos_yaw;
    dir_y = -sin_yaw;
  } else {
    dir_x /= norm;
    dir_y /= norm;
  }

  // Place escape point escape_distance_ behind the obstacle (toward the robot)
  escape_x = nearest_obs_x + dir_x * escape_distance_;
  escape_y = nearest_obs_y + dir_y * escape_distance_;
  return true;
}

}  // namespace nav2_theta_star_oriented_planner

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_theta_star_oriented_planner::ThetaStarOrientedPlanner, nav2_core::GlobalPlanner)
