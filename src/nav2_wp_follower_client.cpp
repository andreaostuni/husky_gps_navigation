// Copyright (c) 2020 Fetullah Atas
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

#include "husky_gps_navigation/nav2_wp_follower_client.hpp"
#include <string>
#include <vector>
#include <functional>

namespace husky_gps_navigation
{
GPSWayPointFollowerClient::GPSWayPointFollowerClient() : Node("GPSWaypointFollowerClient"), goal_done_(false)
{
  gps_waypoint_follower_action_client_ = rclcpp_action::create_client<ClientT>(this, "follow_gps_waypoints");
  this->timer_ = this->create_wall_timer(std::chrono::milliseconds(500),
                                         std::bind(&GPSWayPointFollowerClient::startWaypointFollowing, this));
  // number of poses that robot will go throug, specified in yaml file
  // this->declare_parameter("paths", std::vector<std::string>({"0"}));
  std::string chosen_path;
  this->declare_parameter("chosen_path", "path0");
  chosen_path = this->get_parameter("chosen_path").as_string();

  this->declare_parameter(chosen_path + ".waypoints", std::vector<std::string>({ "0" }));
  path_from_yaml_ = loadGPSWaypointsFromYAML(chosen_path);

  RCLCPP_INFO(this->get_logger(),
              "Loaded %i paths from YAML, gonna pass them to "
              "FollowGPSWaypoint...",
              static_cast<int>(path_from_yaml_.size()));
  RCLCPP_INFO(this->get_logger(), "Created an Instance of GPSWayPointFollowerClient");
}

GPSWayPointFollowerClient::~GPSWayPointFollowerClient()
{
  RCLCPP_INFO(this->get_logger(), "Destroyed an Instance of GPSWayPointFollowerClient");
}

void GPSWayPointFollowerClient::startWaypointFollowing()
{
  this->timer_->cancel();
  this->goal_done_ = false;

  if (!this->gps_waypoint_follower_action_client_)
  {
    RCLCPP_ERROR(this->get_logger(), "Action client not initialized");
  }
  // !this->client_ptr_->wait_for_action_server()
  // auto is_action_server_ready =
  //     gps_waypoint_follower_action_client_->wait_for_action_server();
  if (!this->gps_waypoint_follower_action_client_->wait_for_action_server())
  {
    RCLCPP_ERROR(this->get_logger(),
                 "FollowGPSWaypointsPlugin action server is not available."
                 " Make sure an instance of GPSWaypointFollower is up and running");
    this->goal_done_ = true;
    return;
  }
  RCLCPP_INFO(this->get_logger(), "Sending goal");

  gps_waypoint_follower_goal_ = ClientT::Goal();
  // Send the goal poses
  gps_waypoint_follower_goal_.gps_poses = path_from_yaml_;

  RCLCPP_INFO(this->get_logger(), "Sending %zu poses:", gps_waypoint_follower_goal_.gps_poses.size());

  for (auto pose : gps_waypoint_follower_goal_.gps_poses)
  {
    RCLCPP_INFO(this->get_logger(), "\t(%lf, %lf)", pose.position.latitude, pose.position.longitude);
  }

  auto goal_options = rclcpp_action::Client<ClientT>::SendGoalOptions();

  goal_options.goal_response_callback =
      std::bind(&GPSWayPointFollowerClient::goalResponseCallback, this, std::placeholders::_1);

  goal_options.feedback_callback =
      std::bind(&GPSWayPointFollowerClient::feedbackCallback, this, std::placeholders::_1, std::placeholders::_2);

  goal_options.result_callback = std::bind(&GPSWayPointFollowerClient::resultCallback, this, std::placeholders::_1);

  auto future_goal_handle =
      gps_waypoint_follower_action_client_->async_send_goal(gps_waypoint_follower_goal_, goal_options);
}

// load paths from the yaml file
// std::vector<nav2_msgs::msg::GPSWPathPlugin>
// GPSWayPointFollowerClient::loadGPSWPaths() {
//   std::vector<std::string> paths_vector =
//       this->get_parameter("paths").as_string_array();
//   std::vector<nav2_msgs::msg::GPSWPathPlugin> paths_and_plugins;

//   for (auto curr_path : paths_vector) {
//     this->declare_parameter(curr_path + ".waypoints",
//                             std::vector<std::string>({"0"}));
//     this->declare_parameter(curr_path + ".task", std::string{"0"});
//     std::string task_plugin = get_parameter(curr_path + ".task").as_string();
//     RCLCPP_INFO(this->get_logger(), "task loaded");
//     std::vector<nav2_msgs::msg::OrientedNavSatFix> gps_poses =
//         loadGPSWaypointsFromYAML(curr_path);
//     RCLCPP_INFO(this->get_logger(), "waypoints loaded");
//     GPSWmsg path_with_plugin;
//     path_with_plugin.plugin_id = task_plugin;
//     path_with_plugin.gps_poses = gps_poses;
//     paths_and_plugins.push_back(path_with_plugin);
//   }
//   return paths_and_plugins;
// }

std::vector<geographic_msgs::msg::GeoPose>
GPSWayPointFollowerClient::loadGPSWaypointsFromYAML(const std::string current_path)
{
  std::vector<std::string> waypoints_vector = this->get_parameter(current_path + "." + "waypoints").as_string_array();
  std::vector<geographic_msgs::msg::GeoPose> gps_waypoint_msg_vector;
  for (auto&& curr_waypoint : waypoints_vector)
  {
    try
    {
      this->declare_parameter(current_path + "." + curr_waypoint, std::vector<double>({ 0 }));
      std::vector<double> gps_waypoint_vector =
          this->get_parameter(current_path + "." + curr_waypoint).as_double_array();
      RCLCPP_INFO(this->get_logger(), "loading waypoint");
      // throw exception if incorrect format was detected from yaml file reading
      if (gps_waypoint_vector.size() != 4)
      {
        RCLCPP_FATAL(this->get_logger(),
                     "GPS waypoint that was loaded from YAML file seems to "
                     "have incorrect"
                     " form, the right format is; wpN: [Lat, Long, Alt, yaw(in "
                     "radians)] with double types");
        throw rclcpp::exceptions::InvalidParametersException(
            "[ERROR] See above error"
            " the right format is; wpN: [Lat, Long, Alt, yaw(in radians)] with "
            "double types"
            "E.g gps_waypoint0; [0.0, 0.0, 0.0, 0.0], please chechk YAML file");
      }
      // construct the gps waypoint and push them to their vector
      // lat, long , alt
      geographic_msgs::msg::GeoPose gps_pose;
      gps_pose.position.latitude = gps_waypoint_vector.at(0);
      gps_pose.position.longitude = gps_waypoint_vector.at(1);
      gps_pose.position.altitude = gps_waypoint_vector.at(2);
      tf2::Quaternion gps_pose_quat;
      gps_pose_quat.setRPY(0.0, 0.0, gps_waypoint_vector.at(3));
      gps_pose.orientation = tf2::toMsg(gps_pose_quat);
      gps_waypoint_msg_vector.push_back(gps_pose);
    }
    catch (const std::exception& e)
    {
      std::cerr << e.what() << '\n';
    }
  }
  for (auto pose : gps_waypoint_msg_vector)
  {
    RCLCPP_INFO(this->get_logger(), "\t(%lf, %lf)", pose.position.latitude, pose.position.longitude);
  }
  // return the read pair of this gps waypoint to it's caller
  return gps_waypoint_msg_vector;
}

void GPSWayPointFollowerClient::goalResponseCallback(ClientTGoalHandle::SharedPtr goal_handle)
{
  // auto goal_handle = future.get();
  if (!goal_handle)
  {
    RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
  }
  else
  {
    RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
  }
}

void GPSWayPointFollowerClient::feedbackCallback(ClientTGoalHandle::SharedPtr,
                                                 const std::shared_ptr<const ClientT::Feedback> feedback)
{
  RCLCPP_INFO(this->get_logger(), "current waypoint: %i", feedback->current_waypoint);
}

void GPSWayPointFollowerClient::resultCallback(const ClientTGoalHandle::WrappedResult& result)
{
  this->goal_done_ = true;
  switch (result.code)
  {
    case rclcpp_action::ResultCode::SUCCEEDED:
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
      return;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
      return;
    default:
      RCLCPP_ERROR(this->get_logger(), "Unknown result code");
      return;
  }

  RCLCPP_INFO(this->get_logger(), "Result received");
  for (auto number : result.result->missed_waypoints)
  {
    RCLCPP_INFO(this->get_logger(), "Missed Waypoint %i", number);
  }
}

bool GPSWayPointFollowerClient::is_goal_done() const
{
  return this->goal_done_;
}

}  // namespace husky_gps_navigation

/**
 * @brief Entry point for Way Point following demo Node
 *
 * @param argc
 * @param argv
 * @return int
 */
int main(int argc, char const* argv[])
{
  rclcpp::init(argc, argv);
  auto gps_waypoint_follower_client_node = std::make_shared<husky_gps_navigation::GPSWayPointFollowerClient>();

  while (!gps_waypoint_follower_client_node->is_goal_done())
  {
    rclcpp::spin_some(gps_waypoint_follower_client_node);
  }
  rclcpp::shutdown();
  return 0;
}
