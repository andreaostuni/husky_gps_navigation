// Copyright (c) 2020 Fetullah Atas, Norwegian University of Life Sciences
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

#include "husky_gps_navigation/gps_waypoint_collector.hpp"
#include <fstream>
#include <iostream>
#include <memory>
#include <ostream>
#include <string>
#include <utility>

namespace husky_gps_navigation {

GPSWaypointCollector::GPSWaypointCollector()
    : Node("gps_waypoint_collector_rclcpp_node"),
      is_first_msg_recieved_(false) {
  declare_parameter("frequency", 10);
  declare_parameter("yaml_file_out",
                    "collected_waypoints.yaml");
//   declare_parameter("plugin_id", "switch_localization_at_waypoint");
  declare_parameter("path_length", 5);

  get_parameter("frequency", frequency_);
  get_parameter("yaml_file_out", yaml_file_out_);
//   get_parameter("plugin_id", plugin_id_);
  path_length_ = get_parameter("path_length").as_int();

  wp_count_ = 0;

  timer_ = this->create_wall_timer(
      std::chrono::seconds(60 / frequency_),
      std::bind(&GPSWaypointCollector::timerCallback, this));

  navsat_fix_subscriber_.subscribe(this, "/gps", rmw_qos_profile_sensor_data);
  imu_subscriber_.subscribe(this, "/imu", rmw_qos_profile_sensor_data);

  geopose_publisher_ =
      this->create_publisher<geographic_msgs::msg::GeoPose>(
          "collected_gps_waypoints", rclcpp::SystemDefaultsQoS());

  sensor_data_approx_time_syncher_.reset(
      new SensorDataApprxTimeSyncer(SensorDataApprxTimeSyncPolicy(10),
                                    navsat_fix_subscriber_, imu_subscriber_));

  sensor_data_approx_time_syncher_->registerCallback(
      std::bind(&GPSWaypointCollector::sensorDataCallback, this,
                std::placeholders::_1, std::placeholders::_2));
}

GPSWaypointCollector::~GPSWaypointCollector() {
  if (!current_waypoints_vector_.empty()) {
    collected_paths_vector_.push_back(std::move(current_waypoints_vector_));
  }
  dumpCollectedWaypoints();
}

void GPSWaypointCollector::timerCallback() {
  RCLCPP_INFO_ONCE(this->get_logger(),
                   "Entering to timer callback, this is periodicly called");
  if (is_first_msg_recieved_) {
    std::lock_guard<std::mutex> guard(global_mutex_);
    tf2::Quaternion q(
        reusable_imu_msg_.orientation.x, reusable_imu_msg_.orientation.y,
        reusable_imu_msg_.orientation.z, reusable_imu_msg_.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    RCLCPP_INFO(this->get_logger(),
                "curr_gps_waypoint: [%.8f, %.8f, %.8f, %.8f]",
                reusable_navsat_msg_.latitude, reusable_navsat_msg_.longitude,
                reusable_navsat_msg_.altitude, yaw);

    std::vector<double> curr_waypoint_data = {
        reusable_navsat_msg_.latitude, reusable_navsat_msg_.longitude,
        reusable_navsat_msg_.altitude, yaw};

    current_waypoints_vector_.push_back(curr_waypoint_data);
    geographic_msgs::msg::GeoPose curr_gps_waypoint;
    curr_gps_waypoint.position = navsat_to_geopoint(reusable_navsat_msg_);
    curr_gps_waypoint.orientation = reusable_imu_msg_.orientation;
    geopose_publisher_->publish(curr_gps_waypoint);
    wp_count_++;

    if ((wp_count_ % path_length_) == 0) {
      collected_paths_vector_.push_back(std::move(current_waypoints_vector_));
    }
  }
}

void GPSWaypointCollector::sensorDataCallback(
    const sensor_msgs::msg::NavSatFix::ConstSharedPtr &gps,
    const sensor_msgs::msg::Imu::ConstSharedPtr &imu) {
  std::lock_guard<std::mutex> guard(global_mutex_);
  reusable_navsat_msg_ = *gps;
  reusable_imu_msg_ = *imu;
  is_first_msg_recieved_ = true;
}

void GPSWaypointCollector::dumpCollectedWaypoints() {
  YAML::Emitter waypoints;
  waypoints << YAML::BeginMap;
  waypoints << YAML::Key << "gps_waypoint_follower_demo";
  waypoints << YAML::BeginMap;
  waypoints << YAML::Key << "ros__parameters";

  waypoints << YAML::BeginMap;
  waypoints << YAML::Key << "paths";
  waypoints << YAML::Value << YAML::Flow << YAML::BeginSeq;
  for (size_t i = 0; i < collected_paths_vector_.size(); i++) {
    std::string path_name = "path" + std::to_string(i);
    waypoints << path_name;
  }
  waypoints << YAML::EndSeq;

  for (size_t i = 0; i < collected_paths_vector_.size(); i++) {
    std::string path_name = "path" + std::to_string(i);

    waypoints << YAML::Key << path_name;
    waypoints << YAML::BeginMap;
    // waypoints << YAML::Key << "task";
    // waypoints << YAML::Value << plugin_id_;

    waypoints << YAML::Key << "waypoints";
    waypoints << YAML::Value << YAML::Flow << YAML::BeginSeq;
    for (size_t j = 0; j < collected_paths_vector_[i].size(); j++) {
      std::string wp_name = "wp" + std::to_string(j);
      waypoints << wp_name;
    }
    waypoints << YAML::EndSeq;
    for (size_t j = 0; j < collected_paths_vector_[i].size(); j++) {
      std::string wp_name = "wp" + std::to_string(j);
      waypoints << YAML::Key << wp_name << YAML::Value;
      waypoints << YAML::Flow << YAML::BeginSeq
                << collected_paths_vector_[i][j][0]
                << collected_paths_vector_[i][j][1]
                << collected_paths_vector_[i][j][2]
                << collected_paths_vector_[i][j][3] << YAML::EndSeq;
    }
    waypoints << YAML::EndMap;
  }

  waypoints << YAML::EndMap;
  waypoints << YAML::EndMap;
  waypoints << YAML::EndMap;
  std::ofstream fout(yaml_file_out_, std::ofstream::out);
  fout << waypoints.c_str();
  fout.close();
}
geographic_msgs::msg::GeoPoint GPSWaypointCollector::navsat_to_geopoint(sensor_msgs::msg::NavSatFix &point){
    geographic_msgs::msg::GeoPoint geopoint;
    geopoint.altitude = point.altitude;
    geopoint.latitude = point.latitude;
    geopoint.longitude = point.longitude;
    return geopoint;
}

} // namespace husky_gps_navigation

int main(int argc, char const *argv[]) {
  rclcpp::init(argc, argv);
  auto node =
      std::make_shared<husky_gps_navigation::GPSWaypointCollector>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
