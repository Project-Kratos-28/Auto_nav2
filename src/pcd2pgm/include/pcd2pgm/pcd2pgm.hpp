// Copyright 2025 Lihan Chen
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
// MODIFIED: added an optional live_topic mode. When set, the node subscribes to
// that topic (e.g. GLIM's /glim_ros/map) instead of loading pcd_file once, and
// re-filters/re-rasterizes every time a new cloud arrives instead of only at
// startup. The grid's origin/width/height are then FIXED (from params), not
// recomputed from each cloud's bounding box, so Nav2's static costmap layer sees
// identical dimensions on every update and can do a cheap in-place refresh instead
// of a full resize. File mode (pcd_file set, live_topic empty) is unchanged.

#ifndef PCD2PGM__PCD2PGM_HPP_
#define PCD2PGM__PCD2PGM_HPP_

#include <memory>
#include <string>
#include <vector>

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "pcl/filters/passthrough.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace pcd2pgm
{
class Pcd2PgmNode : public rclcpp::Node
{
public:
  explicit Pcd2PgmNode(const rclcpp::NodeOptions & options);

private:
  void declareParameters();

  void getParameters();

  void passThroughFilter(double thre_low, double thre_high, bool flag_in);

  void radiusOutlierFilter(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & pcd_cloud0, double radius, int thre_count);

  // Original behaviour: grid origin/size computed from the cloud's bounding box.
  void setMapTopicMsgDynamicBounds(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, nav_msgs::msg::OccupancyGrid & msg);

  // Live-mode behaviour: grid origin/size are fixed (from params), cells outside
  // those bounds are dropped rather than growing the grid.
  void setMapTopicMsgFixedBounds(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, nav_msgs::msg::OccupancyGrid & msg);

  void publishCallback();

  void applyTransform();

  // Re-runs the filter chain against a freshly received live cloud.
  void liveCloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);

  float thre_z_min_;
  float thre_z_max_;
  float thre_radius_;
  bool flag_pass_through_;
  float map_resolution_;
  int thres_point_count_;
  std::string pcd_file_;
  std::string map_topic_name_;
  std::vector<double> odom_to_lidar_odom_;

  // New: live-topic mode params.
  std::string live_topic_;
  double fixed_origin_x_;
  double fixed_origin_y_;
  double fixed_width_m_;
  double fixed_height_m_;

  std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> pcd_cloud_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_after_pass_through_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_after_radius_;
  nav_msgs::msg::OccupancyGrid map_topic_msg_;

  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pcd_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr live_cloud_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace pcd2pgm

#endif  // PCD2PGM__PCD2PGM_HPP_
