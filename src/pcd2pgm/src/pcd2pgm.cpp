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
// MODIFIED: added live_topic mode - see pcd2pgm.hpp for the summary.

#include "pcd2pgm/pcd2pgm.hpp"

#include <chrono>

#include "pcl/common/transforms.h"
#include "pcl/filters/radius_outlier_removal.h"
#include "pcl/io/pcd_io.h"
#include "pcl_conversions/pcl_conversions.h"

namespace pcd2pgm
{
Pcd2PgmNode::Pcd2PgmNode(const rclcpp::NodeOptions & options) : Node("pcd2pgm", options)
{
  declareParameters();
  getParameters();

  rclcpp::QoS map_qos(10);
  map_qos.transient_local();
  map_qos.reliable();
  map_qos.keep_last(1);

  pcd_cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  map_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(map_topic_name_, map_qos);
  pcd_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("pcd_cloud", 10);

  if (!live_topic_.empty()) {
    // Live mode: subscribe instead of loading a file once. GLIM's /glim_ros/map is
    // already the full accumulated, loop-closure-corrected cloud in the `map` frame
    // (not a sliding window), so re-rasterizing it from scratch on every message is
    // correct - nothing gets lost between updates.
    RCLCPP_INFO(
      get_logger(), "Live mode: subscribing to '%s', fixed grid origin=(%.2f, %.2f) size=%.1fx%.1fm",
      live_topic_.c_str(), fixed_origin_x_, fixed_origin_y_, fixed_width_m_, fixed_height_m_);

    live_cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      live_topic_, rclcpp::QoS(5).reliable(),
      std::bind(&Pcd2PgmNode::liveCloudCallback, this, std::placeholders::_1));
  } else {
    // Original file-mode behaviour, unchanged.
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_file_, *pcd_cloud_) == -1) {
      RCLCPP_ERROR(get_logger(), "Couldn't read file: %s", pcd_file_.c_str());
      return;
    }

    RCLCPP_INFO(get_logger(), "Initial point cloud size: %lu", pcd_cloud_->points.size());

    applyTransform();

    passThroughFilter(thre_z_min_, thre_z_max_, flag_pass_through_);
    radiusOutlierFilter(cloud_after_pass_through_, thre_radius_, thres_point_count_);
    setMapTopicMsgDynamicBounds(cloud_after_radius_, map_topic_msg_);
  }

  // File mode only: republish heartbeat. In live mode the map publisher is
  // transient_local, so a late subscriber (e.g. Nav2 restarted) already gets the
  // last grid; re-sending a 1000x1000 grid + the filtered cloud every second just
  // makes Nav2 re-copy the whole static layer for nothing.
  if (live_topic_.empty()) {
    timer_ =
      create_wall_timer(std::chrono::seconds(1), std::bind(&Pcd2PgmNode::publishCallback, this));
  }
}

void Pcd2PgmNode::liveCloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
  const auto t_start = std::chrono::steady_clock::now();
  auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  pcl::fromROSMsg(*msg, *cloud);
  pcd_cloud_ = cloud;

  RCLCPP_INFO(get_logger(), "Live cloud received: %lu points", pcd_cloud_->points.size());

  // An empty cloud here (GLIM restarting, or a wrong z band filtering everything out) would
  // rasterize to an all-free grid and silently wipe every obstacle from Nav2's static layer.
  // Keep publishing the previous /map instead.
  passThroughFilter(thre_z_min_, thre_z_max_, flag_pass_through_);
  if (cloud_after_pass_through_->points.empty()) {
    RCLCPP_WARN(get_logger(), "Live cloud (%lu points) is empty after PassThrough - keeping the previous /map",
      pcd_cloud_->points.size());
    return;
  }
  radiusOutlierFilter(cloud_after_pass_through_, thre_radius_, thres_point_count_);
  if (cloud_after_radius_->points.empty()) {
    RCLCPP_WARN(get_logger(), "Live cloud is empty after RadiusOutlier - keeping the previous /map");
    return;
  }
  setMapTopicMsgFixedBounds(cloud_after_radius_, map_topic_msg_);

  // Publish immediately rather than waiting for the next 1s heartbeat tick -
  // GLIM's own updates are already infrequent (~10s), no reason to add latency
  // on top of that.
  publishCallback();

  // GLIM's map only grows, so this gets slower over a run. If it ever
  // approaches GLIM's ~10 s publish period the grid falls behind.
  const double ms = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - t_start).count();
  RCLCPP_INFO(get_logger(), "Live map update took %.0f ms", ms);
}

void Pcd2PgmNode::publishCallback()
{
  if (!cloud_after_radius_) {
    return;
  }
  sensor_msgs::msg::PointCloud2 output;
  pcl::toROSMsg(*cloud_after_radius_, output);
  output.header.frame_id = "map";
  pcd_publisher_->publish(output);
  map_publisher_->publish(map_topic_msg_);
}

void Pcd2PgmNode::declareParameters()
{
  declare_parameter("pcd_file", "");
  declare_parameter("thre_z_min", 0.5);
  declare_parameter("thre_z_max", 2.0);
  declare_parameter("flag_pass_through", false);
  declare_parameter("thre_radius", 0.5);
  declare_parameter("map_resolution", 0.05);
  declare_parameter("thres_point_count", 10);
  declare_parameter("map_topic_name", "map");
  declare_parameter(
    "odom_to_lidar_odom", std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0});  // 新增的参数

  // Live mode params.
  declare_parameter("live_topic", "");           // e.g. "/glim_ros/map" - empty = file mode
  declare_parameter("fixed_origin_x", -25.0);     // metres, in the `map` frame
  declare_parameter("fixed_origin_y", -25.0);
  declare_parameter("fixed_width_m", 50.0);       // size the grid to the WHOLE arena, with
  declare_parameter("fixed_height_m", 50.0);      // margin - not to what's explored so far
}

void Pcd2PgmNode::getParameters()
{
  get_parameter("pcd_file", pcd_file_);
  get_parameter("thre_z_min", thre_z_min_);
  get_parameter("thre_z_max", thre_z_max_);
  get_parameter("flag_pass_through", flag_pass_through_);
  get_parameter("thre_radius", thre_radius_);
  get_parameter("map_resolution", map_resolution_);
  get_parameter("thres_point_count", thres_point_count_);
  get_parameter("map_topic_name", map_topic_name_);
  get_parameter("odom_to_lidar_odom", odom_to_lidar_odom_);  // 获取新的参数

  get_parameter("live_topic", live_topic_);
  get_parameter("fixed_origin_x", fixed_origin_x_);
  get_parameter("fixed_origin_y", fixed_origin_y_);
  get_parameter("fixed_width_m", fixed_width_m_);
  get_parameter("fixed_height_m", fixed_height_m_);
}

void Pcd2PgmNode::passThroughFilter(double thre_low, double thre_high, bool flag_in)
{
  auto filtered_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  pcl::PassThrough<pcl::PointXYZ> passthrough;
  passthrough.setInputCloud(pcd_cloud_);
  passthrough.setFilterFieldName("z");
  passthrough.setFilterLimits(thre_low, thre_high);
  passthrough.setNegative(flag_in);
  passthrough.filter(*filtered_cloud);

  cloud_after_pass_through_ = filtered_cloud;
  RCLCPP_INFO(
    get_logger(), "After PassThrough filtering: %lu points",
    cloud_after_pass_through_->points.size());
}

void Pcd2PgmNode::radiusOutlierFilter(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & input_cloud, double radius, int thre_count)
{
  auto filtered_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  pcl::RadiusOutlierRemoval<pcl::PointXYZ> radius_outlier;
  radius_outlier.setInputCloud(input_cloud);
  radius_outlier.setRadiusSearch(radius);
  radius_outlier.setMinNeighborsInRadius(thre_count);
  radius_outlier.filter(*filtered_cloud);

  cloud_after_radius_ = filtered_cloud;
  RCLCPP_INFO(
    get_logger(), "After RadiusOutlier filtering: %lu points", cloud_after_radius_->points.size());
}

void Pcd2PgmNode::setMapTopicMsgDynamicBounds(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, nav_msgs::msg::OccupancyGrid & msg)
{
  msg.header.stamp = now();
  msg.header.frame_id = "map";

  msg.info.map_load_time = now();
  msg.info.resolution = map_resolution_;

  double x_min = std::numeric_limits<double>::max();
  double x_max = std::numeric_limits<double>::lowest();
  double y_min = std::numeric_limits<double>::max();
  double y_max = std::numeric_limits<double>::lowest();

  if (cloud->points.empty()) {
    RCLCPP_WARN(get_logger(), "Point cloud is empty!");
    return;
  }

  for (const auto & point : cloud->points) {
    x_min = std::min(x_min, static_cast<double>(point.x));
    x_max = std::max(x_max, static_cast<double>(point.x));
    y_min = std::min(y_min, static_cast<double>(point.y));
    y_max = std::max(y_max, static_cast<double>(point.y));
  }

  msg.info.origin.position.x = x_min;
  msg.info.origin.position.y = y_min;
  msg.info.origin.position.z = 0.0;
  msg.info.origin.orientation.x = 0.0;
  msg.info.origin.orientation.y = 0.0;
  msg.info.origin.orientation.z = 0.0;
  msg.info.origin.orientation.w = 1.0;

  msg.info.width = std::ceil((x_max - x_min) / map_resolution_);
  msg.info.height = std::ceil((y_max - y_min) / map_resolution_);
  msg.data.assign(msg.info.width * msg.info.height, 0);

  for (const auto & point : cloud->points) {
    int i = std::floor((point.x - x_min) / map_resolution_);
    int j = std::floor((point.y - y_min) / map_resolution_);

    if (i >= 0 && i < msg.info.width && j >= 0 && j < msg.info.height) {
      msg.data[i + j * msg.info.width] = 100;
    }
  }

  RCLCPP_INFO(get_logger(), "Map data size: %lu", msg.data.size());
}

void Pcd2PgmNode::setMapTopicMsgFixedBounds(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, nav_msgs::msg::OccupancyGrid & msg)
{
  msg.header.stamp = now();
  msg.header.frame_id = "map";

  msg.info.map_load_time = now();
  msg.info.resolution = map_resolution_;

  // Fixed every call - this is what lets Nav2's static layer do a cheap in-place
  // refresh instead of a full resize/reinit on every update.
  msg.info.origin.position.x = fixed_origin_x_;
  msg.info.origin.position.y = fixed_origin_y_;
  msg.info.origin.position.z = 0.0;
  msg.info.origin.orientation.x = 0.0;
  msg.info.origin.orientation.y = 0.0;
  msg.info.origin.orientation.z = 0.0;
  msg.info.origin.orientation.w = 1.0;

  msg.info.width = std::ceil(fixed_width_m_ / map_resolution_);
  msg.info.height = std::ceil(fixed_height_m_ / map_resolution_);
  msg.data.assign(msg.info.width * msg.info.height, 0);

  int dropped = 0;
  for (const auto & point : cloud->points) {
    int i = std::floor((point.x - fixed_origin_x_) / map_resolution_);
    int j = std::floor((point.y - fixed_origin_y_) / map_resolution_);

    if (i >= 0 && i < static_cast<int>(msg.info.width) && j >= 0 &&
      j < static_cast<int>(msg.info.height))
    {
      msg.data[i + j * msg.info.width] = 100;
    } else {
      ++dropped;
    }
  }

  if (dropped > 0) {
    RCLCPP_WARN(
      get_logger(),
      "%d points fell outside the fixed grid bounds (origin=(%.2f, %.2f) size=%.1fx%.1fm) - "
      "widen fixed_width_m/fixed_height_m if this keeps happening",
      dropped, fixed_origin_x_, fixed_origin_y_, fixed_width_m_, fixed_height_m_);
  }
}

void Pcd2PgmNode::applyTransform()
{
  Eigen::Affine3f transform = Eigen::Affine3f::Identity();

  transform.translation() << odom_to_lidar_odom_[0], odom_to_lidar_odom_[1], odom_to_lidar_odom_[2];
  transform.rotate(Eigen::AngleAxisf(odom_to_lidar_odom_[3], Eigen::Vector3f::UnitX()));
  transform.rotate(Eigen::AngleAxisf(odom_to_lidar_odom_[4], Eigen::Vector3f::UnitY()));
  transform.rotate(Eigen::AngleAxisf(odom_to_lidar_odom_[5], Eigen::Vector3f::UnitZ()));

  pcl::transformPointCloud(*pcd_cloud_, *pcd_cloud_, transform.inverse());
}

}  // namespace pcd2pgm

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(pcd2pgm::Pcd2PgmNode)
