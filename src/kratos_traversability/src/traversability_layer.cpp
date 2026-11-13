// Nav2 costmap layer: copies the traversability grid (nav_msgs/OccupancyGrid from
// traversability_mapper.py) into the costmap, keeping the higher of the two costs.
//
//   grid value -1      unknown: the costmap cell is left as it is
//   grid value 0..99   cost 0..252 (scaled)
//   grid value 100     LETHAL_OBSTACLE
//
// The grid may be in another frame than the costmap (odom grid, map costmap): every cell is
// transformed (x, y, yaw) with the latest TF. Put this layer before the inflation layer, so
// lethal cells are inflated.
//
// When a new grid arrives, only the square of update_radius around the rover is redrawn (the
// mapper re-rates only around the rover; elsewhere the costmap keeps what it has). Redrawing
// the whole 50 x 50 m global costmap and its inflation every second starved the planner.
#include <algorithm>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/exceptions.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"

namespace kratos_traversability
{

class TraversabilityLayer : public nav2_costmap_2d::Layer
{
public:
  void onInitialize() override
  {
    auto node = node_.lock();
    if (!node) {
      throw std::runtime_error("TraversabilityLayer: node expired");
    }
    declareParameter("enabled", rclcpp::ParameterValue(true));
    declareParameter("grid_topic", rclcpp::ParameterValue(std::string("/traversability/grid")));
    declareParameter("transform_tolerance", rclcpp::ParameterValue(0.5));
    declareParameter("update_radius", rclcpp::ParameterValue(15.0));
    // true: where the grid has a rating, it replaces what the layers before this one wrote
    // (a fixed height band such as nvblox's slice marks every ramp and hill as an obstacle; the
    // terrain rating knows better). Those layers still count where the terrain is unknown.
    declareParameter("authoritative", rclcpp::ParameterValue(false));
    node->get_parameter(name_ + ".authoritative", authoritative_);
    node->get_parameter(name_ + ".update_radius", update_radius_);
    node->get_parameter(name_ + ".enabled", enabled_);
    std::string topic;
    node->get_parameter(name_ + ".grid_topic", topic);
    double tolerance;
    node->get_parameter(name_ + ".transform_tolerance", tolerance);
    tolerance_ = tf2::durationFromSec(tolerance);

    sub_ = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
      topic, rclcpp::QoS(1).reliable().transient_local(),
      [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        grid_ = msg;
      });
    current_ = true;
  }

  void reset() override {}
  bool isClearable() override {return false;}

  void updateBounds(
    double robot_x, double robot_y, double /*robot_yaw*/,
    double * min_x, double * min_y, double * max_x, double * max_y) override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    have_tf_ = false;
    if (!enabled_ || !grid_) {
      return;
    }
    const std::string costmap_frame = layered_costmap_->getGlobalFrameID();
    geometry_msgs::msg::TransformStamped t;
    try {
      // costmap frame -> grid frame (to look up grid cells from costmap coordinates)
      t = tf_->lookupTransform(grid_->header.frame_id, costmap_frame, tf2::TimePointZero, tolerance_);
    } catch (const tf2::TransformException & e) {
      RCLCPP_WARN_THROTTLE(logger_, *clock_, 5000, "traversability: no TF %s -> %s: %s",
        costmap_frame.c_str(), grid_->header.frame_id.c_str(), e.what());
      return;
    }
    tx_ = t.transform.translation.x;
    ty_ = t.transform.translation.y;
    const auto & q = t.transform.rotation;
    const double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    c_ = std::cos(yaw);
    s_ = std::sin(yaw);
    have_tf_ = true;
    grid_used_ = grid_;

    // New grid: redraw around the rover. In a rolling costmap the whole window moves anyway.
    if (grid_used_ != drawn_ || layered_costmap_->isRolling()) {
      drawn_ = grid_used_;
      *min_x = std::min(*min_x, robot_x - update_radius_);
      *min_y = std::min(*min_y, robot_y - update_radius_);
      *max_x = std::max(*max_x, robot_x + update_radius_);
      *max_y = std::max(*max_y, robot_y + update_radius_);
    }
  }

  void updateCosts(
    nav2_costmap_2d::Costmap2D & master, int min_i, int min_j, int max_i, int max_j) override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!enabled_ || !have_tf_ || !grid_used_) {
      return;
    }
    const auto & info = grid_used_->info;
    const auto & data = grid_used_->data;
    const double res = info.resolution, ox = info.origin.position.x, oy = info.origin.position.y;
    const int w = static_cast<int>(info.width), h = static_cast<int>(info.height);
    unsigned char * costs = master.getCharMap();
    const unsigned int size_x = master.getSizeInCellsX();
    for (int j = min_j; j < max_j; ++j) {
      for (int i = min_i; i < max_i; ++i) {
        double wx, wy;
        master.mapToWorld(i, j, wx, wy);
        const double gx = c_ * wx - s_ * wy + tx_, gy = s_ * wx + c_ * wy + ty_;
        const int gi = static_cast<int>(std::floor((gx - ox) / res));
        const int gj = static_cast<int>(std::floor((gy - oy) / res));
        if (gi < 0 || gj < 0 || gi >= w || gj >= h) {
          continue;
        }
        const int8_t v = data[gj * w + gi];
        if (v < 0) {
          continue;
        }
        const unsigned char cost = v >= 100 ? nav2_costmap_2d::LETHAL_OBSTACLE :
          static_cast<unsigned char>(std::lround(v * 2.52));
        unsigned char & old = costs[j * size_x + i];
        if (authoritative_ || old == nav2_costmap_2d::NO_INFORMATION || cost > old) {
          old = cost;
        }
      }
    }
  }

private:
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_;
  nav_msgs::msg::OccupancyGrid::ConstSharedPtr grid_, grid_used_, drawn_;
  double update_radius_{15.0};
  std::mutex mutex_;
  tf2::Duration tolerance_;
  bool have_tf_{false};
  bool authoritative_{false};
  double tx_{0}, ty_{0}, c_{1}, s_{0};
};

}  // namespace kratos_traversability

PLUGINLIB_EXPORT_CLASS(kratos_traversability::TraversabilityLayer, nav2_costmap_2d::Layer)
