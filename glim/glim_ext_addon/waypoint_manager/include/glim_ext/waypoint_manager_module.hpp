#pragma once

#include <algorithm>
#include <mutex>
#include <chrono>
#include <string>
#include <vector>
#include <fstream>
#include <unordered_map>
#include <cmath>
#include <Eigen/Core>
#include <Eigen/Geometry>

#define GLIM_ROS2

#include <glim/mapping/callbacks.hpp>
#include <glim/mapping/sub_map.hpp>
#include <glim/odometry/callbacks.hpp>
#include <glim/util/logging.hpp>

#ifdef GLIM_ROS2
#include <glim/util/extension_module_ros2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <waypoint_interfaces/srv/add_waypoint.hpp>
#include <waypoint_interfaces/srv/get_waypoint.hpp>
#include <waypoint_interfaces/srv/save_waypoints.hpp>
#include <waypoint_interfaces/srv/list_waypoints.hpp>

using ExtensionModuleBase = glim::ExtensionModuleROS2;
#endif

#include <spdlog/spdlog.h>
#include <yaml-cpp/yaml.h>

namespace glim {

/**
 * @brief Tags waypoints relative to the current submap's local origin instead of a
 *        frozen world pose, so they automatically ride along with every pose-graph
 *        loop-closure correction instead of going stale.
 *
 * Persistence note: `local_pose` (submap-relative) is what's saved to disk, not a
 * resolved world coordinate - the day a relocalizer is added, this same file format
 * still works, it just needs submap correspondence solved on load. Loading is not
 * implemented yet (single-session use only for now); saving is, and autosaves every
 * 10s so a crash doesn't lose an entire session's tags again.
 */
class WaypointManager : public ExtensionModuleBase {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  struct WaypointEntry {
    std::string name;
    int submap_id;
    Eigen::Isometry3d T_submap_sensor;
  };

  // A completed submap expressed in GLIM odometry coordinates.  The global
  // mapper later changes T_world_origin, but these two values remain fixed.
  struct SubmapReference {
    int id;
    double first_stamp;
    double last_stamp;
    Eigen::Isometry3d T_odom_origin_sensor;
  };

  struct PendingWaypoint {
    std::string name;
    double stamp;
    Eigen::Isometry3d T_odom_sensor;
  };

  WaypointManager() : logger(create_module_logger("waypoint_manager")) {
    logger->info("initializing waypoint manager");

    using std::placeholders::_1;
    odom_update_callback_id = OdometryEstimationCallbacks::on_update_new_frame.add(std::bind(&WaypointManager::on_update_new_frame, this, _1));
    new_submap_callback_id = SubMappingCallbacks::on_new_submap.add(std::bind(&WaypointManager::on_new_submap, this, _1));
    insert_submap_callback_id = GlobalMappingCallbacks::on_insert_submap.add(std::bind(&WaypointManager::on_insert_submap, this, _1));
    update_submaps_callback_id = GlobalMappingCallbacks::on_update_submaps.add(std::bind(&WaypointManager::on_update_submaps, this, _1));
  }

  ~WaypointManager() override {
    // Callback slots outlive extension instances.  Remove every raw-this
    // callback so unloading the module cannot leave a dangling invocation.
    OdometryEstimationCallbacks::on_update_new_frame.remove(odom_update_callback_id);
    SubMappingCallbacks::on_new_submap.remove(new_submap_callback_id);
    GlobalMappingCallbacks::on_insert_submap.remove(insert_submap_callback_id);
    GlobalMappingCallbacks::on_update_submaps.remove(update_submaps_callback_id);
  }

  virtual std::vector<GenericTopicSubscription::Ptr> create_subscriptions(rclcpp::Node& node) override {
    marker_pub = node.create_publisher<visualization_msgs::msg::MarkerArray>("~/waypoints", 10);

    add_waypoint_srv = node.create_service<waypoint_interfaces::srv::AddWaypoint>(
      "add_waypoint",
      std::bind(&WaypointManager::add_waypoint_cb, this, std::placeholders::_1, std::placeholders::_2));

    get_waypoint_srv = node.create_service<waypoint_interfaces::srv::GetWaypoint>(
      "get_waypoint",
      std::bind(&WaypointManager::get_waypoint_cb, this, std::placeholders::_1, std::placeholders::_2));

    save_waypoints_srv = node.create_service<waypoint_interfaces::srv::SaveWaypoints>(
      "save_waypoints",
      std::bind(&WaypointManager::save_waypoints_cb, this, std::placeholders::_1, std::placeholders::_2));

    list_waypoints_srv = node.create_service<waypoint_interfaces::srv::ListWaypoints>(
      "list_waypoints",
      std::bind(&WaypointManager::list_waypoints_cb, this, std::placeholders::_1, std::placeholders::_2));

    marker_timer = node.create_wall_timer(std::chrono::milliseconds(1000), std::bind(&WaypointManager::publish_markers, this));
    autosave_timer = node.create_wall_timer(std::chrono::seconds(10), std::bind(&WaypointManager::autosave, this));

    logger->info("waypoint manager ready: ~/add_waypoint ~/get_waypoint ~/save_waypoints ~/list_waypoints ~/waypoints");
    return {};
  }

  void on_update_new_frame(const EstimationFrame::ConstPtr& frame) {
    if (!frame) {
      return;
    }
    std::lock_guard<std::mutex> lk(mtx);
    latest_frame_pose = frame->T_world_sensor();
    latest_frame_stamp = frame->stamp;
    has_frame = true;
  }

  void on_new_submap(const SubMap::ConstPtr& submap) {
    if (!submap || submap->frames.empty()) {
      return;
    }

    // A frame's T_world_sensor() is in GLIM's odometry trajectory despite its
    // historical name.  Keep the submap origin in that same coordinate system;
    // only T_world_origin is in the graph-corrected map frame.
    const auto& first = submap->frames.front();
    const auto& last = submap->frames.back();
    const auto& origin = submap->origin_frame();
    if (!first || !last || !origin) {
      return;
    }

    std::lock_guard<std::mutex> lk(mtx);
    submap_references.push_back({submap->id, first->stamp, last->stamp, origin->T_world_sensor()});
    bind_pending_waypoints_locked();
  }

  void on_insert_submap(const SubMap::ConstPtr& submap) {
    if (!submap) {
      return;
    }
    std::lock_guard<std::mutex> lk(mtx);
    submap_world_pose[submap->id] = submap->T_world_origin;
  }

  void on_update_submaps(const std::vector<SubMap::Ptr>& submaps) {
    std::lock_guard<std::mutex> lk(mtx);
    for (const auto& sm : submaps) {
      if (sm) {
        submap_world_pose[sm->id] = sm->T_world_origin;
      }
    }
  }

  void add_waypoint_cb(
    const std::shared_ptr<waypoint_interfaces::srv::AddWaypoint::Request> req,
    std::shared_ptr<waypoint_interfaces::srv::AddWaypoint::Response> res) {
    std::lock_guard<std::mutex> lk(mtx);

    if (!has_frame) {
      res->success = false;
      res->message = "no odometry yet - move the sensor first";
      return;
    }

    if (req->name.empty()) {
      res->success = false;
      res->message = "waypoint name must not be empty";
      return;
    }

    erase_waypoint_named_locked(req->name);
    erase_pending_waypoint_named_locked(req->name);
    const auto submap = find_submap_for_stamp_locked(latest_frame_stamp);
    if (submap) {
      add_waypoint_locked(req->name, latest_frame_pose, *submap);
      res->message = "waypoint '" + req->name + "' tagged in submap " + std::to_string(submap->id);
    } else {
      pending_waypoints.push_back({req->name, latest_frame_stamp, latest_frame_pose});
      res->message = "waypoint '" + req->name + "' queued until its submap is finalized";
      logger->info("queued waypoint '{}' at stamp={}", req->name, latest_frame_stamp);
    }
    res->success = true;
  }

  void get_waypoint_cb(
    const std::shared_ptr<waypoint_interfaces::srv::GetWaypoint::Request> req,
    std::shared_ptr<waypoint_interfaces::srv::GetWaypoint::Response> res) {
    std::lock_guard<std::mutex> lk(mtx);

    for (const auto& wp : waypoints) {
      if (wp.name != req->name) continue;
      const auto it = submap_world_pose.find(wp.submap_id);
      if (it == submap_world_pose.end()) break;

      // Resolved fresh from the submap's CURRENT pose - this is the whole point.
      const Eigen::Isometry3d T_world = it->second * wp.T_submap_sensor;
      const Eigen::Vector3d t = T_world.translation();
      const Eigen::Quaterniond q = Eigen::Quaterniond(T_world.linear()).normalized();

      res->found = true;
      res->pose.header.frame_id = "map";
      res->pose.header.stamp = rclcpp::Clock().now();
      res->pose.pose.position.x = t.x();
      res->pose.pose.position.y = t.y();
      res->pose.pose.position.z = t.z();
      res->pose.pose.orientation.w = q.w();
      res->pose.pose.orientation.x = q.x();
      res->pose.pose.orientation.y = q.y();
      res->pose.pose.orientation.z = q.z();
      return;
    }

    res->found = false;
  }

  void list_waypoints_cb(
    const std::shared_ptr<waypoint_interfaces::srv::ListWaypoints::Request>,
    std::shared_ptr<waypoint_interfaces::srv::ListWaypoints::Response> res) {
    std::lock_guard<std::mutex> lk(mtx);
    for (const auto& wp : waypoints) {
      res->names.push_back(wp.name);
    }
    for (const auto& pwp : pending_waypoints) {
      res->names.push_back(pwp.name + " (pending)");
    }
  }

  void save_waypoints_cb(
    const std::shared_ptr<waypoint_interfaces::srv::SaveWaypoints::Request> req,
    std::shared_ptr<waypoint_interfaces::srv::SaveWaypoints::Response> res) {
    std::lock_guard<std::mutex> lk(mtx);
    std::string path = req->path;
    if (path.empty()) {
      path = "/tmp/waypoints.yaml";
    }
    const bool ok = write_yaml(path);
    res->success = ok;
    res->message = ok ? ("saved " + std::to_string(waypoints.size()) + " waypoints (" +
                          std::to_string(pending_waypoints.size()) + " pending) to " + path)
                       : ("failed to open " + path);
  }

  void autosave() {
    std::lock_guard<std::mutex> lk(mtx);
    if (waypoints.empty() && pending_waypoints.empty()) {
      return;
    }
    write_yaml(autosave_path);
  }

  // NOTE: caller must already hold mtx.
  bool write_yaml(const std::string& path) {
    std::ofstream out(path);
    if (!out) {
      logger->warn("failed to open {} for writing waypoints", path);
      return false;
    }

    YAML::Emitter yml;
    yml << YAML::BeginMap;
    yml << YAML::Key << "waypoints" << YAML::Value << YAML::BeginSeq;
    for (const auto& wp : waypoints) {
      const Eigen::Vector3d t = wp.T_submap_sensor.translation();
      const Eigen::Quaterniond q = Eigen::Quaterniond(wp.T_submap_sensor.linear()).normalized();

      yml << YAML::BeginMap;
      yml << YAML::Key << "name" << YAML::Value << wp.name;
      yml << YAML::Key << "submap_id" << YAML::Value << wp.submap_id;
      yml << YAML::Key << "local_xyz" << YAML::Value << YAML::Flow << std::vector<double>{t.x(), t.y(), t.z()};
      yml << YAML::Key << "local_qxyzw" << YAML::Value << YAML::Flow << std::vector<double>{q.x(), q.y(), q.z(), q.w()};

      const auto it = submap_world_pose.find(wp.submap_id);
      if (it != submap_world_pose.end()) {
        const Eigen::Isometry3d T_world = it->second * wp.T_submap_sensor;
        const Eigen::Vector3d t_map = T_world.translation();
        const Eigen::Quaterniond q_map = Eigen::Quaterniond(T_world.linear()).normalized();
        const double yaw = std::atan2(2.0 * (q_map.w() * q_map.z() + q_map.x() * q_map.y()),
                                      1.0 - 2.0 * (q_map.y() * q_map.y() + q_map.z() * q_map.z()));
        yml << YAML::Key << "map_xyz" << YAML::Value << YAML::Flow << std::vector<double>{t_map.x(), t_map.y(), t_map.z()};
        yml << YAML::Key << "map_yaw" << YAML::Value << yaw;
        yml << YAML::Key << "map_qxyzw" << YAML::Value << YAML::Flow << std::vector<double>{q_map.x(), q_map.y(), q_map.z(), q_map.w()};
      }

      yml << YAML::EndMap;
    }
    yml << YAML::EndSeq;

    // Waypoints tagged inside the still-forming submap have no submap
    // reference yet (bind_pending_waypoints_locked() hasn't run for them) -
    // without this, ending a mapping session right after tagging one drops
    // it silently, since it never enters `waypoints`. Persist the raw
    // odometry pose as a best-effort fallback: not loop-closure-corrected,
    // but recoverable instead of lost.
    yml << YAML::Key << "pending_waypoints" << YAML::Value << YAML::BeginSeq;
    for (const auto& pwp : pending_waypoints) {
      const Eigen::Vector3d t = pwp.T_odom_sensor.translation();
      const Eigen::Quaterniond q = Eigen::Quaterniond(pwp.T_odom_sensor.linear()).normalized();

      yml << YAML::BeginMap;
      yml << YAML::Key << "name" << YAML::Value << pwp.name;
      yml << YAML::Key << "stamp" << YAML::Value << pwp.stamp;
      yml << YAML::Key << "odom_xyz" << YAML::Value << YAML::Flow << std::vector<double>{t.x(), t.y(), t.z()};
      yml << YAML::Key << "odom_qxyzw" << YAML::Value << YAML::Flow << std::vector<double>{q.x(), q.y(), q.z(), q.w()};
      yml << YAML::EndMap;
    }
    yml << YAML::EndSeq;
    yml << YAML::EndMap;

    out << yml.c_str();
    logger->info("wrote {} waypoints ({} pending) to {}", waypoints.size(), pending_waypoints.size(), path);
    return true;
  }

  void publish_markers() {
    std::lock_guard<std::mutex> lk(mtx);
    if (!marker_pub || waypoints.empty()) {
      return;
    }

    const auto stamp = rclcpp::Clock().now();
    visualization_msgs::msg::MarkerArray markers;
    int idx = 0;
    for (const auto& wp : waypoints) {
      const auto it = submap_world_pose.find(wp.submap_id);
      if (it == submap_world_pose.end()) {
        continue;
      }
      // Recomputed from the submap's CURRENT (possibly loop-closure-corrected) pose every publish.
      Eigen::Isometry3d T_world = it->second * wp.T_submap_sensor;
      T_world.linear() = Eigen::Quaterniond(T_world.linear()).normalized().toRotationMatrix();
      const Eigen::Vector3d t = T_world.translation();
      const Eigen::Quaterniond q(T_world.linear());

      visualization_msgs::msg::Marker m;
      m.header.frame_id = "map";
      m.header.stamp = stamp;
      m.ns = "waypoints";
      m.id = idx;
      m.type = visualization_msgs::msg::Marker::SPHERE;
      m.action = visualization_msgs::msg::Marker::ADD;
      m.pose.position.x = t.x();
      m.pose.position.y = t.y();
      m.pose.position.z = t.z();
      m.pose.orientation.w = q.w();
      m.pose.orientation.x = q.x();
      m.pose.orientation.y = q.y();
      m.pose.orientation.z = q.z();
      m.scale.x = m.scale.y = m.scale.z = 0.3;
      m.color.a = 1.0;
      m.color.r = 0.1;
      m.color.g = 0.9;
      m.color.b = 0.2;
      markers.markers.push_back(m);

      visualization_msgs::msg::Marker text;
      text.header = m.header;
      text.ns = "waypoint_names";
      text.id = idx;
      text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      text.action = visualization_msgs::msg::Marker::ADD;
      text.pose = m.pose;
      text.pose.position.z += 0.4;
      text.scale.z = 0.25;
      text.color.a = 1.0;
      text.color.r = 1.0;
      text.color.g = 1.0;
      text.color.b = 1.0;
      text.text = wp.name;
      markers.markers.push_back(text);

      idx++;
    }
    marker_pub->publish(markers);
  }

  // Accessors for testing
  const std::vector<WaypointEntry>& get_waypoints() const { return waypoints; }
  size_t count() const { return waypoints.size(); }

private:
  const SubmapReference* find_submap_for_stamp_locked(const double stamp) const {
    for (auto it = submap_references.rbegin(); it != submap_references.rend(); ++it) {
      if (it->first_stamp <= stamp && stamp <= it->last_stamp) {
        return &*it;
      }
    }
    return nullptr;
  }

  void erase_waypoint_named_locked(const std::string& name) {
    waypoints.erase(
      std::remove_if(waypoints.begin(), waypoints.end(), [&name](const WaypointEntry& wp) { return wp.name == name; }),
      waypoints.end());
  }

  void erase_pending_waypoint_named_locked(const std::string& name) {
    pending_waypoints.erase(
      std::remove_if(pending_waypoints.begin(), pending_waypoints.end(), [&name](const PendingWaypoint& wp) { return wp.name == name; }),
      pending_waypoints.end());
  }

  void add_waypoint_locked(
    const std::string& name,
    const Eigen::Isometry3d& T_odom_sensor,
    const SubmapReference& submap) {
    // Both terms are odometry poses, so this is invariant to all prior and
    // future global-map corrections.  It can safely be multiplied by the live
    // T_world_origin when the waypoint is queried or visualized.
    const Eigen::Isometry3d local_pose = submap.T_odom_origin_sensor.inverse() * T_odom_sensor;
    waypoints.push_back({name, submap.id, local_pose});
    logger->info("tagged waypoint '{}' submap_id={}", name, submap.id);
  }

  void bind_pending_waypoints_locked() {
    for (auto it = pending_waypoints.begin(); it != pending_waypoints.end();) {
      const auto submap = find_submap_for_stamp_locked(it->stamp);
      if (!submap) {
        ++it;
        continue;
      }

      add_waypoint_locked(it->name, it->T_odom_sensor, *submap);
      it = pending_waypoints.erase(it);
    }
  }

  std::mutex mtx;
  bool has_frame = false;
  Eigen::Isometry3d latest_frame_pose = Eigen::Isometry3d::Identity();
  double latest_frame_stamp = 0.0;

  std::unordered_map<int, Eigen::Isometry3d> submap_world_pose;
  std::vector<SubmapReference> submap_references;
  std::vector<WaypointEntry> waypoints;
  std::vector<PendingWaypoint> pending_waypoints;

  const std::string autosave_path = "/tmp/waypoints_autosave.yaml";

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub;
  rclcpp::Service<waypoint_interfaces::srv::AddWaypoint>::SharedPtr add_waypoint_srv;
  rclcpp::Service<waypoint_interfaces::srv::GetWaypoint>::SharedPtr get_waypoint_srv;
  rclcpp::Service<waypoint_interfaces::srv::SaveWaypoints>::SharedPtr save_waypoints_srv;
  rclcpp::Service<waypoint_interfaces::srv::ListWaypoints>::SharedPtr list_waypoints_srv;
  rclcpp::TimerBase::SharedPtr marker_timer;
  rclcpp::TimerBase::SharedPtr autosave_timer;

  std::shared_ptr<spdlog::logger> logger;

  int odom_update_callback_id = -1;
  int new_submap_callback_id = -1;
  int insert_submap_callback_id = -1;
  int update_submaps_callback_id = -1;
};

}  // namespace glim

extern "C" glim::ExtensionModule* create_extension_module() {
  return new glim::WaypointManager();
}
