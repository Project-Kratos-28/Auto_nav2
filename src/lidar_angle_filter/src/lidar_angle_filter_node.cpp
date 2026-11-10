#include "lidar_angle_filter/angle_filter.hpp"

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <functional>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>

namespace lidar_angle_filter
{

struct CoordinateField
{
  std::uint32_t offset;
  std::uint8_t datatype;
};

class LidarAngleFilterNode : public rclcpp::Node
{
public:
  LidarAngleFilterNode()
  : Node("lidar_angle_filter")
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/livox/lidar");
    output_topic_ = declare_parameter<std::string>("output_topic", "/livox/lidar_filtered");
    front_center_deg_ = declare_parameter<double>("front_center_deg", 0.0);
    front_sector_deg_ = declare_parameter<double>("front_sector_deg", 30.0);
    back_sector_deg_ = declare_parameter<double>("back_sector_deg", 30.0);
    // Scan gate: a scan with fewer than min_usable_points points at least
    // min_usable_range metres from the sensor is dropped, not published.
    // 0 disables the gate. Keep min_usable_range above GLIM's
    // distance_near_thresh (0.5 m in config_preprocess.json).
    min_usable_points_ = declare_parameter<int>("min_usable_points", 120);
    min_usable_range_ = declare_parameter<double>("min_usable_range", 0.7);

    // Vertical window in degrees above the sensor's horizontal plane. The defaults
    // (-90..90) keep every point. See config/angle_filter.yaml before changing them.
    min_elevation_deg_ = declare_parameter<double>("min_elevation_deg", -90.0);
    max_elevation_deg_ = declare_parameter<double>("max_elevation_deg", 90.0);

    validate_parameters();

    publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      output_topic_, rclcpp::SensorDataQoS());
    subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_, rclcpp::SensorDataQoS(),
      std::bind(&LidarAngleFilterNode::filter_cloud, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "Filtering %.1f deg at the front (center %.1f deg) and %.1f deg at the back: %s -> %s",
      front_sector_deg_, front_center_deg_, back_sector_deg_, input_topic_.c_str(), output_topic_.c_str());
  }

private:
  void validate_parameters() const
  {
    if (input_topic_ == output_topic_) {
      throw std::invalid_argument("input_topic and output_topic must be different");
    }
    if (!std::isfinite(front_center_deg_)) {
      throw std::invalid_argument("front_center_deg must be finite");
    }
    if (min_usable_points_ < 0 || !std::isfinite(min_usable_range_) || min_usable_range_ < 0.0) {
      throw std::invalid_argument("min_usable_points and min_usable_range must be >= 0");
    }
    if (!std::isfinite(min_elevation_deg_) || !std::isfinite(max_elevation_deg_) ||
      min_elevation_deg_ < -90.0 || max_elevation_deg_ > 90.0 ||
      min_elevation_deg_ >= max_elevation_deg_)
    {
      throw std::invalid_argument(
        "min_elevation_deg and max_elevation_deg must satisfy -90 <= min < max <= 90");
    }
    if (!valid_sector_width(front_sector_deg_) || !valid_sector_width(back_sector_deg_)) {
      throw std::invalid_argument("front_sector_deg and back_sector_deg must be between 0 and 180");
    }
  }

  static bool valid_sector_width(const double width)
  {
    return std::isfinite(width) && width >= 0.0 && width <= 180.0;
  }

  static CoordinateField find_coordinate_field(
    const sensor_msgs::msg::PointCloud2 & cloud, const std::string & name)
  {
    const auto field = std::find_if(
      cloud.fields.begin(), cloud.fields.end(),
      [&name](const sensor_msgs::msg::PointField & candidate) {return candidate.name == name;});

    if (field == cloud.fields.end()) {
      throw std::runtime_error("PointCloud2 is missing the '" + name + "' field");
    }
    if (field->datatype != sensor_msgs::msg::PointField::FLOAT32 &&
      field->datatype != sensor_msgs::msg::PointField::FLOAT64)
    {
      throw std::runtime_error("PointCloud2 field '" + name + "' must be FLOAT32 or FLOAT64");
    }

    const std::size_t field_size =
      field->datatype == sensor_msgs::msg::PointField::FLOAT32 ? sizeof(float) : sizeof(double);
    if (field->offset + field_size > cloud.point_step) {
      throw std::runtime_error("PointCloud2 field '" + name + "' extends beyond point_step");
    }
    return {field->offset, field->datatype};
  }

  template<typename T>
  static T byte_swap(T value)
  {
    std::uint8_t source[sizeof(T)];
    std::uint8_t destination[sizeof(T)];
    std::memcpy(source, &value, sizeof(T));
    std::reverse_copy(source, source + sizeof(T), destination);
    std::memcpy(&value, destination, sizeof(T));
    return value;
  }

  static bool host_is_big_endian()
  {
    const std::uint16_t value = 0x0102;
    return *reinterpret_cast<const std::uint8_t *>(&value) == 0x01;
  }

  static double read_coordinate(
    const std::uint8_t * point, const CoordinateField & field, const bool cloud_is_big_endian)
  {
    if (field.datatype == sensor_msgs::msg::PointField::FLOAT32) {
      float value;
      std::memcpy(&value, point + field.offset, sizeof(value));
      if (cloud_is_big_endian != host_is_big_endian()) {
        value = byte_swap(value);
      }
      return value;
    }

    double value;
    std::memcpy(&value, point + field.offset, sizeof(value));
    if (cloud_is_big_endian != host_is_big_endian()) {
      value = byte_swap(value);
    }
    return value;
  }

  void filter_cloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud)
  {
    try {
      const CoordinateField x_field = find_coordinate_field(*cloud, "x");
      const CoordinateField y_field = find_coordinate_field(*cloud, "y");
      const CoordinateField z_field = find_coordinate_field(*cloud, "z");

      auto filtered = std::make_unique<sensor_msgs::msg::PointCloud2>();
      filtered->header = cloud->header;
      filtered->height = 1;
      filtered->fields = cloud->fields;
      filtered->is_bigendian = cloud->is_bigendian;
      filtered->point_step = cloud->point_step;
      filtered->is_dense = cloud->is_dense;
      filtered->data.reserve(cloud->data.size());

      std::size_t kept_points = 0;
      std::size_t masked_points = 0;
      unsigned long usable_points = 0;
      for (std::uint32_t row = 0; row < cloud->height; ++row) {
        const std::size_t row_offset = static_cast<std::size_t>(row) * cloud->row_step;
        for (std::uint32_t column = 0; column < cloud->width; ++column) {
          const std::size_t offset = row_offset + static_cast<std::size_t>(column) * cloud->point_step;
          if (offset + cloud->point_step > cloud->data.size()) {
            throw std::runtime_error("PointCloud2 data is shorter than its dimensions declare");
          }

          const std::uint8_t * point = cloud->data.data() + offset;
          const double x = read_coordinate(point, x_field, cloud->is_bigendian);
          const double y = read_coordinate(point, y_field, cloud->is_bigendian);
          const double z = read_coordinate(point, z_field, cloud->is_bigendian);
          if (!point_in_elevation_range(x, y, z, min_elevation_deg_, max_elevation_deg_)) {
            ++masked_points;
            continue;
          }
          if (point_is_masked(
              x, y, front_center_deg_, front_sector_deg_, back_sector_deg_))
          {
            ++masked_points;
            continue;
          }

          if (point_is_usable(x, y, z, min_usable_range_)) {
            ++usable_points;
          }

          filtered->data.insert(filtered->data.end(), point, point + cloud->point_step);
          ++kept_points;
        }
      }

      if (!scan_has_enough_points(usable_points, static_cast<unsigned long>(min_usable_points_))) {
        ++dropped_scans_;
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Dropping scan: only %lu points beyond %.2f m (need %d). LiDAR covered or blocked? "
          "%lu scans dropped so far. GLIM is not sent this scan.",
          usable_points, min_usable_range_, min_usable_points_, dropped_scans_);
        return;
      }

      filtered->width = static_cast<std::uint32_t>(kept_points);
      filtered->row_step = filtered->width * filtered->point_step;
      publisher_->publish(std::move(filtered));

      RCLCPP_DEBUG_THROTTLE(
        get_logger(), *get_clock(), 5000, "Kept %zu points; masked %zu points",
        kept_points, masked_points);
    } catch (const std::exception & error) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "%s", error.what());
    }
  }

  std::string input_topic_;
  std::string output_topic_;
  double front_center_deg_;
  double front_sector_deg_;
  double back_sector_deg_;
  int min_usable_points_;
  double min_usable_range_;
  double min_elevation_deg_;
  double max_elevation_deg_;
  unsigned long dropped_scans_ = 0;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
};

}  // namespace lidar_angle_filter

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<lidar_angle_filter::LidarAngleFilterNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("lidar_angle_filter"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
