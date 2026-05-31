#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <vision_msgs/msg/detection3_d_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <string>
#include <vector>

namespace james_perception
{

class BottleDetector : public rclcpp::Node
{
public:
  explicit BottleDetector(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  // Public so that unit / BDD tests can call it directly without ROS spin.
  vision_msgs::msg::Detection3DArray detect(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
    const std_msgs::msg::Header & header);

  struct CapColor {
    std::string name;
    float hue;
    float hue_tol;
  };

  // Per-slot data used for the ASCII grid display.
  struct SlotGridInfo {
    std::string name;      // "empty" or bottle type name
    int         points  = 0;
    float       hue     = -1.0f;   // -1 = unknown
    float       occ_conf = 0.0f;   // 0-100: how sure there is a bottle
    float       col_conf = -1.0f;  // 0-100: how sure of the colour; -1 = n/a
  };

private:
  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  // Convert an 8-bit RGB triplet to HSV.
  // h in [0, 360), s and v in [0, 255].
  static void rgbToHsv(uint8_t r, uint8_t g, uint8_t b,
                        float & h, float & s, float & v);

  // Given a list of hue values from the cap points of one slot,
  // return the name of the matching cap colour (or "unknown").
  std::string classifyHue(const std::vector<float> & hues,
                          float & out_mean_hue,
                          float & out_col_conf) const;

  // Reload all ROS2 parameters into member variables.
  // Called at the start of every detect() so ros2 param set takes effect immediately.
  void reloadParameters();

  // Log a table of all slot positions to the console.
  // Called once on startup; call again after changing crate geometry parameters.
  void logSlotMap();

  void publishMarkers(const vision_msgs::msg::Detection3DArray & detections);

  // Print a 3×4 ASCII grid of all slots to the console (throttled to ~1 Hz).
  // On subsequent calls the grid is overwritten in-place using ANSI escape codes.
  void logCrateGrid(const std::vector<std::vector<SlotGridInfo>> & grid,
                    std::size_t filled) const;

  mutable bool first_grid_ = true;   // true until the first grid has been printed

  // Convert a hue angle (0–360°) to an RGBA colour at full saturation and value.
  // Used to derive marker colours directly from the configured cap hues.
  static std_msgs::msg::ColorRGBA hueToRgba(float hue, float alpha = 0.5f);

  // Return the display colour for a cap name by looking up its hue.
  // Falls back to grey for "unknown".
  std_msgs::msg::ColorRGBA colorForCap(const std::string & cap_name) const;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rclcpp::Publisher<vision_msgs::msg::Detection3DArray>::SharedPtr detections_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr crate_cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr slot_cloud_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;

  // ── Camera tilt (radians, converted from degrees in reloadParameters) ────────
  float camera_tilt_x_rad_ = 0.0f;
  float camera_tilt_y_rad_ = 0.0f;

  // ── Crate bounding box in camera frame (metres) ─────────────────────────────
  // All six values are explicit min/max — tune them directly in RViz2.
  double crate_z_near_;   // Z: distance to bottle tops (closest to camera)
  double crate_z_far_;    // Z: distance to crate bottom
  double crate_x_min_;    // X: left edge of crate
  double crate_x_max_;    // X: right edge of crate
  double crate_y_min_;    // Y: near edge of crate
  double crate_y_max_;    // Y: far edge of crate

  // ── Slot grid ────────────────────────────────────────────────────────────────
  int crate_rows_;
  int crate_cols_;

  // ── Occupancy ────────────────────────────────────────────────────────────────
  int min_points_per_slot_;

  // ── Cap colour classification ─────────────────────────────────────────────
  std::vector<CapColor> cap_colors_;
  float cap_saturation_min_;  // ignore desaturated (grey/white) pixels
  float cap_value_min_;       // ignore very dark pixels
};

}  // namespace james_perception
