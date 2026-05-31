#include "james_perception/bottle_detector.hpp"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>

#include <vision_msgs/msg/detection3_d.hpp>
#include <vision_msgs/msg/object_hypothesis_with_pose.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

namespace james_perception
{

// ---------------------------------------------------------------------------
// Constructor — declare all ROS2 parameters with sensible defaults
// ---------------------------------------------------------------------------
BottleDetector::BottleDetector(const rclcpp::NodeOptions & options)
: Node("bottle_detector", options)
{
  // Camera tilt relative to crate top surface (degrees, default 0 = perfectly vertical).
  // Tune with /james/crate_cloud in RViz2: adjust until the crate surface appears
  // at a uniform Z depth across all slots.
  //   tilt_x_deg > 0  →  camera leans forward  (far edge of crate moves closer in Z)
  //   tilt_y_deg > 0  →  camera leans rightward (right edge moves closer in Z)
  declare_parameter("camera_tilt_x_deg", 0.0);
  declare_parameter("camera_tilt_y_deg", 0.0);

  // Crate bounding box — explicit min/max per axis, no centering needed
  declare_parameter("crate_z_near",  0.20);
  declare_parameter("crate_z_far",   0.50);
  declare_parameter("crate_x_min",  -0.20);
  declare_parameter("crate_x_max",   0.20);
  declare_parameter("crate_y_min",  -0.15);
  declare_parameter("crate_y_max",   0.15);

  // Slot grid
  declare_parameter("crate_rows", 3);
  declare_parameter("crate_cols", 4);

  // Occupancy
  declare_parameter("min_points_per_slot", 20);
  declare_parameter("terminal_update_hz", 1.0);

  // Cap colour: how many bottle types (max 4)
  declare_parameter("num_cap_colors", 3);

  // Cap colour slots 0-3
  // Hue in degrees [0, 360).  Run with a bottle and read the "hue=XX°" log
  // lines to find the right value for each cap, then update the launch file.
  for (int i = 0; i < 4; ++i) {
    const std::string p = "cap_" + std::to_string(i) + "_";
    declare_parameter(p + "name",    "unknown_" + std::to_string(i));
    declare_parameter(p + "hue",     0.0);
    declare_parameter(p + "hue_tol", 20.0);
  }

  // Colour filter: pixels below these thresholds are grey/dark and ignored
  // when computing the dominant cap hue.
  declare_parameter("cap_saturation_min", 80.0);   // 0-255
  declare_parameter("cap_value_min",      50.0);   // 0-255

  subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    "/oak/points", rclcpp::SensorDataQoS(),
    std::bind(&BottleDetector::pointCloudCallback, this, std::placeholders::_1));

  detections_pub_ = create_publisher<vision_msgs::msg::Detection3DArray>(
    "/james/detections", 10);

  crate_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
    "/james/crate_cloud", rclcpp::SensorDataQoS());

  // Each point coloured by slot index — use to verify slot boundaries in RViz2
  slot_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
    "/james/slot_cloud", rclcpp::SensorDataQoS());

  markers_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
    "/james/bottle_markers", 10);

  reloadParameters();
  logSlotMap();
}

// ---------------------------------------------------------------------------
// ROS callback
// ---------------------------------------------------------------------------
void BottleDetector::pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::fromROSMsg(*msg, *cloud);
  detections_pub_->publish(detect(cloud, msg->header));
}

// ---------------------------------------------------------------------------
// logSlotMap — print the fixed XYZ of every slot so the user has a reference.
// The robot arm can use these coordinates directly (in the "oak" camera frame).
// Call once on startup and whenever parameters change.
// ---------------------------------------------------------------------------
void BottleDetector::logSlotMap()
{
  double crate_w = crate_x_max_ - crate_x_min_;
  double crate_d = crate_y_max_ - crate_y_min_;
  double slot_w  = crate_w / crate_cols_;
  double slot_d  = crate_d / crate_rows_;

  RCLCPP_INFO(get_logger(),
    "BottleDetector ready — %d×%d grid, crate %.0f×%.0f cm, "
    "X[%.2f..%.2f] Y[%.2f..%.2f] Z[%.2f..%.2f], %zu cap colour(s)",
    crate_rows_, crate_cols_, crate_w * 100, crate_d * 100,
    crate_x_min_, crate_x_max_, crate_y_min_, crate_y_max_,
    crate_z_near_, crate_z_far_, cap_colors_.size());

  RCLCPP_INFO(get_logger(), "Slot map (camera/oak frame, metres):");
  RCLCPP_INFO(get_logger(), "  %-12s  %6s  %6s  %6s", "slot", "x", "y", "z");

  for (int row = 0; row < crate_rows_; ++row) {
    for (int col = 0; col < crate_cols_; ++col) {
      double cx = crate_x_min_ + (col + 0.5) * slot_w;
      double cy = crate_y_min_ + (row + 0.5) * slot_d;
      RCLCPP_INFO(get_logger(), "  slot[%d][%d]     %+.3f  %+.3f  %+.3f",
        row, col, cx, cy, crate_z_near_);
    }
  }
}

// ---------------------------------------------------------------------------
// reloadParameters — read all params so ros2 param set takes effect instantly
// ---------------------------------------------------------------------------
void BottleDetector::reloadParameters()
{
  const double deg2rad = M_PI / 180.0;
  camera_tilt_x_rad_ = static_cast<float>(
    get_parameter("camera_tilt_x_deg").as_double() * deg2rad);
  camera_tilt_y_rad_ = static_cast<float>(
    get_parameter("camera_tilt_y_deg").as_double() * deg2rad);

  crate_z_near_        = get_parameter("crate_z_near").as_double();
  crate_z_far_         = get_parameter("crate_z_far").as_double();
  crate_x_min_         = get_parameter("crate_x_min").as_double();
  crate_x_max_         = get_parameter("crate_x_max").as_double();
  crate_y_min_         = get_parameter("crate_y_min").as_double();
  crate_y_max_         = get_parameter("crate_y_max").as_double();
  crate_rows_          = static_cast<int>(get_parameter("crate_rows").as_int());
  crate_cols_          = static_cast<int>(get_parameter("crate_cols").as_int());
  min_points_per_slot_  = static_cast<int>(get_parameter("min_points_per_slot").as_int());
  terminal_update_hz_   = get_parameter("terminal_update_hz").as_double();
  cap_saturation_min_  = static_cast<float>(get_parameter("cap_saturation_min").as_double());
  cap_value_min_       = static_cast<float>(get_parameter("cap_value_min").as_double());

  int n = static_cast<int>(get_parameter("num_cap_colors").as_int());
  cap_colors_.clear();
  for (int i = 0; i < n && i < 4; ++i) {
    const std::string p = "cap_" + std::to_string(i) + "_";
    CapColor c;
    c.name    = get_parameter(p + "name").as_string();
    c.hue     = static_cast<float>(get_parameter(p + "hue").as_double());
    c.hue_tol = static_cast<float>(get_parameter(p + "hue_tol").as_double());
    cap_colors_.push_back(c);
  }
}

// ---------------------------------------------------------------------------
// rgbToHsv — converts one pixel; h∈[0,360), s and v∈[0,255]
// ---------------------------------------------------------------------------
void BottleDetector::rgbToHsv(uint8_t r, uint8_t g, uint8_t b,
                               float & h, float & s, float & v)
{
  float rf = r / 255.0f, gf = g / 255.0f, bf = b / 255.0f;
  float cmax = std::max({rf, gf, bf});
  float cmin = std::min({rf, gf, bf});
  float delta = cmax - cmin;

  v = cmax * 255.0f;
  s = (cmax > 1e-6f) ? (delta / cmax * 255.0f) : 0.0f;

  if (delta < 1e-6f) { h = 0.0f; return; }  // achromatic

  if (cmax == rf) {
    h = 60.0f * std::fmod((gf - bf) / delta, 6.0f);
  } else if (cmax == gf) {
    h = 60.0f * ((bf - rf) / delta + 2.0f);
  } else {
    h = 60.0f * ((rf - gf) / delta + 4.0f);
  }
  if (h < 0.0f) h += 360.0f;
}

// ---------------------------------------------------------------------------
// classifyHue — circular mean of hues → nearest cap colour
// ---------------------------------------------------------------------------
std::string BottleDetector::classifyHue(
  const std::vector<float> & hues, float & out_mean_hue, float & out_col_conf) const
{
  out_mean_hue = -1.0f;
  out_col_conf = -1.0f;

  if (hues.size() < 5) {
    return "unknown";
  }

  float sin_sum = 0.0f, cos_sum = 0.0f;
  for (float hue : hues) {
    float rad = hue * static_cast<float>(M_PI) / 180.0f;
    sin_sum += std::sin(rad);
    cos_sum += std::cos(rad);
  }
  out_mean_hue = std::atan2(sin_sum, cos_sum) * 180.0f / static_cast<float>(M_PI);
  if (out_mean_hue < 0.0f) out_mean_hue += 360.0f;

  float best_dist = std::numeric_limits<float>::max();
  float best_tol  = 1.0f;
  std::string best_name = "unknown";

  for (const auto & cap : cap_colors_) {
    float diff = std::abs(out_mean_hue - cap.hue);
    if (diff > 180.0f) diff = 360.0f - diff;
    if (diff < cap.hue_tol && diff < best_dist) {
      best_dist = diff;
      best_tol  = cap.hue_tol;
      best_name = cap.name;
    }
  }

  if (best_name != "unknown") {
    out_col_conf = (1.0f - best_dist / best_tol) * 100.0f;
  }

  return best_name;
}

// ---------------------------------------------------------------------------
// hueToRgba — full-saturation, full-value colour from a hue angle (0–360°)
// ---------------------------------------------------------------------------
std_msgs::msg::ColorRGBA BottleDetector::hueToRgba(float hue, float alpha)
{
  // HSV→RGB with S=1, V=1
  float h = hue / 60.0f;
  int   i = static_cast<int>(h) % 6;
  float f = h - std::floor(h);
  float r, g, b;
  switch (i) {
    case 0: r=1; g=f; b=0; break;
    case 1: r=1-f; g=1; b=0; break;
    case 2: r=0; g=1; b=f; break;
    case 3: r=0; g=1-f; b=1; break;
    case 4: r=f; g=0; b=1; break;
    default: r=1; g=0; b=1-f; break;
  }
  std_msgs::msg::ColorRGBA c;
  c.r = r; c.g = g; c.b = b; c.a = alpha;
  return c;
}

// ---------------------------------------------------------------------------
// colorForCap — look up the display colour for a given cap name
// ---------------------------------------------------------------------------
std_msgs::msg::ColorRGBA BottleDetector::colorForCap(const std::string & cap_name) const
{
  for (const auto & cap : cap_colors_) {
    if (cap.name == cap_name) {
      return hueToRgba(cap.hue, 0.6f);
    }
  }
  // "unknown" → grey
  std_msgs::msg::ColorRGBA grey;
  grey.r = 0.5f; grey.g = 0.5f; grey.b = 0.5f; grey.a = 0.4f;
  return grey;
}

// ---------------------------------------------------------------------------
// publishMarkers — one coloured box + label per detected bottle
// ---------------------------------------------------------------------------
void BottleDetector::publishMarkers(
  const std::vector<std::vector<SlotGridInfo>> & grid,
  const std_msgs::msg::Header & header)
{
  visualization_msgs::msg::MarkerArray array;

  // Publish all 12 slot markers every frame with fixed IDs.
  // Empty slots get alpha=0 (invisible). RViz2 updates markers in-place
  // without a delete-then-add cycle, so there is no flickering.
  // Lifetime=0 means "persist until replaced" — no timeout needed.

  const double slot_w = (crate_x_max_ - crate_x_min_) / crate_cols_;
  const double slot_d = (crate_y_max_ - crate_y_min_) / crate_rows_;
  const double box_z  = (crate_z_near_ + crate_z_far_) / 2.0;
  const double box_sz = crate_z_far_ - crate_z_near_;

  for (int row = 0; row < crate_rows_; ++row) {
    for (int col = 0; col < crate_cols_; ++col) {
      const auto & slot = grid[row][col];
      const bool filled = (slot.name != "empty");

      const double cx = crate_x_min_ + (col + 0.5) * slot_w;
      const double cy = crate_y_min_ + (row + 0.5) * slot_d;
      const int slot_id = row * crate_cols_ + col;

      // ── Box ────────────────────────────────────────────────────────────────
      visualization_msgs::msg::Marker box;
      box.header   = header;
      box.ns       = "bottles";
      box.id       = slot_id;
      box.lifetime = rclcpp::Duration(0, 0);
      box.action   = filled ? visualization_msgs::msg::Marker::ADD
                            : visualization_msgs::msg::Marker::DELETE;
      if (filled) {
        box.type   = visualization_msgs::msg::Marker::CUBE;
        box.pose.position.x    = cx;
        box.pose.position.y    = cy;
        box.pose.position.z    = box_z;
        box.pose.orientation.w = 1.0;
        box.scale.x = slot_w;
        box.scale.y = slot_d;
        box.scale.z = box_sz;
        box.color   = colorForCap(slot.name);
      }
      array.markers.push_back(box);

      // ── Label ───────────────────────────────────────────────────────────────
      visualization_msgs::msg::Marker label;
      label.header   = header;
      label.ns       = "bottle_labels";
      label.id       = 100 + slot_id;
      label.lifetime = rclcpp::Duration(0, 0);
      label.action   = filled ? visualization_msgs::msg::Marker::ADD
                              : visualization_msgs::msg::Marker::DELETE;
      if (filled) {
        label.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        label.pose.position.x    = cx;
        label.pose.position.y    = cy;
        label.pose.position.z    = box_z - box_sz * 0.5 - 0.02;
        label.pose.orientation.w = 1.0;
        label.scale.z  = 0.03;
        label.text     = slot.name;
        label.color.r  = label.color.g = label.color.b = 1.0f;
        label.color.a  = 1.0f;
      }
      array.markers.push_back(label);
    }
  }

  markers_pub_->publish(array);
}

// ---------------------------------------------------------------------------
// logCrateGrid — ASCII matrix of all slots, printed ~1 Hz
// ---------------------------------------------------------------------------
void BottleDetector::logCrateGrid(
  const std::vector<std::vector<SlotGridInfo>> & grid,
  std::size_t filled) const
{
  auto bar = [](float pct) -> std::string {
    int f = static_cast<int>(
      std::round(std::clamp(pct, 0.0f, 100.0f) / 100.0f * 5));
    std::string s = "[";
    for (int i = 0; i < 5; ++i) s += (i < f) ? '#' : '.';
    return s + "]";
  };

  auto fit = [](std::string s, int w) {
    if (static_cast<int>(s.size()) > w) s.resize(w);
    s.resize(w, ' ');
    return s;
  };

  const int CW = 18;
  std::string hline = "+";
  for (int c = 0; c < crate_cols_; ++c) hline += std::string(CW, '-') + "+";

  // Build all lines into a vector so we know the exact line count.
  std::vector<std::string> lines;

  char hdr[64];
  snprintf(hdr, sizeof(hdr), "=== crate  %zu / %d filled ===",
    filled, crate_rows_ * crate_cols_);
  lines.push_back(hdr);
  lines.push_back(hline);

  for (int row = 0; row < crate_rows_; ++row) {
    std::string l1, l2, l3, l4;
    l1 = l2 = l3 = l4 = "|";

    for (int col = 0; col < crate_cols_; ++col) {
      const auto & s = grid[row][col];
      char buf[64];

      std::string name = s.name.size() > 11 ? s.name.substr(0, 11) : s.name;
      snprintf(buf, sizeof(buf), "[%d,%d] %-11s", row, col, name.c_str());
      l1 += fit(buf, CW) + "|";

      if (s.hue >= 0.0f) {
        snprintf(buf, sizeof(buf), " %4dpt  hue:%3.0f", s.points, s.hue);
      } else {
        snprintf(buf, sizeof(buf), " %4dpt  hue:---", s.points);
      }
      l2 += fit(buf, CW) + "|";

      snprintf(buf, sizeof(buf), " occ:%s %3.0f%%",
        bar(s.occ_conf).c_str(), s.occ_conf);
      l3 += fit(buf, CW) + "|";

      if (s.col_conf >= 0.0f) {
        snprintf(buf, sizeof(buf), " col:%s %3.0f%%",
          bar(s.col_conf).c_str(), s.col_conf);
      } else {
        snprintf(buf, sizeof(buf), " col: ---");
      }
      l4 += fit(buf, CW) + "|";
    }

    lines.push_back(l1);
    lines.push_back(l2);
    lines.push_back(l3);
    lines.push_back(l4);
    lines.push_back(hline);
  }

  // In-place update: move cursor up to the start of the previous grid and
  // overwrite line-by-line. \033[2K clears the current line before printing.
  // Note: works best when no other logs appear between refreshes.
  if (!first_grid_) {
    printf("\033[%dA", static_cast<int>(lines.size()));
  }
  for (const auto & line : lines) {
    printf("\033[2K%s\n", line.c_str());
  }
  fflush(stdout);

  first_grid_ = false;
}

// ---------------------------------------------------------------------------
// Helper: PassThrough filter on one axis
// ---------------------------------------------------------------------------
static pcl::PointCloud<pcl::PointXYZRGB>::Ptr passThrough(
  const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & in,
  const std::string & field, double lo, double hi)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr out(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PassThrough<pcl::PointXYZRGB> pass;
  pass.setInputCloud(in);
  pass.setFilterFieldName(field);
  pass.setFilterLimits(static_cast<float>(lo), static_cast<float>(hi));
  pass.filter(*out);
  return out;
}

// ---------------------------------------------------------------------------
// detect()
// ---------------------------------------------------------------------------
vision_msgs::msg::Detection3DArray BottleDetector::detect(
  const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
  const std_msgs::msg::Header & header)
{
  vision_msgs::msg::Detection3DArray result;
  result.header = header;

  reloadParameters();

  // ── 0. Camera tilt correction ───────────────────────────────────────────────
  // Rotate the cloud so that Z becomes perpendicular to the crate top surface.
  // With tilt = 0° this is a no-op (identity transform, no copy made).
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr input = cloud;
  if (std::abs(camera_tilt_x_rad_) > 1e-4f || std::abs(camera_tilt_y_rad_) > 1e-4f) {
    Eigen::Affine3f correction = Eigen::Affine3f::Identity();
    // Apply Y rotation first, then X (extrinsic convention)
    correction.prerotate(
      Eigen::AngleAxisf(-camera_tilt_y_rad_, Eigen::Vector3f::UnitY()));
    correction.prerotate(
      Eigen::AngleAxisf(-camera_tilt_x_rad_, Eigen::Vector3f::UnitX()));
    input.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::transformPointCloud(*cloud, *input, correction);
  }

  // ── 1. Filter to crate bounding box ────────────────────────────────────────
  auto roi = passThrough(input, "z", crate_z_near_, crate_z_far_);
  roi       = passThrough(roi,   "x", crate_x_min_,  crate_x_max_);
  roi       = passThrough(roi,   "y", crate_y_min_,  crate_y_max_);

  // ── 2. Publish filtered cloud for RViz2 ────────────────────────────────────
  sensor_msgs::msg::PointCloud2 cloud_msg;
  pcl::toROSMsg(*roi, cloud_msg);
  cloud_msg.header = header;
  crate_cloud_pub_->publish(cloud_msg);

  if (roi->empty()) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
      "No points in crate ROI — check crate_z_near/far, crate_x/y_min/max");
    return result;
  }

  // ── 3. Voxel grid downsample (5 mm voxels) ─────────────────────────────────
  if (roi->size() > 500000) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
      "ROI has %zu points — too large. Narrow the bounding box first.", roi->size());
    return result;
  }

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsampled(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::VoxelGrid<pcl::PointXYZRGB> vg;
  vg.setInputCloud(roi);
  vg.setLeafSize(0.005f, 0.005f, 0.005f);
  vg.filter(*downsampled);

  // ── 4. One pass over points: count, collect hues, and colour by slot ───────
  double slot_w = (crate_x_max_ - crate_x_min_) / crate_cols_;
  double slot_d = (crate_y_max_ - crate_y_min_) / crate_rows_;
  double x0     = crate_x_min_;
  double y0     = crate_y_min_;

  // Slot-coloured debug cloud: each slot gets a distinct hue so you can verify
  // the grid boundaries in RViz2 (/james/slot_cloud).
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr slot_cloud(
    new pcl::PointCloud<pcl::PointXYZRGB>);

  struct SlotData {
    int count = 0;
    std::vector<float> hues;  // hues of chromatic (non-grey) pixels
  };
  std::vector<std::vector<SlotData>> slots(crate_rows_,
                                           std::vector<SlotData>(crate_cols_));

  for (const auto & pt : downsampled->points) {
    int col = static_cast<int>((pt.x - x0) / slot_w);
    int row = static_cast<int>((pt.y - y0) / slot_d);
    if (col < 0 || col >= crate_cols_ || row < 0 || row >= crate_rows_) continue;

    slots[row][col].count++;

    float h, s, v;
    rgbToHsv(pt.r, pt.g, pt.b, h, s, v);
    if (s >= cap_saturation_min_ && v >= cap_value_min_) {
      slots[row][col].hues.push_back(h);
    }

    // Colour this point by slot index for the debug cloud
    int slot_idx = row * crate_cols_ + col;
    float slot_hue = static_cast<float>(slot_idx) * 360.0f / (crate_rows_ * crate_cols_);
    auto slot_color = hueToRgba(slot_hue, 1.0f);
    pcl::PointXYZRGB colored = pt;
    colored.r = static_cast<uint8_t>(slot_color.r * 255);
    colored.g = static_cast<uint8_t>(slot_color.g * 255);
    colored.b = static_cast<uint8_t>(slot_color.b * 255);
    slot_cloud->push_back(colored);
  }

  // Publish slot-coloured cloud
  sensor_msgs::msg::PointCloud2 slot_msg;
  pcl::toROSMsg(*slot_cloud, slot_msg);
  slot_msg.header = header;
  slot_cloud_pub_->publish(slot_msg);

  // ── 5. Build detections + grid info for all slots ─────────────────────────
  std::vector<std::vector<SlotGridInfo>> grid(
    crate_rows_, std::vector<SlotGridInfo>(crate_cols_));

  for (int row = 0; row < crate_rows_; ++row) {
    for (int col = 0; col < crate_cols_; ++col) {
      const auto & slot = slots[row][col];
      const float occ_conf = std::min(
        100.0f, slot.count * 100.0f / (min_points_per_slot_ * 5.0f));

      if (slot.count < min_points_per_slot_) {
        grid[row][col] = {"empty", slot.count, -1.0f, occ_conf, -1.0f};
        continue;
      }

      float out_hue = -1.0f, out_col_conf = -1.0f;
      const std::string cap_name = classifyHue(slot.hues, out_hue, out_col_conf);
      grid[row][col] = {cap_name, slot.count, out_hue, occ_conf, out_col_conf};

      vision_msgs::msg::Detection3D det;
      det.header = header;

      vision_msgs::msg::ObjectHypothesisWithPose hyp;
      hyp.hypothesis.class_id = cap_name;
      hyp.hypothesis.score    = std::min(
        1.0, static_cast<double>(slot.count) / (min_points_per_slot_ * 5.0));
      hyp.pose.pose.position.x = x0 + (col + 0.5) * slot_w;
      hyp.pose.pose.position.y = y0 + (row + 0.5) * slot_d;
      hyp.pose.pose.position.z = crate_z_near_;
      hyp.pose.pose.orientation.w = 1.0;
      det.results.push_back(hyp);
      det.bbox.center  = hyp.pose.pose;
      det.bbox.size.x  = slot_w;
      det.bbox.size.y  = slot_d;
      det.bbox.size.z  = crate_z_far_ - crate_z_near_;
      result.detections.push_back(det);
    }
  }

  // Log ASCII grid at configured rate (default 1 Hz)
  const double min_interval = terminal_update_hz_ > 0.0
    ? 1.0 / terminal_update_hz_ : 1.0;
  static rclcpp::Time last_grid_log(0, 0, RCL_ROS_TIME);
  auto now = get_clock()->now();
  if ((now - last_grid_log).seconds() >= min_interval) {
    logCrateGrid(grid, result.detections.size());
    last_grid_log = now;
  }

  publishMarkers(grid, header);
  return result;
}

}  // namespace james_perception
