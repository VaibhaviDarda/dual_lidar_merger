// =============================================================================
//  dual_lidar_merger_node.cpp
//  Merges two 2-D LaserScan streams from YDLidar X2 units mounted on the same
//  horizontal plane.
//
//  Features
//  ────────
//  • TF2-aware: each scan is transformed into target_frame before merging
//  • Configurable overlap strategy: nearest / farthest / average
//  • Body/chassis filter: drops points inside a configurable XY box expressed
//    in body_filter_frame (default base_link).  Applied per-point during
//    projection — zero extra topics, zero extra latency.
//  • Per-sector angle masking in the merged output (e.g. mounting blind spots)
//  • Diagnostics on /merger/diagnostics every 5 s
//  • Single output topic /scan_merged (configurable)
//
//  Subscribed topics
//  ─────────────────
//  /scan   — LaserScan from LiDAR 1  (frame: laser_frame)
//  /scan2  — LaserScan from LiDAR 2  (frame: laser_frame2)
//
//  Published topics
//  ────────────────
//  /scan_merged        — merged + body-filtered LaserScan
//  /merger/diagnostics — std_msgs/String with run-time stats
// =============================================================================

#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>
#include <mutex>
#include <algorithm>
#include <cinttypes>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/exceptions.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"

// ── Helper ─────────────────────────────────────────────────────────────────
static inline float normalise_angle(float a)
{
  while (a >  M_PI) a -= 2.f * M_PI;
  while (a < -M_PI) a += 2.f * M_PI;
  return a;
}

// ── Node ───────────────────────────────────────────────────────────────────
class DualLidarMerger : public rclcpp::Node
{
public:
  explicit DualLidarMerger(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("dual_lidar_merger", options)
  {
    // ── Parameters ────────────────────────────────────────────────────────

    // Topics
    declare_parameter("scan1_topic",  std::string("/scan"));
    declare_parameter("scan2_topic",  std::string("/scan2"));
    declare_parameter("output_topic", std::string("/scan_merged"));

    // Frames
    // target_frame  — all scan points projected into this frame for merging
    // output_frame  — frame_id on /scan_merged (defaults to target_frame)
    declare_parameter("target_frame", std::string("base_link"));
    declare_parameter("output_frame", std::string(""));
    declare_parameter("scan1_frame",  std::string("laser_frame"));
    declare_parameter("scan2_frame",  std::string("laser_frame2"));

    // Output scan geometry
    declare_parameter("angle_min",      -M_PI);
    declare_parameter("angle_max",       M_PI);
    declare_parameter("angle_increment", 0.00872664626);  // ~0.5 deg
    declare_parameter("range_min",       0.12);
    declare_parameter("range_max",       8.0);

    // Overlap strategy when both LiDARs see the same bin:
    //   "nearest"  — keep closest reading  (safest for obstacle avoidance)
    //   "farthest" — keep farthest reading
    //   "average"  — arithmetic mean
    declare_parameter("overlap_mode", std::string("nearest"));

    // Drop cached scans older than this many seconds
    declare_parameter("max_scan_age", 0.5);

    // Output-space angle mask (degrees).  Flat list of [min,max] pairs.
    // Pass [0.0] (sentinel) to disable — bare [] causes ROS 2 type crash.
    declare_parameter("mask_angles", std::vector<double>{0.0});

    // Fill value for empty / filtered output bins
    declare_parameter("invalid_range_is_inf", true);

    // ── Body / chassis filter ─────────────────────────────────────────────
    //
    // Removes points that fall inside the robot body box BEFORE they reach
    // the merge bins.  This catches standoffs, wires, cables, and chassis
    // edges seen by either LiDAR.  The filter runs in-line during projection
    // so there is no extra topic, publisher, or copy of the data.
    //
    // The box is defined in body_filter_frame (default: base_link).
    // x_min / x_max : forward axis  (ROS +X = forward)
    // y_min / y_max : lateral axis  (ROS +Y = left)
    // padding       : extra metres added to every side — lets you tune
    //                 clearance without re-entering the exact URDF numbers.
    //
    // Typical starting values for a 432 x 288 mm chassis:
    //   x_min: -0.216   x_max: 0.216
    //   y_min: -0.144   y_max: 0.144
    //   padding: 0.02   (2 cm safety margin)
    declare_parameter("body_filter_enabled", false);
    declare_parameter("body_filter_frame",   std::string("base_link"));
    declare_parameter("body_filter_x_min",  -0.216);
    declare_parameter("body_filter_x_max",   0.216);
    declare_parameter("body_filter_y_min",  -0.144);
    declare_parameter("body_filter_y_max",   0.144);
    declare_parameter("body_filter_padding", 0.02);

    // Diagnostics topic
    declare_parameter("publish_diagnostics", true);

    // ── Read all params ───────────────────────────────────────────────────
    scan1_topic_  = get_parameter("scan1_topic").as_string();
    scan2_topic_  = get_parameter("scan2_topic").as_string();
    output_topic_ = get_parameter("output_topic").as_string();
    target_frame_ = get_parameter("target_frame").as_string();
    output_frame_ = get_parameter("output_frame").as_string();
    if (output_frame_.empty()) output_frame_ = target_frame_;
    scan1_frame_  = get_parameter("scan1_frame").as_string();
    scan2_frame_  = get_parameter("scan2_frame").as_string();

    angle_min_ = static_cast<float>(get_parameter("angle_min").as_double());
    angle_max_ = static_cast<float>(get_parameter("angle_max").as_double());
    angle_inc_ = static_cast<float>(get_parameter("angle_increment").as_double());
    range_min_ = static_cast<float>(get_parameter("range_min").as_double());
    range_max_ = static_cast<float>(get_parameter("range_max").as_double());

    overlap_mode_   = get_parameter("overlap_mode").as_string();
    max_scan_age_   = get_parameter("max_scan_age").as_double();
    invalid_is_inf_ = get_parameter("invalid_range_is_inf").as_bool();

    auto mask_raw = get_parameter("mask_angles").as_double_array();
    for (size_t i = 0; i + 1 < mask_raw.size(); i += 2) {
      mask_angles_.push_back({
        static_cast<float>(mask_raw[i]   * M_PI / 180.0),
        static_cast<float>(mask_raw[i+1] * M_PI / 180.0)
      });
    }

    body_filter_enabled_ = get_parameter("body_filter_enabled").as_bool();
    body_filter_frame_   = get_parameter("body_filter_frame").as_string();
    float pad = static_cast<float>(get_parameter("body_filter_padding").as_double());
    body_x_min_ = static_cast<float>(get_parameter("body_filter_x_min").as_double()) - pad;
    body_x_max_ = static_cast<float>(get_parameter("body_filter_x_max").as_double()) + pad;
    body_y_min_ = static_cast<float>(get_parameter("body_filter_y_min").as_double()) - pad;
    body_y_max_ = static_cast<float>(get_parameter("body_filter_y_max").as_double()) + pad;

    pub_diag_ = get_parameter("publish_diagnostics").as_bool();

    num_bins_ = static_cast<int>(
      std::round((angle_max_ - angle_min_) / angle_inc_)) + 1;

    RCLCPP_INFO(get_logger(),
      "DualLidarMerger starting\n"
      "  in:  %s  %s  ->  out: %s\n"
      "  target_frame: %s   output_frame: %s   bins: %d   overlap: %s\n"
      "  body_filter: %s  frame: %s  x[%.3f, %.3f]  y[%.3f, %.3f]  pad=%.3f m",
      scan1_topic_.c_str(), scan2_topic_.c_str(), output_topic_.c_str(),
      target_frame_.c_str(), output_frame_.c_str(), num_bins_, overlap_mode_.c_str(),
      body_filter_enabled_ ? "ON " : "OFF", body_filter_frame_.c_str(),
      body_x_min_, body_x_max_, body_y_min_, body_y_max_, pad);

    // ── TF2 ───────────────────────────────────────────────────────────────
    tf_buffer_   = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // ── Publishers ────────────────────────────────────────────────────────
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();
    pub_scan_ = create_publisher<sensor_msgs::msg::LaserScan>(output_topic_, qos);

    if (pub_diag_) {
      pub_diag_msg_ = create_publisher<std_msgs::msg::String>(
        "/merger/diagnostics", rclcpp::SystemDefaultsQoS());
    }

    // ── Subscribers ───────────────────────────────────────────────────────
    // Sensor-data (best-effort) QoS on the inputs: compatible with drivers that
    // publish either reliable or best-effort.
    const auto sub_qos = rclcpp::SensorDataQoS();

    sub1_ = create_subscription<sensor_msgs::msg::LaserScan>(
      scan1_topic_, sub_qos,
      [this](sensor_msgs::msg::LaserScan::SharedPtr msg) {
        std::lock_guard<std::mutex> lk(mutex_);
        scan1_ = msg;
        scan1_count_++;
        try_merge();
      });

    sub2_ = create_subscription<sensor_msgs::msg::LaserScan>(
      scan2_topic_, sub_qos,
      [this](sensor_msgs::msg::LaserScan::SharedPtr msg) {
        std::lock_guard<std::mutex> lk(mutex_);
        scan2_ = msg;
        scan2_count_++;
        try_merge();
      });

    if (pub_diag_) {
      diag_timer_ = create_wall_timer(
        std::chrono::seconds(5),
        [this]() { publish_diagnostics(); });
    }
  }

private:
  // ── Types ─────────────────────────────────────────────────────────────
  struct AnglePair { float min; float max; };

  // ── State ──────────────────────────────────────────────────────────────
  sensor_msgs::msg::LaserScan::SharedPtr scan1_, scan2_;
  std::mutex mutex_;

  uint64_t scan1_count_         = 0;
  uint64_t scan2_count_         = 0;
  uint64_t merge_count_         = 0;
  uint64_t drop_count_          = 0;
  uint64_t body_filtered_count_ = 0;   // cumulative points removed by body filter

  // ── Params ─────────────────────────────────────────────────────────────
  std::string scan1_topic_, scan2_topic_, output_topic_;
  std::string target_frame_, output_frame_, scan1_frame_, scan2_frame_;
  float       angle_min_, angle_max_, angle_inc_;
  float       range_min_, range_max_;
  std::string overlap_mode_;
  double      max_scan_age_;
  bool        invalid_is_inf_;
  bool        pub_diag_;
  int         num_bins_;
  std::vector<AnglePair> mask_angles_;

  bool        body_filter_enabled_;
  std::string body_filter_frame_;
  float       body_x_min_, body_x_max_;   // already include padding
  float       body_y_min_, body_y_max_;

  // ── ROS objects ────────────────────────────────────────────────────────
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_scan_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr       pub_diag_msg_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub1_, sub2_;
  rclcpp::TimerBase::SharedPtr diag_timer_;
  std::shared_ptr<tf2_ros::Buffer>            tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // ── Helpers ────────────────────────────────────────────────────────────

  bool scan_is_fresh(const sensor_msgs::msg::LaserScan::SharedPtr & s) const
  {
    double age = (now() - s->header.stamp).seconds();
    return age >= 0.0 && age < max_scan_age_;
  }

  bool is_masked(float angle_rad) const
  {
    for (const auto & p : mask_angles_) {
      if (angle_rad >= p.min && angle_rad <= p.max) return true;
    }
    return false;
  }

  // Check if point (px, py) — expressed in target_frame — falls inside the
  // body box expressed in body_filter_frame.
  //
  // bf_dx, bf_dy : origin of body_filter_frame in target_frame coords
  // bf_cos, bf_sin: yaw rotation from target_frame to body_filter_frame
  //
  // When both frames are the same, these are 0, 0, 1, 0 (identity).
  inline bool in_body_box(float px, float py,
                           float bf_dx, float bf_dy,
                           float bf_cos, float bf_sin) const
  {
    // Translate then rotate into body_filter_frame
    float dx = px - bf_dx;
    float dy = py - bf_dy;
    float bx =  bf_cos * dx + bf_sin * dy;
    float by = -bf_sin * dx + bf_cos * dy;
    return (bx >= body_x_min_ && bx <= body_x_max_ &&
            by >= body_y_min_ && by <= body_y_max_);
  }

  // ── project_scan ─────────────────────────────────────────────────────
  // Projects one LaserScan into target_frame.
  // Body filter is applied per-point right after projection.
  // Valid points are accumulated into the merge bins.
  void project_scan(
    const sensor_msgs::msg::LaserScan::SharedPtr & scan,
    std::vector<float> & bin_min,
    std::vector<float> & bin_max,
    std::vector<float> & bin_sum,
    std::vector<float> & bin_int_sum,
    std::vector<int>   & bin_count)
  {
    // ── 1. Look up scan_frame → target_frame transform ────────────────────
    geometry_msgs::msg::TransformStamped tf_to_target;
    try {
      tf_to_target = tf_buffer_->lookupTransform(
        target_frame_, scan->header.frame_id,
        scan->header.stamp,
        rclcpp::Duration::from_seconds(0.1));
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "TF lookup failed (%s -> %s): %s",
        scan->header.frame_id.c_str(), target_frame_.c_str(), ex.what());
      drop_count_++;
      return;
    }

    const auto & q = tf_to_target.transform.rotation;
    const float scan_yaw = std::atan2(
      2.f * (q.w * q.z + q.x * q.y),
      1.f - 2.f * (q.y * q.y + q.z * q.z));
    const float s_cos = std::cos(scan_yaw);
    const float s_sin = std::sin(scan_yaw);
    float tx = static_cast<float>(tf_to_target.transform.translation.x);
    float ty = static_cast<float>(tf_to_target.transform.translation.y);

    // ── 2. Pre-compute body_filter_frame → target_frame offset ───────────
    // Only needed when body filter is on AND the frames differ.
    // For the common case (body_filter_frame == target_frame == base_link)
    // this block is skipped and identity values are used.
    float bf_dx = 0.f, bf_dy = 0.f, bf_cos = 1.f, bf_sin = 0.f;
    if (body_filter_enabled_ && body_filter_frame_ != target_frame_) {
      try {
        auto tf_bf = tf_buffer_->lookupTransform(
          target_frame_, body_filter_frame_,
          scan->header.stamp,
          rclcpp::Duration::from_seconds(0.1));
        bf_dx = static_cast<float>(tf_bf.transform.translation.x);
        bf_dy = static_cast<float>(tf_bf.transform.translation.y);
        const auto & qb = tf_bf.transform.rotation;
        float bf_yaw = std::atan2(
          2.f * (qb.w * qb.z + qb.x * qb.y),
          1.f - 2.f * (qb.y * qb.y + qb.z * qb.z));
        bf_cos = std::cos(bf_yaw);
        bf_sin = std::sin(bf_yaw);
      } catch (const tf2::TransformException & ex) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
          "Body filter TF lookup failed (%s -> %s): %s  -- filter skipped",
          target_frame_.c_str(), body_filter_frame_.c_str(), ex.what());
        // Proceed without body filter for this scan rather than dropping it
      }
    }

    // ── 3. Per-point loop ─────────────────────────────────────────────────
    for (size_t i = 0; i < scan->ranges.size(); ++i) {
      float r = scan->ranges[i];
      if (!std::isfinite(r) || r < scan->range_min || r > scan->range_max) continue;

      float angle_local = scan->angle_min + static_cast<float>(i) * scan->angle_increment;

      // Point in the LiDAR's own frame
      const float lx = r * std::cos(angle_local);
      const float ly = r * std::sin(angle_local);

      // Rotate by the mount yaw, then translate into target_frame
      // (2-D: LiDARs are coplanar, z ignored)
      float px = s_cos * lx - s_sin * ly + tx;
      float py = s_sin * lx + s_cos * ly + ty;

      // Body filter — discard points inside the chassis box
      if (body_filter_enabled_) {
        if (in_body_box(px, py, bf_dx, bf_dy, bf_cos, bf_sin)) {
          body_filtered_count_++;
          continue;
        }
      }

      // Convert back to polar in target_frame for binning
      float r_global     = std::sqrt(px * px + py * py);
      float angle_merged = std::atan2(py, px);

      if (r_global < range_min_ || r_global > range_max_) continue;
      if (angle_merged < angle_min_ || angle_merged > angle_max_) continue;

      int idx = static_cast<int>(
        std::round((angle_merged - angle_min_) / angle_inc_));
      if (idx < 0 || idx >= num_bins_) continue;

      float intensity = (i < scan->intensities.size()) ? scan->intensities[i] : 0.f;

      bin_min[idx]      = std::min(bin_min[idx], r_global);
      bin_max[idx]      = std::max(bin_max[idx], r_global);
      bin_sum[idx]     += r_global;
      bin_int_sum[idx] += intensity;
      bin_count[idx]++;
    }
  }

  // ── try_merge ─────────────────────────────────────────────────────────
  // Called on every incoming scan.  Single pipeline:
  //   project + body-filter scan1
  //   project + body-filter scan2
  //   merge bins with chosen overlap strategy
  //   apply output-space angle mask
  //   publish /scan_merged
  void try_merge()
  {
    if (!scan1_ || !scan2_) return;
    if (!scan_is_fresh(scan1_) || !scan_is_fresh(scan2_)) {
      drop_count_++;
      return;
    }

    const float NO_RANGE = std::numeric_limits<float>::infinity();
    const float BIG      = std::numeric_limits<float>::max();

    std::vector<float> bin_min(num_bins_, BIG);
    std::vector<float> bin_max(num_bins_, 0.f);
    std::vector<float> bin_sum(num_bins_, 0.f);
    std::vector<float> bin_int_sum(num_bins_, 0.f);
    std::vector<int>   bin_count(num_bins_, 0);

    // Both scans are processed in the same pass — body filter applied inside
    project_scan(scan1_, bin_min, bin_max, bin_sum, bin_int_sum, bin_count);
    project_scan(scan2_, bin_min, bin_max, bin_sum, bin_int_sum, bin_count);

    // ── Build output LaserScan ────────────────────────────────────────────
    auto out = std::make_shared<sensor_msgs::msg::LaserScan>();
    out->header.stamp    = now();
    out->header.frame_id = output_frame_;
    out->angle_min       = angle_min_;
    out->angle_max       = angle_max_;
    out->angle_increment = angle_inc_;
    out->range_min       = range_min_;
    out->range_max       = range_max_;
    out->scan_time       = std::max(scan1_->scan_time, scan2_->scan_time);
    out->time_increment  = out->scan_time / static_cast<float>(num_bins_);

    out->ranges.resize(num_bins_, invalid_is_inf_ ? NO_RANGE : 0.f);
    out->intensities.resize(num_bins_, 0.f);

    for (int i = 0; i < num_bins_; ++i) {
      float angle = angle_min_ + static_cast<float>(i) * angle_inc_;

      // Output-space angular mask (mounting-hardware blind spots)
      if (is_masked(angle)) {
        out->ranges[i] = invalid_is_inf_ ? NO_RANGE : 0.f;
        continue;
      }

      if (bin_count[i] == 0) continue;

      if (overlap_mode_ == "nearest") {
        out->ranges[i] = bin_min[i];
      } else if (overlap_mode_ == "farthest") {
        out->ranges[i] = bin_max[i];
      } else {
        out->ranges[i] = bin_sum[i] / static_cast<float>(bin_count[i]);
      }
      out->intensities[i] = bin_int_sum[i] / static_cast<float>(bin_count[i]);
    }

    pub_scan_->publish(*out);
    merge_count_++;
  }

  void publish_diagnostics()
  {
    std::lock_guard<std::mutex> lk(mutex_);
    std_msgs::msg::String msg;
    char buf[320];
    snprintf(buf, sizeof(buf),
      "DualLidarMerger | scan1_rx=%" PRIu64 "  scan2_rx=%" PRIu64
      "  merged=%" PRIu64 "  dropped=%" PRIu64 "  body_filtered_pts=%" PRIu64,
      scan1_count_, scan2_count_, merge_count_,
      drop_count_, body_filtered_count_);
    msg.data = buf;
    pub_diag_msg_->publish(msg);
  }
};

// ── main ───────────────────────────────────────────────────────────────────
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DualLidarMerger>());
  rclcpp::shutdown();
  return 0;
}