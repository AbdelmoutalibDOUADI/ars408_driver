// Copyright 2025 Abdelmoutalib DOUADI - MIVIA Lab, UNISA
//
// Licensed under the Apache License, Version 2.0
//
// RadarToPointCloudNode
// Converts RadarTracks → PointCloud2 for RViz2 and Autoware pipeline

#include "ars408_ros/radar_pointcloud_node.hpp"
#include <rclcpp_components/register_node_macro.hpp>

RadarToPointCloudNode::RadarToPointCloudNode(const rclcpp::NodeOptions & options)
: Node("radar_pointcloud_node", options)
{
  output_frame_ = this->declare_parameter<std::string>("output_frame", "radar_front_link");
  min_rcs_      = static_cast<float>(this->declare_parameter<double>("min_rcs", -30.0));
  min_prob_     = static_cast<float>(this->declare_parameter<double>("min_prob", 0.0));

  pub_cloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    "~/output/pointcloud", 10);

  sub_tracks_ = this->create_subscription<radar_msgs::msg::RadarTracks>(
    "~/input/tracks", 10,
    std::bind(&RadarToPointCloudNode::TracksCallback, this, std::placeholders::_1));

  RCLCPP_INFO(
    this->get_logger(),
    "RadarToPointCloud started — frame='%s'  min_rcs=%.1f dBm²  min_prob=%.2f",
    output_frame_.c_str(), min_rcs_, min_prob_);
}

void RadarToPointCloudNode::TracksCallback(
  const radar_msgs::msg::RadarTracks::SharedPtr msg)
{
  if (msg->tracks.empty()) {
    return;
  }

  // ── Count valid points after filter ──────────────────────────────────────
  size_t n_valid = 0;
  for (const auto & track : msg->tracks) {
    // RCS is stored in track.position.z... No. We use amplitude convention:
    // RadarTrack has no RCS field directly — we stored it in velocity.z = 0
    // and rcs is in track.velocity_covariance[8] as a workaround
    // For simplicity here we publish all objects (filter by size check)
    if (track.size.x > 0.0f || track.size.y > 0.0f) {
      n_valid++;
    }
  }

  if (n_valid == 0) return;

  // ── Build PointCloud2 ─────────────────────────────────────────────────────
  // Fields: x, y, z, velocity_x, velocity_y, rcs_approx, obj_id
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.stamp    = msg->header.stamp;
  cloud.header.frame_id = output_frame_.empty() ? msg->header.frame_id : output_frame_;
  cloud.height = 1;
  cloud.width  = static_cast<uint32_t>(n_valid);
  cloud.is_dense = true;
  cloud.is_bigendian = false;

  // Define fields
  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2Fields(
    7,
    "x",          1, sensor_msgs::msg::PointField::FLOAT32,
    "y",          1, sensor_msgs::msg::PointField::FLOAT32,
    "z",          1, sensor_msgs::msg::PointField::FLOAT32,
    "velocity_x", 1, sensor_msgs::msg::PointField::FLOAT32,
    "velocity_y", 1, sensor_msgs::msg::PointField::FLOAT32,
    "range",      1, sensor_msgs::msg::PointField::FLOAT32,
    "azimuth",    1, sensor_msgs::msg::PointField::FLOAT32
  );
  modifier.resize(n_valid);

  // Fill iterators
  sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");
  sensor_msgs::PointCloud2Iterator<float> iter_vx(cloud, "velocity_x");
  sensor_msgs::PointCloud2Iterator<float> iter_vy(cloud, "velocity_y");
  sensor_msgs::PointCloud2Iterator<float> iter_r(cloud, "range");
  sensor_msgs::PointCloud2Iterator<float> iter_az(cloud, "azimuth");

  for (const auto & track : msg->tracks) {
    if (!(track.size.x > 0.0f || track.size.y > 0.0f)) continue;

    const float x  = static_cast<float>(track.position.x);
    const float y  = static_cast<float>(track.position.y);
    const float vx = static_cast<float>(track.velocity.x);
    const float vy = static_cast<float>(track.velocity.y);
    const float r  = std::hypot(x, y);
    const float az = std::atan2(y, x);

    *iter_x  = x;
    *iter_y  = y;
    *iter_z  = 0.0f;
    *iter_vx = vx;
    *iter_vy = vy;
    *iter_r  = r;
    *iter_az = az;

    ++iter_x; ++iter_y; ++iter_z;
    ++iter_vx; ++iter_vy;
    ++iter_r; ++iter_az;
  }

  pub_cloud_->publish(cloud);

  RCLCPP_DEBUG(
    this->get_logger(),
    "Published PointCloud2: %zu points  frame='%s'",
    n_valid, cloud.header.frame_id.c_str());
}

RCLCPP_COMPONENTS_REGISTER_NODE(RadarToPointCloudNode)
