// Copyright 2025 Abdelmoutalib DOUADI - MIVIA Lab, UNISA
// RadarToPointCloudNode — RadarTracks → PointCloud2

#include "ars408_ros/radar_pointcloud_node.hpp"
#include <rclcpp_components/register_node_macro.hpp>
#include <cmath>

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
    "RadarToPointCloud started — frame='%s'  min_rcs=%.1f dBm²",
    output_frame_.c_str(), min_rcs_);
}

void RadarToPointCloudNode::TracksCallback(
  const radar_msgs::msg::RadarTracks::SharedPtr msg)
{
  if (msg->tracks.empty()) return;

  const size_t n = msg->tracks.size();

  // Build PointCloud2
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.stamp    = msg->header.stamp;
  cloud.header.frame_id = output_frame_.empty() ? msg->header.frame_id : output_frame_;
  cloud.height   = 1;
  cloud.width    = static_cast<uint32_t>(n);
  cloud.is_dense = true;
  cloud.is_bigendian = false;

  // Fields: x y z velocity_x velocity_y range azimuth
  sensor_msgs::PointCloud2Modifier mod(cloud);
  mod.setPointCloud2Fields(
    7,
    "x",          1, sensor_msgs::msg::PointField::FLOAT32,
    "y",          1, sensor_msgs::msg::PointField::FLOAT32,
    "z",          1, sensor_msgs::msg::PointField::FLOAT32,
    "velocity_x", 1, sensor_msgs::msg::PointField::FLOAT32,
    "velocity_y", 1, sensor_msgs::msg::PointField::FLOAT32,
    "range",      1, sensor_msgs::msg::PointField::FLOAT32,
    "azimuth",    1, sensor_msgs::msg::PointField::FLOAT32
  );
  mod.resize(n);

  sensor_msgs::PointCloud2Iterator<float> ix(cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> iy(cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> iz(cloud, "z");
  sensor_msgs::PointCloud2Iterator<float> ivx(cloud, "velocity_x");
  sensor_msgs::PointCloud2Iterator<float> ivy(cloud, "velocity_y");
  sensor_msgs::PointCloud2Iterator<float> ir(cloud, "range");
  sensor_msgs::PointCloud2Iterator<float> iaz(cloud, "azimuth");

  for (const auto & t : msg->tracks) {
    const float x  = static_cast<float>(t.position.x);
    const float y  = static_cast<float>(t.position.y);
    const float vx = static_cast<float>(t.velocity.x);
    const float vy = static_cast<float>(t.velocity.y);

    *ix  = x;
    *iy  = y;
    *iz  = 0.0f;
    *ivx = vx;
    *ivy = vy;
    *ir  = std::hypot(x, y);
    *iaz = std::atan2(y, x);

    ++ix; ++iy; ++iz; ++ivx; ++ivy; ++ir; ++iaz;
  }

  pub_cloud_->publish(cloud);

  RCLCPP_INFO(
    this->get_logger(),
    "PointCloud2 published: %zu points", n);
}

RCLCPP_COMPONENTS_REGISTER_NODE(RadarToPointCloudNode)
