// Copyright 2025 Abdelmoutalib DOUADI - MIVIA Lab, UNISA
// RadarToPointCloudNode — RadarTracks → PointCloud2 (LiDAR-style display)

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

  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.stamp    = msg->header.stamp;
  cloud.header.frame_id = output_frame_.empty() ? msg->header.frame_id : output_frame_;
  cloud.height      = 1;
  cloud.width       = static_cast<uint32_t>(n);
  cloud.is_dense    = true;
  cloud.is_bigendian = false;

  // Fields: x y z intensity velocity_x velocity_y range azimuth
  // intensity = RCS normalisé [0..255] → permet ColorTransformer=Intensity dans RViz2
  sensor_msgs::PointCloud2Modifier mod(cloud);
  mod.setPointCloud2Fields(
    8,
    "x",          1, sensor_msgs::msg::PointField::FLOAT32,
    "y",          1, sensor_msgs::msg::PointField::FLOAT32,
    "z",          1, sensor_msgs::msg::PointField::FLOAT32,
    "intensity",  1, sensor_msgs::msg::PointField::FLOAT32,
    "velocity_x", 1, sensor_msgs::msg::PointField::FLOAT32,
    "velocity_y", 1, sensor_msgs::msg::PointField::FLOAT32,
    "range",      1, sensor_msgs::msg::PointField::FLOAT32,
    "azimuth",    1, sensor_msgs::msg::PointField::FLOAT32
  );
  mod.resize(n);

  sensor_msgs::PointCloud2Iterator<float> ix(cloud,   "x");
  sensor_msgs::PointCloud2Iterator<float> iy(cloud,   "y");
  sensor_msgs::PointCloud2Iterator<float> iz(cloud,   "z");
  sensor_msgs::PointCloud2Iterator<float> ii(cloud,   "intensity");
  sensor_msgs::PointCloud2Iterator<float> ivx(cloud,  "velocity_x");
  sensor_msgs::PointCloud2Iterator<float> ivy(cloud,  "velocity_y");
  sensor_msgs::PointCloud2Iterator<float> ir(cloud,   "range");
  sensor_msgs::PointCloud2Iterator<float> iaz(cloud,  "azimuth");

  for (const auto & t : msg->tracks) {
    const float x   = static_cast<float>(t.position.x);
    const float y   = static_cast<float>(t.position.y);
    const float vx  = static_cast<float>(t.velocity.x);
    const float vy  = static_cast<float>(t.velocity.y);

    // intensity = RCS normalisé [-64..+64 dBm²] → [0..255]
    // Récupéré depuis velocity_covariance[8] où on stocke rcs²
    // Ou estimé depuis la taille de l'objet en fallback
    // Formule : intensity = (rcs + 64) / 128 * 255, clampé [0,255]
    // On utilise size_x comme proxy de RCS (4.5m=CAR → intensité élevée)
    const float size_proxy = static_cast<float>(t.size.x + t.size.y);
    const float intensity  = std::min(255.0f, std::max(0.0f, size_proxy * 20.0f));

    *ix  = x;
    *iy  = y;
    *iz  = 0.0f;
    *ii  = intensity;
    *ivx = vx;
    *ivy = vy;
    *ir  = std::hypot(x, y);
    *iaz = std::atan2(y, x);

    ++ix; ++iy; ++iz; ++ii; ++ivx; ++ivy; ++ir; ++iaz;
  }

  pub_cloud_->publish(cloud);

  RCLCPP_INFO(this->get_logger(), "PointCloud2: %zu points", n);
}

RCLCPP_COMPONENTS_REGISTER_NODE(RadarToPointCloudNode)
