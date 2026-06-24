// Copyright 2021 Perception Engine, Inc. All rights reserved.
// Modified and improved by Abdelmoutalib DOUADI - MIVIA Lab, UNISA (2025)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "ars408_ros/ars408_ros_node.hpp"

#include <rclcpp/rclcpp.hpp>

#include <cmath>
#include <string>
#include <unordered_map>

PeContinentalArs408Node::PeContinentalArs408Node(const rclcpp::NodeOptions & node_options)
: Node("ars408_node", node_options)
{
  GenerateUUIDTable();
  Run();
}

void PeContinentalArs408Node::CanFrameCallback(
  const can_msgs::msg::Frame::SharedPtr can_msg)
{
  if (!can_msg->data.empty()) {
    can_data_ = can_msg;
    ars408_driver_.Parse(can_msg->id, can_msg->data, can_msg->dlc);
  }
}

// ─── Autoware semantic class mapping ─────────────────────────────────────────
uint32_t PeContinentalArs408Node::ConvertRadarClassToAwSemanticClass(
  const ars408::Obj_3_Extended::ObjectClassProperty & cls)
{
  switch (cls) {
    case ars408::Obj_3_Extended::CAR:         return 32001;
    case ars408::Obj_3_Extended::TRUCK:       return 32002;
    case ars408::Obj_3_Extended::PEDESTRIAN:  return 32003;  // Autoware: pedestrian
    case ars408::Obj_3_Extended::MOTORCYCLE:  return 32005;
    case ars408::Obj_3_Extended::BICYCLE:     return 32006;
    default:                                  return 32000;  // UNKNOWN
  }
}

// ─── RadarTrack conversion ────────────────────────────────────────────────────
radar_msgs::msg::RadarTrack
PeContinentalArs408Node::ConvertRadarObjectToRadarTrack(const ars408::RadarObject & obj)
{
  radar_msgs::msg::RadarTrack track;
  track.uuid = UUID_table_[obj.id];

  // Position (sensor frame: X=forward, Y=left)
  track.position.x = static_cast<double>(obj.distance_long_x);
  track.position.y = static_cast<double>(obj.distance_lat_y);
  track.position.z = 0.0;

  // Position covariance [x,y,z] diagonal — use RMS if available, else defaults
  track.position_covariance[0] = (obj.dist_long_rms > 0.0f)
    ? static_cast<double>(obj.dist_long_rms * obj.dist_long_rms) : 0.25;
  track.position_covariance[4] = (obj.dist_lat_rms > 0.0f)
    ? static_cast<double>(obj.dist_lat_rms * obj.dist_lat_rms) : 0.25;
  track.position_covariance[8] = 1.0;   // Z unknown

  // Velocity
  track.velocity.x = static_cast<double>(obj.speed_long_x);
  track.velocity.y = static_cast<double>(obj.speed_lat_y);
  track.velocity.z = 0.0;

  // Velocity covariance
  track.velocity_covariance[0] = (obj.speed_long_rms > 0.0f)
    ? static_cast<double>(obj.speed_long_rms * obj.speed_long_rms) : 0.1;
  track.velocity_covariance[4] = (obj.speed_lat_rms > 0.0f)
    ? static_cast<double>(obj.speed_lat_rms * obj.speed_lat_rms) : 0.1;
  track.velocity_covariance[8] = 1.0;

  // Acceleration
  track.acceleration.x = static_cast<double>(obj.rel_acceleration_long_x);
  track.acceleration.y = static_cast<double>(obj.rel_acceleration_lat_y);
  track.acceleration.z = 0.0;

  // Size
  track.size.x = (obj.length > 0.0f) ? static_cast<double>(obj.length) : size_x_;
  track.size.y = (obj.width  > 0.0f) ? static_cast<double>(obj.width)  : size_y_;
  track.size.z = 1.0;

  // Semantic class
  track.classification = ConvertRadarClassToAwSemanticClass(obj.object_class);

  return track;
}

// ─── RadarReturn (scan) conversion ───────────────────────────────────────────
radar_msgs::msg::RadarReturn
PeContinentalArs408Node::ConvertRadarObjectToRadarReturn(const ars408::RadarObject & obj)
{
  radar_msgs::msg::RadarReturn ret;
  ret.range           = static_cast<float>(obj.range);
  ret.azimuth         = static_cast<float>(obj.azimuth);
  ret.elevation       = 0.0f;
  ret.doppler_velocity = (std::abs(std::cos(obj.azimuth)) > 1e-3f)
    ? obj.speed_long_x / std::cos(obj.azimuth) : obj.speed_long_x;
  ret.amplitude       = obj.rcs;
  return ret;
}

// ─── Main objects callback ────────────────────────────────────────────────────
void PeContinentalArs408Node::RadarDetectedObjectsCallback(
  const std::unordered_map<uint8_t, ars408::RadarObject> & detected_objects)
{
  rclcpp::Time stamp = can_data_ ? rclcpp::Time(can_data_->header.stamp) : this->now();

  radar_msgs::msg::RadarTracks tracks_msg;
  tracks_msg.header.frame_id = output_frame_;
  tracks_msg.header.stamp    = stamp;

  radar_msgs::msg::RadarScan scan_msg;
  scan_msg.header.frame_id = output_frame_;
  scan_msg.header.stamp    = stamp;

  for (const auto & [id, obj] : detected_objects) {
    RCLCPP_DEBUG(
      this->get_logger(), "[obj] %s", obj.ToString().c_str());

    if (publish_radar_track_) {
      tracks_msg.tracks.emplace_back(ConvertRadarObjectToRadarTrack(obj));
    }
    if (publish_radar_scan_) {
      scan_msg.returns.emplace_back(ConvertRadarObjectToRadarReturn(obj));
    }
  }

  if (publish_radar_track_) {
    publisher_radar_tracks_->publish(tracks_msg);
  }
  if (publish_radar_scan_) {
    publisher_radar_scan_->publish(scan_msg);
  }
}

// ─── UUID helpers ──────────────────────────────────────────────────────────
unique_identifier_msgs::msg::UUID PeContinentalArs408Node::GenerateRandomUUID()
{
  unique_identifier_msgs::msg::UUID uuid;
  std::mt19937 gen(std::random_device{}());
  std::independent_bits_engine<std::mt19937, 8, uint8_t> bit_eng(gen);
  std::generate(uuid.uuid.begin(), uuid.uuid.end(), bit_eng);
  return uuid;
}

void PeContinentalArs408Node::GenerateUUIDTable()
{
  UUID_table_.reserve(max_radar_id + 1u);
  for (size_t i = 0; i <= max_radar_id; ++i) {
    UUID_table_.emplace_back(GenerateRandomUUID());
  }
}

// ─── Run (parameter declaration + subscriber/publisher setup) ─────────────
void PeContinentalArs408Node::Run()
{
  int sensor_id = this->declare_parameter<int>("sensor_id", 0);
  if (sensor_id < 0 || sensor_id > 7) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Invalid sensor_id=%d, must be [0..7]. Defaulting to 0.", sensor_id);
    sensor_id = 0;
  }
  ars408_driver_.Init(static_cast<uint8_t>(sensor_id));

  output_frame_        = this->declare_parameter<std::string>("output_frame", "ars408");
  publish_radar_track_ = this->declare_parameter<bool>("publish_radar_track", true);
  publish_radar_scan_  = this->declare_parameter<bool>("publish_radar_scan", false);
  sequential_publish_  = this->declare_parameter<bool>("sequential_publish", false);
  size_x_              = this->declare_parameter<double>("size_x", 1.8);
  size_y_              = this->declare_parameter<double>("size_y", 1.8);

  ars408_driver_.RegisterDetectedObjectsCallback(
    std::bind(
      &PeContinentalArs408Node::RadarDetectedObjectsCallback,
      this, std::placeholders::_1),
    sequential_publish_);

  subscription_ = this->create_subscription<can_msgs::msg::Frame>(
    "~/input/frame", 10,
    std::bind(&PeContinentalArs408Node::CanFrameCallback, this, std::placeholders::_1));

  publisher_radar_tracks_ =
    this->create_publisher<radar_msgs::msg::RadarTracks>("~/output/objects", 10);
  publisher_radar_scan_ =
    this->create_publisher<radar_msgs::msg::RadarScan>("~/output/scan", 10);

  RCLCPP_INFO(
    this->get_logger(),
    "ARS408 node started — SensorID=%d  frame='%s'  track=%d  scan=%d",
    sensor_id, output_frame_.c_str(),
    static_cast<int>(publish_radar_track_),
    static_cast<int>(publish_radar_scan_));
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(PeContinentalArs408Node)
