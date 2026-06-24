// Copyright 2025 Abdelmoutalib DOUADI - MIVIA Lab, UNISA
//
// Licensed under the Apache License, Version 2.0

#ifndef ARS408_ROS__RADAR_POINTCLOUD_NODE_HPP_
#define ARS408_ROS__RADAR_POINTCLOUD_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <radar_msgs/msg/radar_tracks.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

// ─────────────────────────────────────────────────────────────────────────────
//  RadarToPointCloudNode
//
//  Converts radar_msgs/RadarTracks → sensor_msgs/PointCloud2
//
//  PointCloud2 fields per point :
//    x        [float32]  longitudinal distance  [m]
//    y        [float32]  lateral distance       [m]
//    z        [float32]  0.0 (radar is 2D)      [m]
//    velocity [float32]  radial velocity Vx     [m/s]
//    rcs      [float32]  radar cross section    [dBm²]
//    id       [uint16]   object ID              [0..255]
//
//  Compatible with RViz2 PointCloud2 display and Autoware
//  sensing/radar/pointcloud topic convention.
// ─────────────────────────────────────────────────────────────────────────────

class RadarToPointCloudNode : public rclcpp::Node
{
public:
  explicit RadarToPointCloudNode(const rclcpp::NodeOptions & options);

private:
  void TracksCallback(const radar_msgs::msg::RadarTracks::SharedPtr msg);

  rclcpp::Subscription<radar_msgs::msg::RadarTracks>::SharedPtr sub_tracks_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr   pub_cloud_;

  std::string output_frame_;
  float       min_rcs_;       // filter: ignore points below this RCS
  float       min_prob_;      // filter: ignore objects below this probability
};

#endif  // ARS408_ROS__RADAR_POINTCLOUD_NODE_HPP_
