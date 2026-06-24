// Copyright 2021 Perception Engine, Inc. All rights reserved.
// Modified and improved by Abdelmoutalib DOUADI - MIVIA Lab, UNISA (2025)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef ARS408_ROS__ARS408_OBJECT_HPP_
#define ARS408_ROS__ARS408_OBJECT_HPP_

#include "ars408_ros/ars408_commands.hpp"

#include <cmath>
#include <sstream>
#include <string>

namespace ars408
{

class RadarObject
{
public:
  // ── Identity ────────────────────────────────────────────────────────────
  uint16_t sequence_id{0};   ///< MeasurementCounter of the cycle this object belongs to
  uint8_t  id{0};            ///< Object ID [0..255], stable across cycles (tracker output)

  // ── Position (Obj_1_General) ─────────────────────────────────────────────
  float distance_long_x{0.0f};  ///< Longitudinal distance X  [m]   range: [-500, +1138.2]
  float distance_lat_y{0.0f};   ///< Lateral distance      Y  [m]   range: [-204.6, +204.8]

  // ── Velocity (Obj_1_General) ─────────────────────────────────────────────
  float speed_long_x{0.0f};     ///< Relative longitudinal velocity [m/s]  range: [-128, +127.75]
  float speed_lat_y{0.0f};      ///< Relative lateral velocity      [m/s]  range: [-64,  +63.75]

  // ── Dynamics (Obj_1_General) ─────────────────────────────────────────────
  ars408::Obj_1_General::DynamicProperty dynamic_property{
    ars408::Obj_1_General::DynamicProperty::UNKNOWN};

  // ── Signal strength (Obj_1_General) ──────────────────────────────────────
  float rcs{0.0f};              ///< Radar cross section [dBm²]  range: [-64, +63.5]

  // ── Derived geometry ──────────────────────────────────────────────────────
  float range{0.0f};            ///< Radial range  = sqrt(x²+y²)  [m]
  float azimuth{0.0f};          ///< Azimuth angle = atan2(y,x)   [rad]

  // ── Quality (Obj_2_Quality) — filled when SendQuality=1 ──────────────────
  float probability_existence{0.0f};          ///< Existence probability [0..1]
  float dist_long_rms{-1.0f};                 ///< Longitudinal distance std dev [m], -1=invalid
  float dist_lat_rms{-1.0f};                  ///< Lateral distance std dev       [m], -1=invalid
  float speed_long_rms{-1.0f};               ///< Longitudinal velocity std dev [m/s]
  float speed_lat_rms{-1.0f};               ///< Lateral velocity std dev      [m/s]
  uint8_t meas_state{0};                      ///< 0=Deleted 1=New 2=Measured 3=Predicted …

  // ── Extended (Obj_3_Extended) — filled when SendExtInfo=1 ────────────────
  ars408::Obj_3_Extended::ObjectClassProperty object_class{
    ars408::Obj_3_Extended::ObjectClassProperty::POINT};
  float rel_acceleration_long_x{0.0f};        ///< Relative longitudinal acceleration [m/s²]
  float rel_acceleration_lat_y{0.0f};         ///< Relative lateral acceleration       [m/s²]
  float orientation_angle{0.0f};              ///< Orientation angle [deg]  range: [-180, +229.2]
  float length{0.0f};                         ///< Object length [m]
  float width{0.0f};                          ///< Object width  [m]

  // ── String helpers ────────────────────────────────────────────────────────
  std::string DynamicPropertyToString() const
  {
    switch (dynamic_property) {
      case ars408::Obj_1_General::DynamicProperty::MOVING:        return "MOVING";
      case ars408::Obj_1_General::DynamicProperty::STATIONARY:    return "STATIONARY";
      case ars408::Obj_1_General::DynamicProperty::ONCOMING:      return "ONCOMING";
      case ars408::Obj_1_General::DynamicProperty::CROSSING_LEFT: return "CROSSING_LEFT";
      case ars408::Obj_1_General::DynamicProperty::CROSSING_RIGHT:return "CROSSING_RIGHT";
      case ars408::Obj_1_General::DynamicProperty::STOPPED:       return "STOPPED";
      default:                                                     return "UNKNOWN";
    }
  }

  std::string ObjectClassToString() const
  {
    switch (object_class) {
      case ars408::Obj_3_Extended::ObjectClassProperty::CAR:         return "CAR";
      case ars408::Obj_3_Extended::ObjectClassProperty::TRUCK:       return "TRUCK";
      case ars408::Obj_3_Extended::ObjectClassProperty::MOTORCYCLE:  return "MOTORCYCLE";
      case ars408::Obj_3_Extended::ObjectClassProperty::BICYCLE:     return "BICYCLE";
      case ars408::Obj_3_Extended::ObjectClassProperty::WIDE:        return "WIDE";
      case ars408::Obj_3_Extended::ObjectClassProperty::POINT:       return "POINT";
      default:                                                         return "PEDESTRIAN";
    }
  }

  std::string ToString() const
  {
    std::ostringstream s;
    s << "ID=" << static_cast<int>(id)
      << " seq=" << sequence_id
      << " X=" << distance_long_x << "m"
      << " Y=" << distance_lat_y << "m"
      << " Vx=" << speed_long_x << "m/s"
      << " Vy=" << speed_lat_y << "m/s"
      << " R=" << range << "m"
      << " Az=" << (azimuth * 180.0f / 3.14159265f) << "deg"
      << " RCS=" << rcs << "dBm2"
      << " Dyn=" << DynamicPropertyToString()
      << " Class=" << ObjectClassToString()
      << " PExist=" << probability_existence
      << " L=" << length << "m W=" << width << "m";
    return s.str();
  }
};

}  // namespace ars408

#endif  // ARS408_ROS__ARS408_OBJECT_HPP_
