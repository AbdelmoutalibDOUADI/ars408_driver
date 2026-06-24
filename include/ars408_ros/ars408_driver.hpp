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

#ifndef ARS408_ROS__ARS408_DRIVER_HPP_
#define ARS408_ROS__ARS408_DRIVER_HPP_

#include "ars408_ros/ars408_commands.hpp"
#include "ars408_ros/ars408_constants.hpp"
#include "ars408_ros/ars408_object.hpp"

#include <functional>
#include <iostream>
#include <string>
#include <unordered_map>
#include <vector>

namespace ars408
{

class Ars408Driver
{
private:
  // ── SensorID and dynamically computed CAN IDs ────────────────────────────
  // Continental formula: MsgId = MsgId_BASE + sensor_id_ × 0x10
  uint8_t  sensor_id_{0};

  uint32_t radar_state_id_;
  uint32_t obj_status_id_;
  uint32_t obj_general_id_;
  uint32_t obj_quality_id_;
  uint32_t obj_extended_id_;
  uint32_t obj_warning_id_;
  uint32_t radar_cfg_id_;

  // ── Internal state ────────────────────────────────────────────────────────
  bool valid_radar_state_{false};
  bool sequential_publish_{false};

  ars408::RadarState    current_radar_state_;
  ars408::Obj_0_Status  current_objects_status_;

  std::unordered_map<uint8_t, ars408::RadarObject> radar_objects_;
  uint8_t updated_objects_general_{0};
  uint8_t updated_objects_quality_{0};
  uint8_t updated_objects_ext_{0};

  std::function<void(const std::unordered_map<uint8_t, ars408::RadarObject> &)>
    detected_objects_callback_;

  // ── Private parse methods ─────────────────────────────────────────────────

  /// Parses RadarState (0x201+offset) — radar configuration status
  void ParseRadarState(const std::array<uint8_t, 8> & in_can_data);

  /// Parses Obj_0_Status (0x60A+offset) — cycle header: NofObjects, MeasCounter
  ars408::Obj_0_Status ParseObject0_Status(const std::array<uint8_t, 8> & in_can_data);

  /// Parses Obj_1_General (0x60B+offset) — position, velocity, RCS, DynProp
  ars408::RadarObject ParseObject1_General(const std::array<uint8_t, 8> & in_can_data);

  /// Parses Obj_2_Quality (0x60C+offset) — RMS uncertainties, existence probability
  ars408::Obj_2_Quality ParseObject2_Quality(const std::array<uint8_t, 8> & in_can_data);

  /// Parses Obj_3_Extended (0x60D+offset) — class, acceleration, size, orientation
  ars408::Obj_3_Extended ParseObject3_Extended(const std::array<uint8_t, 8> & in_can_data);

  // ── Object pool helpers ───────────────────────────────────────────────────
  void AddDetectedObject(ars408::RadarObject in_object);
  void CallDetectedObjectsCallback(
    std::unordered_map<uint8_t, ars408::RadarObject> & in_detected_objects);
  bool DetectedObjectsReady();
  void ClearRadarObjects();
  void UpdateObjectQuality(uint8_t in_object_id, const ars408::Obj_2_Quality & in_object_quality);
  void UpdateObjectExtInfo(uint8_t in_object_id, const ars408::Obj_3_Extended & in_object_ext_info);

public:
  /// Initializes the driver with the radar SensorID (0–7).
  /// Computes all CAN IDs dynamically: MsgId = BASE + sensor_id × 0x10
  void Init(uint8_t sensor_id);

  /// Dispatches an incoming CAN frame to the appropriate parser.
  std::string Parse(
    const uint32_t & can_id,
    const std::array<uint8_t, 8> & in_can_data,
    const uint8_t & in_data_length);

  /// Returns true and fills out_current_state if RadarState has been received.
  bool GetCurrentRadarState(ars408::RadarState & out_current_state);

  /// Builds the 8-byte RadarConfiguration (0x200) CAN frame from in_new_status.
  std::array<uint8_t, 8> GenerateRadarConfiguration(const ars408::RadarCfg & in_new_status);

  /// Registers the callback invoked once a full set of objects is decoded.
  void RegisterDetectedObjectsCallback(
    std::function<void(const std::unordered_map<uint8_t, ars408::RadarObject> &)> objects_callback,
    bool sequential_publish);
};

}  // namespace ars408

#endif  // ARS408_ROS__ARS408_DRIVER_HPP_
