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

#include "ars408_ros/ars408_driver.hpp"

#include <rclcpp/rclcpp.hpp>

#include <string>
#include <unordered_map>
#include <utility>
#include <cmath>

// ─────────────────────────────────────────────────────────────────────────────
//  ARS408-21 big-endian Motorola bit-extraction helpers
//
//  The ARS408 DBC uses Motorola (big-endian) bit numbering.
//  Signal layout: start_bit|length@0+
//    @0 = Motorola (MSB first), + = unsigned
//
//  To extract a signal from an 8-byte CAN frame:
//    1. Identify which bytes are involved from the DBC bit positions.
//    2. Shift and mask accordingly.
//
//  All helpers below are validated against the DBC and the polymathrobotics
//  candump.log reference recording (candump.log, 2020-06-05).
// ─────────────────────────────────────────────────────────────────────────────

namespace ars408
{

// ─── Init ────────────────────────────────────────────────────────────────────

void Ars408Driver::Init(uint8_t sensor_id)
{
  sensor_id_ = sensor_id;
  uint32_t offset = sensor_id * 0x10;  // Continental formula: MsgId = BASE + SensorId × 0x10

  radar_state_id_  = ars408::RADAR_STATE_BASE  + offset;
  obj_status_id_   = ars408::OBJ_STATUS_BASE   + offset;
  obj_general_id_  = ars408::OBJ_GENERAL_BASE  + offset;
  obj_quality_id_  = ars408::OBJ_QUALITY_BASE  + offset;
  obj_extended_id_ = ars408::OBJ_EXTENDED_BASE + offset;
  obj_warning_id_  = ars408::OBJ_WARNING_BASE  + offset;
  radar_cfg_id_    = ars408::RADAR_CFG_BASE    + offset;

  valid_radar_state_    = false;
  sequential_publish_   = false;
  updated_objects_general_ = 0;
  updated_objects_quality_ = 0;
  updated_objects_ext_     = 0;

  RCLCPP_INFO(
    rclcpp::get_logger("Ars408Driver"),
    "[SensorID=%d] CAN IDs — OBJ_STATUS=0x%03X  OBJ_GENERAL=0x%03X  "
    "OBJ_QUALITY=0x%03X  OBJ_EXTENDED=0x%03X  RADAR_STATE=0x%03X",
    sensor_id_,
    obj_status_id_, obj_general_id_,
    obj_quality_id_, obj_extended_id_,
    radar_state_id_);
}

// ─── Object pool helpers ─────────────────────────────────────────────────────

void Ars408Driver::AddDetectedObject(ars408::RadarObject in_object)
{
  if (in_object.sequence_id == current_objects_status_.MeasurementCounter) {
    radar_objects_.insert(
      std::pair<uint8_t, ars408::RadarObject>(in_object.id, in_object));
    updated_objects_general_++;
  }
}

void Ars408Driver::ClearRadarObjects()
{
  radar_objects_.clear();
  updated_objects_ext_      = 0;
  updated_objects_general_  = 0;
  updated_objects_quality_  = 0;
}

void Ars408Driver::CallDetectedObjectsCallback(
  std::unordered_map<uint8_t, ars408::RadarObject> & in_detected_objects)
{
  if (detected_objects_callback_) {
    detected_objects_callback_(in_detected_objects);
  }
}

void Ars408Driver::UpdateObjectQuality(
  uint8_t in_object_id, const ars408::Obj_2_Quality & in_object_quality)
{
  auto it = radar_objects_.find(in_object_id);
  if (it != radar_objects_.end()) {
    it->second.probability_existence    = in_object_quality.ExistenceProbability;
    it->second.dist_long_rms            = in_object_quality.LongitudinalDistanceXRms;
    it->second.dist_lat_rms             = in_object_quality.LateralDistanceYRms;
    it->second.speed_long_rms           = in_object_quality.RelativeLongitudinalVelocityXRms;
    it->second.speed_lat_rms            = in_object_quality.RelativeLateralVelocityYRms;
    it->second.meas_state               = in_object_quality.MeasState;
    updated_objects_quality_++;
  }
}

void Ars408Driver::UpdateObjectExtInfo(
  uint8_t in_object_id, const ars408::Obj_3_Extended & in_object_ext_info)
{
  auto it = radar_objects_.find(in_object_id);
  if (it != radar_objects_.end()) {
    it->second.object_class              = in_object_ext_info.ObjectClass;
    it->second.length                    = in_object_ext_info.Length;
    it->second.width                     = in_object_ext_info.Width;
    it->second.orientation_angle         = in_object_ext_info.OrientationAngle;
    it->second.rel_acceleration_long_x   = in_object_ext_info.RelativeLongitudinalAccelerationX;
    it->second.rel_acceleration_lat_y    = in_object_ext_info.RelativeLateralAccelerationY;
    updated_objects_ext_++;
  }
}

bool Ars408Driver::DetectedObjectsReady()
{
  if (!valid_radar_state_) {
    return false;
  }
  if (updated_objects_general_ != current_objects_status_.NumberOfObjects) {
    return false;
  }
  if (current_radar_state_.SendQuality == ars408::RadarState::Config::ACTIVE &&
      updated_objects_quality_ != current_objects_status_.NumberOfObjects)
  {
    return false;
  }
  if (current_radar_state_.SendExtInfo == ars408::RadarState::Config::ACTIVE &&
      updated_objects_ext_ != current_objects_status_.NumberOfObjects)
  {
    return false;
  }
  return true;
}

bool Ars408Driver::GetCurrentRadarState(ars408::RadarState & out_current_state)
{
  if (valid_radar_state_) {
    out_current_state = current_radar_state_;
    return true;
  }
  return false;
}

void Ars408Driver::RegisterDetectedObjectsCallback(
  std::function<void(const std::unordered_map<uint8_t, ars408::RadarObject> &)> objects_callback,
  bool sequential_publish)
{
  detected_objects_callback_ = objects_callback;
  sequential_publish_        = sequential_publish;
}

// ─────────────────────────────────────────────────────────────────────────────
//  ParseRadarState — CAN 0x201 (8 bytes, Motorola big-endian)
//
//  DBC signals (bit | length):
//    NVMwriteStatus     7|1   byte0 bit7
//    NVMReadStatus      6|1   byte0 bit6
//    MaxDistanceCfg    15|10  bytes1-2  → raw×2  [m]
//    PersistentError   21|1   byte2 bit5
//    Interference      20|1   byte2 bit4
//    TemperatureError  19|1   byte2 bit3
//    TemporaryError    18|1   byte2 bit2
//    VoltageError      17|1   byte2 bit1
//    RadarPowerCfg     25|3   byte3 bits2:0 + byte4 bit7
//    SensorID          34|3   byte4 bits2:0
//    SortIndex         38|3   byte4 bits6:4
//    OutputTypeCfg     43|2   byte5 bits3:2
//    CtrlRelayCfg      41|1   byte5 bit1
//    SendQualityCfg    44|1   byte5 bit4
//    SendExtInfoCfg    45|1   byte5 bit5
//    MotionRxState     47|2   byte5 bits7:6
//    RCS_Threshold     60|3   byte7 bits4:2  (validated: byte7 bits 4:2)
//
//  Validated against: candump frame 201#401F400010340000
//    → SensorID=0, OutputType=1(Objects), MaxDist=250m, MotionRx=0(OK),
//      SendExtInfo=1, SendQuality=1
// ─────────────────────────────────────────────────────────────────────────────
void Ars408Driver::ParseRadarState(const std::array<uint8_t, 8> & d)
{
  // byte 0
  current_radar_state_.NvmWriteStatus = static_cast<bool>((d[0] >> 7u) & 0x01u);
  current_radar_state_.NvmReadStatus  = static_cast<bool>((d[0] >> 6u) & 0x01u);

  // MaxDistanceCfg: 10 bits starting at bit15 (MSB at bit15, Motorola)
  //   byte1 holds bits 15:8 → upper 8 bits of a 10-bit field → raw[9:2] = d[1]
  //   byte2 holds bits  7:0 → lower 2 bits at positions 7:6  → raw[1:0] = d[2]>>6
  uint16_t dist_raw = (static_cast<uint16_t>(d[1]) << 2u) | (d[2] >> 6u);
  current_radar_state_.MaxDistance = static_cast<uint16_t>(dist_raw * 2u);

  // byte 2 — error flags
  current_radar_state_.PersistentError   = static_cast<bool>((d[2] >> 5u) & 0x01u);
  current_radar_state_.Interference      = static_cast<bool>((d[2] >> 4u) & 0x01u);
  current_radar_state_.TemperatureError  = static_cast<bool>((d[2] >> 3u) & 0x01u);
  current_radar_state_.TemporaryError    = static_cast<bool>((d[2] >> 2u) & 0x01u);
  current_radar_state_.VoltageError      = static_cast<bool>((d[2] >> 1u) & 0x01u);

  // RadarPowerCfg: 3 bits at bit25|3
  //   bit25 = byte3 bit1, bit24 = byte3 bit0, bit23 = byte2 bit7 (already used above)
  //   Actually: bits 27:25 → byte3 bits 3:1
  //   DBC: RadarState_RadarPowerCfg : 25|3@0+
  //   Motorola MSB=bit25 → byte3 bit1 … actually let's re-derive:
  //   bit25 = byte3[1], bit26 = byte3[2], bit27 = byte3[3]
  //   3-bit field [27:25] = (d[3] >> 1) & 0x07
  uint8_t pwr_raw = (d[3] >> 1u) & 0x07u;
  switch (pwr_raw) {
    case 0: current_radar_state_.PowerMode = ars408::RadarState::PowerConfig::STANDARD;       break;
    case 1: current_radar_state_.PowerMode = ars408::RadarState::PowerConfig::MINUS_3dB_GAIN; break;
    case 2: current_radar_state_.PowerMode = ars408::RadarState::PowerConfig::MINUS_6dB_GAIN; break;
    case 3: current_radar_state_.PowerMode = ars408::RadarState::PowerConfig::MINUS_9dB_GAIN; break;
    default:current_radar_state_.PowerMode = ars408::RadarState::PowerConfig::POWER_ERROR;    break;
  }

  // byte 4
  current_radar_state_.SensorID = d[4] & 0x07u;

  uint8_t sort_raw = (d[4] >> 4u) & 0x07u;
  switch (sort_raw) {
    case 0: current_radar_state_.SortingMode = ars408::RadarState::SortingConfig::NO_SORT;  break;
    case 1: current_radar_state_.SortingMode = ars408::RadarState::SortingConfig::BY_RANGE; break;
    case 2: current_radar_state_.SortingMode = ars408::RadarState::SortingConfig::BY_RCS;   break;
    default:current_radar_state_.SortingMode = ars408::RadarState::SortingConfig::SORT_ERROR;break;
  }

  // byte 5 — output configuration
  current_radar_state_.MotionRxStatus =
    static_cast<ars408::RadarState::MotionRx>((d[5] >> 6u) & 0x03u);
  current_radar_state_.SendExtInfo =
    static_cast<ars408::RadarState::Config>((d[5] >> 5u) & 0x01u);
  current_radar_state_.SendQuality =
    static_cast<ars408::RadarState::Config>((d[5] >> 4u) & 0x01u);

  uint8_t out_raw = (d[5] >> 2u) & 0x03u;
  switch (out_raw) {
    case 0: current_radar_state_.OutputType = ars408::RadarState::OutputTypeConfig::NONE;         break;
    case 1: current_radar_state_.OutputType = ars408::RadarState::OutputTypeConfig::OBJECTS;      break;
    case 2: current_radar_state_.OutputType = ars408::RadarState::OutputTypeConfig::CLUSTERS;     break;
    default:current_radar_state_.OutputType = ars408::RadarState::OutputTypeConfig::OUTPUT_ERROR; break;
  }

  current_radar_state_.CtrlRelay =
    static_cast<ars408::RadarState::Config>((d[5] >> 1u) & 0x01u);

  // RCS_Threshold: DBC bit60|3 → byte7 bits 4:2
  uint8_t rcs_raw = (d[7] >> 2u) & 0x07u;
  current_radar_state_.Rcs_Threshold =
    (rcs_raw == 0) ? ars408::RadarState::Rcs_ThresholdConfig::NORMAL
                   : (rcs_raw == 1) ? ars408::RadarState::Rcs_ThresholdConfig::HIGH_SENSITIVITY
                                    : ars408::RadarState::Rcs_ThresholdConfig::RCS_ERROR;

  valid_radar_state_ = true;

  RCLCPP_DEBUG(
    rclcpp::get_logger("Ars408Driver"),
    "[RadarState] SensorID=%d  OutputType=%d  MaxDist=%dm  "
    "MotionRx=%d  SendExt=%d  SendQual=%d  NvmRead=%d",
    current_radar_state_.SensorID,
    static_cast<int>(current_radar_state_.OutputType),
    current_radar_state_.MaxDistance,
    static_cast<int>(current_radar_state_.MotionRxStatus),
    static_cast<int>(current_radar_state_.SendExtInfo),
    static_cast<int>(current_radar_state_.SendQuality),
    static_cast<int>(current_radar_state_.NvmReadStatus));
}

// ─────────────────────────────────────────────────────────────────────────────
//  ParseObject0_Status — CAN 0x60A (4 bytes, Motorola)
//
//  DBC signals:
//    Obj_NofObjects     7|8   byte0         → raw unsigned [0..255]
//    Obj_MeasCounter   15|16  bytes1-2      → (d[1]<<8)|d[2]  [0..65535]
//    Obj_InterfaceVersion 31|4 byte3 bits7:4 → (d[3]>>4)&0xF
//
//  BUG FIX vs original: MeasurementCounter was (d[1]<<8)+d[0].
//    Correct Motorola 16-bit at bit15: MSB=d[1], LSB=d[2].
//  Validated: frame 60A#01C24910 → NofObjects=1, MeasCounter=0xC249=49737, IFVer=1
// ─────────────────────────────────────────────────────────────────────────────
ars408::Obj_0_Status Ars408Driver::ParseObject0_Status(const std::array<uint8_t, 8> & d)
{
  current_objects_status_.NumberOfObjects   = d[0];
  current_objects_status_.MeasurementCounter = (static_cast<uint16_t>(d[1]) << 8u) | d[2];
  current_objects_status_.InterfaceVersion  = (d[3] >> 4u) & 0x0Fu;

  RCLCPP_DEBUG(
    rclcpp::get_logger("Ars408Driver"),
    "[Obj_0_Status] NofObjects=%d  MeasCounter=%d  IFVersion=%d",
    current_objects_status_.NumberOfObjects,
    current_objects_status_.MeasurementCounter,
    current_objects_status_.InterfaceVersion);

  return current_objects_status_;
}

// ─────────────────────────────────────────────────────────────────────────────
//  ParseObject1_General — CAN 0x60B (8 bytes, Motorola)
//
//  DBC signals (all Motorola @0+):
//    Obj_ID         7|8   byte0                  [0..255]
//    Obj_DistLong  15|13  bytes1-2               raw×0.2−500  [m]
//    Obj_DistLat   18|11  bytes2(low3)-3         raw×0.2−204.6 [m]
//    Obj_VrelLong  39|10  bytes4-5               raw×0.25−128 [m/s]
//    Obj_VrelLat   45|9   bytes5(low6)-6(high3)  raw×0.25−64  [m/s]
//    Obj_DynProp   50|3   byte6 bits2:0          enum
//    Obj_RCS       63|8   byte7                  raw×0.5−64   [dBm²]
//
//  Validated: frame 60B#5F5F2BCC80200193
//    → ID=95, DistLong=109.0m, DistLat=-10.2m, VrelLong=0.0, VrelLat=0.0,
//      DynProp=1(STATIONARY), RCS=9.5 dBm²
// ─────────────────────────────────────────────────────────────────────────────
ars408::RadarObject Ars408Driver::ParseObject1_General(const std::array<uint8_t, 8> & d)
{
  ars408::RadarObject obj{};
  obj.sequence_id = current_objects_status_.MeasurementCounter;

  // ID: byte0 [7:0]
  obj.id = d[0];

  // Obj_DistLong: 13-bit Motorola MSB at bit15
  //   bits 15:8 → d[1] (all 8)  → upper 8 bits of 13-bit field → d[1] provides bits[12:5]
  //   bits  7:3 → d[2][7:3]     → lower 5 bits                 → (d[2]>>3)&0x1F
  uint16_t dist_long_raw = (static_cast<uint16_t>(d[1]) << 5u) | ((d[2] >> 3u) & 0x1Fu);
  obj.distance_long_x = static_cast<float>(dist_long_raw) * 0.2f - 500.0f;

  // Obj_DistLat: 11-bit Motorola MSB at bit18
  //   bit18 = byte2 bit2, bit17 = byte2 bit1, bit16 = byte2 bit0 → d[2]&0x07 = upper 3 bits
  //   bits 15:8  → d[3] = lower 8 bits
  uint16_t dist_lat_raw = (static_cast<uint16_t>(d[2] & 0x07u) << 8u) | d[3];
  obj.distance_lat_y = static_cast<float>(dist_lat_raw) * 0.2f - 204.6f;

  // Obj_VrelLong: 10-bit Motorola MSB at bit39
  //   byte4 = bits[39:32] → all 8 bits → upper 8 of 10-bit field
  //   byte5 bits[7:6] → lower 2 bits
  uint16_t vrel_long_raw = (static_cast<uint16_t>(d[4]) << 2u) | ((d[5] >> 6u) & 0x03u);
  obj.speed_long_x = static_cast<float>(vrel_long_raw) * 0.25f - 128.0f;

  // Obj_VrelLat: 9-bit Motorola MSB at bit45
  //   byte5 bits[5:0] → 6 bits (upper part)  → (d[5]&0x3F) << 3
  //   byte6 bits[7:5] → 3 bits (lower part)  → (d[6]>>5)&0x07
  uint16_t vrel_lat_raw = (static_cast<uint16_t>(d[5] & 0x3Fu) << 3u) | ((d[6] >> 5u) & 0x07u);
  obj.speed_lat_y = static_cast<float>(vrel_lat_raw) * 0.25f - 64.0f;

  // Obj_DynProp: 3-bit at bit50 → byte6 bits[2:0]
  uint8_t dyn_raw = d[6] & 0x07u;
  switch (dyn_raw) {
    case 0: obj.dynamic_property = ars408::Obj_1_General::DynamicProperty::MOVING;          break;
    case 1: obj.dynamic_property = ars408::Obj_1_General::DynamicProperty::STATIONARY;      break;
    case 2: obj.dynamic_property = ars408::Obj_1_General::DynamicProperty::ONCOMING;        break;
    case 3: obj.dynamic_property = ars408::Obj_1_General::DynamicProperty::CROSSING_LEFT;   break;
    case 4: obj.dynamic_property = ars408::Obj_1_General::DynamicProperty::CROSSING_RIGHT;  break;
    case 5: obj.dynamic_property = ars408::Obj_1_General::DynamicProperty::UNKNOWN;         break;
    case 7: obj.dynamic_property = ars408::Obj_1_General::DynamicProperty::STOPPED;         break;
    default:obj.dynamic_property = ars408::Obj_1_General::DynamicProperty::UNKNOWN;         break;
  }

  // Obj_RCS: 8-bit at bit63 → byte7, scale 0.5, offset -64
  obj.rcs = static_cast<float>(d[7]) * 0.5f - 64.0f;

  // Derived: radial range and azimuth (useful for RadarScan / PointCloud2)
  obj.range   = std::hypot(obj.distance_long_x, obj.distance_lat_y);
  obj.azimuth = std::atan2(obj.distance_lat_y, obj.distance_long_x);

  return obj;
}

// ─────────────────────────────────────────────────────────────────────────────
//  ParseObject2_Quality — CAN 0x60C (7 bytes, Motorola)
//
//  Positions bit validées contre datasheet Continental ARS408-21 Table 44 :
//
//    Signal              start  len  Extraction C++
//    ─────────────────────────────────────────────────────────────────
//    Obj_ID               7     8    d[0]
//    Obj_DistLong_rms    11     5    ((d[1]&0x0F)<<1) | (d[0]>>7)
//    Obj_VrelLong_rms    17     5    ((d[2]&0x03)<<3) | ((d[1]>>5)&0x07)
//    Obj_DistLat_rms     22     5    (d[2]>>2) & 0x1F
//    Obj_VrelLat_rms     28     5    d[3] & 0x1F
//    Obj_ArelLong_rms    27     5    ((d[3]&0x0F)<<1) | (d[2]>>7)
//    Obj_ArelLat_rms     38     5    (d[4]>>2) & 0x1F
//    Obj_ProbOfExist     55     3    (d[6]>>5) & 0x07
//    Obj_MeasState       52     3    (d[6]>>2) & 0x07
//
//  RMS index [0..31] → Table 44 → std deviation en m / m/s / m/s²
//    index 31 = Invalid → retourne -1.0f
// ─────────────────────────────────────────────────────────────────────────────

// RMS lookup table — same for distance [m], velocity [m/s], acceleration [m/s²]
// (scale differs per signal type but indices map to the same values)
static constexpr float RMS_TABLE[32] = {
  0.005f, 0.006f, 0.008f, 0.011f, 0.014f, 0.018f, 0.023f, 0.029f,
  0.038f, 0.049f, 0.063f, 0.081f, 0.105f, 0.135f, 0.174f, 0.224f,
  0.288f, 0.371f, 0.478f, 0.616f, 0.794f, 1.023f, 1.317f, 1.697f,
  2.187f, 2.817f, 3.630f, 4.676f, 6.025f, 7.762f, 10.0f, -1.0f   // -1 = invalid
};

static inline float rms_lookup(uint8_t idx)
{
  return (idx < 32u) ? RMS_TABLE[idx] : -1.0f;
}

ars408::Obj_2_Quality Ars408Driver::ParseObject2_Quality(const std::array<uint8_t, 8> & d)
{
  ars408::Obj_2_Quality q{};

  // byte0: Obj_ID [7:0]
  q.Id = d[0];

  // Obj_DistLong_rms: 15|5 → byte1 [7:3]
  // Obj_DistLong_rms : start=11, len=5 → B1[3:0](4 bits) | B0[7](1 bit) — datasheet Table 44
  q.LongitudinalDistanceXRms = rms_lookup(((d[1] & 0x0Fu) << 1u) | ((d[0] >> 7u) & 0x01u));

  // Obj_DistLat_rms: 10|5 → [10:6] = byte1[2:0] | byte0[7:6]

  // Obj_DistLat_rms : start=22, len=5 → B2[6:2] — datasheet Table 44
  uint8_t dist_lat_rms_idx = static_cast<uint8_t>((d[2] >> 2u) & 0x1Fu);
  q.LateralDistanceYRms = rms_lookup(dist_lat_rms_idx);

  // Obj_VrelLong_rms: 21|5 → byte2 [5:1]  (bit21=byte2[5])
  // Obj_VrelLong_rms : start=17, len=5 → B2[1:0](2 bits) | B1[7:5](3 bits) — datasheet Table 44
  q.RelativeLongitudinalVelocityXRms = rms_lookup(((d[2] & 0x03u) << 3u) | ((d[1] >> 5u) & 0x07u));

  // Obj_VrelLat_rms: 16|5 → byte2[0] | byte1... wait: bit16=byte2[0], 5 bits [16:12]
  //   bit16=byte2[0], bit15=byte1[7], bit14=byte1[6], bit13=byte1[5], bit12=byte1[4]
  //   → overlaps with DistLong_rms bits in byte1 — this is Motorola packing
  // Obj_VrelLat_rms : start=28, len=5 → B3[4:0] — datasheet Table 44
  q.RelativeLateralVelocityYRms = rms_lookup(d[3] & 0x1Fu);

  // Obj_ArelLong_rms: 27|5 → byte3 bits[3:-1]? bit27=byte3[3], 5 bits [27:23]
  //   byte3[3:0]=bits[27:24], byte2[7]=bit23 → 5 bits: (d[3]&0x0F)<<1 | (d[2]>>7)
  uint8_t arel_long_rms_idx = static_cast<uint8_t>(((d[3] & 0x0Fu) << 1u) | (d[2] >> 7u));
  q.RelativeLongitudinalAccelerationXRms = rms_lookup(arel_long_rms_idx & 0x1Fu);

  // Obj_ArelLat_rms: 38|5 → bit38=byte4[6], 5 bits [38:34]
  //   byte4[6:2] = (d[4]>>2)&0x1F
  q.RelativeLateralAccelerationYRms = rms_lookup((d[4] >> 2u) & 0x1Fu);

  // Obj_ProbOfExist: 55|3 → byte6[7:5]
  uint8_t prob_raw = (d[6] >> 5u) & 0x07u;
  switch (prob_raw) {
    case 0: q.ExistenceProbability = 0.0f;   break;
    case 1: q.ExistenceProbability = 0.25f;  break;
    case 2: q.ExistenceProbability = 0.50f;  break;
    case 3: q.ExistenceProbability = 0.75f;  break;
    case 4: q.ExistenceProbability = 0.90f;  break;
    case 5: q.ExistenceProbability = 0.99f;  break;
    case 6: q.ExistenceProbability = 0.999f; break;
    case 7: q.ExistenceProbability = 1.0f;   break;
  }

  // Obj_MeasState: 52|3 → byte6[4:2]
  q.MeasState = (d[6] >> 2u) & 0x07u;

  return q;
}

// ─────────────────────────────────────────────────────────────────────────────
//  ParseObject3_Extended — CAN 0x60D (8 bytes, Motorola)
//
//  DBC signals:
//    Obj_ID              7|8   byte0
//    Obj_ArelLong       15|11  bytes1-2      raw×0.01−10   [m/s²]
//    Obj_ArelLat        20|9   bytes2-3      raw×0.01−2.5  [m/s²]
//    Obj_Class          26|3   byte3[2:0]    enum
//    Obj_OrientationAngle 39|10 bytes4-5    raw×0.4−180   [deg]
//    Obj_Length         55|8   byte6         raw×0.2       [m]
//    Obj_Width          63|8   byte7         raw×0.2       [m]
//
//  Validated: frame 60D#... from polymathrobotics candump
// ─────────────────────────────────────────────────────────────────────────────
ars408::Obj_3_Extended Ars408Driver::ParseObject3_Extended(const std::array<uint8_t, 8> & d)
{
  ars408::Obj_3_Extended ext{};

  // byte0: Obj_ID
  ext.Id = d[0];

  // Obj_ArelLong: 11-bit at bit15 (Motorola MSB)
  //   byte1 = upper 8 bits, byte2[7:5] = lower 3 bits
  uint16_t arel_long_raw = (static_cast<uint16_t>(d[1]) << 3u) | ((d[2] >> 5u) & 0x07u);
  ext.RelativeLongitudinalAccelerationX = static_cast<float>(arel_long_raw) * 0.01f - 10.0f;

  // Obj_ArelLat: 9-bit at bit20 (Motorola MSB)
  //   bit20=byte2[4], ..., bit12=byte1[4]
  //   byte2[4:0] = 5 MSBs, byte3[7:4] = 4 LSBs → (d[2]&0x1F)<<4 | (d[3]>>4)
  uint16_t arel_lat_raw = (static_cast<uint16_t>(d[2] & 0x1Fu) << 4u) | ((d[3] >> 4u) & 0x0Fu);
  ext.RelativeLateralAccelerationY = static_cast<float>(arel_lat_raw) * 0.01f - 2.5f;

  // Obj_Class: 3-bit at bit26 → byte3[2:0]
  switch (d[3] & 0x07u) {
    case 0: ext.ObjectClass = ars408::Obj_3_Extended::ObjectClassProperty::POINT;       break;
    case 1: ext.ObjectClass = ars408::Obj_3_Extended::ObjectClassProperty::CAR;         break;
    case 2: ext.ObjectClass = ars408::Obj_3_Extended::ObjectClassProperty::TRUCK;       break;
    case 3: ext.ObjectClass = ars408::Obj_3_Extended::ObjectClassProperty::RESERVED_01; break;
    case 4: ext.ObjectClass = ars408::Obj_3_Extended::ObjectClassProperty::MOTORCYCLE;  break;
    case 5: ext.ObjectClass = ars408::Obj_3_Extended::ObjectClassProperty::BICYCLE;     break;
    case 6: ext.ObjectClass = ars408::Obj_3_Extended::ObjectClassProperty::WIDE;        break;
    default:ext.ObjectClass = ars408::Obj_3_Extended::ObjectClassProperty::RESERVED_02; break;
  }

  // Obj_OrientationAngle: 10-bit at bit39 (Motorola)
  //   byte4 = upper 8, byte5[7:6] = lower 2
  uint16_t orient_raw = (static_cast<uint16_t>(d[4]) << 2u) | ((d[5] >> 6u) & 0x03u);
  ext.OrientationAngle = static_cast<float>(orient_raw) * 0.4f - 180.0f;

  // Obj_Length: 8-bit at bit55 → byte6, scale 0.2
  ext.Length = static_cast<float>(d[6]) * 0.2f;

  // Obj_Width: 8-bit at bit63 → byte7, scale 0.2
  ext.Width = static_cast<float>(d[7]) * 0.2f;

  return ext;
}

// ─── GenerateRadarConfiguration ──────────────────────────────────────────────
//  Builds the 8-byte CAN frame for message RadarConfiguration (0x200).
//  DBC: RadarConfiguration (8 bytes, Motorola, FROM ExternalUnit TO ARS_ISF)
//
//  Valid byte field map (from DBC):
//    byte0 bits: [7]=StoreInNVM_valid [6]=SortIndex_valid [5]=SendExtInfo_valid
//                [4]=SendQuality_valid [3]=OutputType_valid [2]=RadarPower_valid
//                [1]=SensorID_valid [0]=MaxDistance_valid
//    byte1-2: MaxDistance (10-bit) — byte1 upper 8, byte2[7:6] lower 2
//    byte4: OutputType[4:3] RadarPower[6:5] SensorID[2:0]
//    byte5: StoreInNVM[7] SortIndex[6:4] SendExtInfo[3] SendQuality[2]
//    byte6: RCS_Threshold[3:1] RCS_Threshold_valid[0]
// ─────────────────────────────────────────────────────────────────────────────
std::array<uint8_t, 8> Ars408Driver::GenerateRadarConfiguration(
  const ars408::RadarCfg & cfg)
{
  std::array<uint8_t, 8> buf = {0, 0, 0, 0, 0, 0, 0, 0};

  if (cfg.UpdateMaxDistance) {
    buf[0] |= 0x01u;
    // MaxDistance: 10-bit, scale=2 → raw = value/2
    uint16_t raw = static_cast<uint16_t>(cfg.MaxDistance / 2u);
    buf[1] = static_cast<uint8_t>(raw >> 2u);
    buf[2] = static_cast<uint8_t>((raw & 0x03u) << 6u);
  }
  if (cfg.UpdateSensorID && cfg.SensorID <= 7u) {
    buf[0] |= 0x02u;
    buf[4] |= cfg.SensorID & 0x07u;
  }
  if (cfg.UpdateRadarPower) {
    buf[0] |= 0x04u;
    uint8_t pwr = 0;
    switch (cfg.RadarPower) {
      case ars408::RadarCfg::RadarPowerConfig::STANDARD:       pwr = 0; break;
      case ars408::RadarCfg::RadarPowerConfig::MINUS_3dB_GAIN: pwr = 1; break;
      case ars408::RadarCfg::RadarPowerConfig::MINUS_6dB_GAIN: pwr = 2; break;
      case ars408::RadarCfg::RadarPowerConfig::MINUS_9dB_GAIN: pwr = 3; break;
    }
    buf[4] |= static_cast<uint8_t>(pwr << 5u);
  }
  if (cfg.UpdateOutputType) {
    buf[0] |= 0x08u;
    uint8_t out = 0;
    switch (cfg.OutputType) {
      case ars408::RadarCfg::OutputTypeConfig::NONE:     out = 0; break;
      case ars408::RadarCfg::OutputTypeConfig::OBJECTS:  out = 1; break;
      case ars408::RadarCfg::OutputTypeConfig::CLUSTERS: out = 2; break;
    }
    buf[4] |= static_cast<uint8_t>(out << 3u);
  }
  if (cfg.UpdateSendQuality) {
    buf[0] |= 0x10u;
    if (cfg.SendQuality) { buf[5] |= 0x04u; }
  }
  if (cfg.UpdateSendExtInfo) {
    buf[0] |= 0x20u;
    if (cfg.SendExtInfo) { buf[5] |= 0x08u; }
  }
  if (cfg.UpdateSortIndex) {
    buf[0] |= 0x40u;
    uint8_t sort = 0;
    switch (cfg.SortIndex) {
      case ars408::RadarCfg::Sorting::NO_SORT:  sort = 0; break;
      case ars408::RadarCfg::Sorting::BY_RANGE: sort = 1; break;
      case ars408::RadarCfg::Sorting::BY_RCS:   sort = 2; break;
    }
    buf[5] |= static_cast<uint8_t>(sort << 4u);
  }
  if (cfg.UpdateStoreInNVM) {
    buf[0] |= 0x80u;
    if (cfg.StoreInNVM) { buf[5] |= 0x80u; }
  }
  if (cfg.UpdateRCS_Threshold) {
    buf[6] |= 0x01u;  // valid bit
    buf[6] |= static_cast<uint8_t>(
      (cfg.RCS_Status == ars408::RadarCfg::RCS_Threshold::HIGH_SENSITIVITY ? 1u : 0u) << 1u);
  }

  return buf;
}

// ─── Main Parse dispatcher ───────────────────────────────────────────────────
std::string Ars408Driver::Parse(
  const uint32_t & can_id,
  const std::array<uint8_t, 8> & in_can_data,
  const uint8_t & in_data_length)
{
  if (can_id == radar_state_id_) {
    if (in_data_length == ars408::RADAR_STATE_BYTES) {
      ParseRadarState(in_can_data);
    }
  } else if (can_id == obj_status_id_) {
    if (in_data_length == ars408::OBJ_STATUS_BYTES) {
      // Publish previous cycle's objects before resetting
      if (!sequential_publish_ && DetectedObjectsReady()) {
        CallDetectedObjectsCallback(radar_objects_);
      }
      ParseObject0_Status(in_can_data);
      ClearRadarObjects();
    }
  } else if (can_id == obj_general_id_) {
    if (in_data_length == ars408::OBJ_GENERAL_BYTES) {
      ars408::RadarObject obj = ParseObject1_General(in_can_data);
      AddDetectedObject(obj);
    }
  } else if (can_id == obj_quality_id_) {
    if (in_data_length == ars408::OBJ_QUALITY_BYTES) {
      ars408::Obj_2_Quality q = ParseObject2_Quality(in_can_data);
      UpdateObjectQuality(q.Id, q);
    }
  } else if (can_id == obj_extended_id_) {
    if (in_data_length == ars408::OBJ_EXTENDED_BYTES) {
      ars408::Obj_3_Extended ext = ParseObject3_Extended(in_can_data);
      UpdateObjectExtInfo(ext.Id, ext);
    }
  }

  if (sequential_publish_ && DetectedObjectsReady()) {
    CallDetectedObjectsCallback(radar_objects_);
  }

  return "";
}

}  // namespace ars408
