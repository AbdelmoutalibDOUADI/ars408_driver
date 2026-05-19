// Copyright 2021 Perception Engine, Inc. All rights reserved.
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

#ifndef ARS408_ROS__ARS408_CONSTANTS_HPP_
#define ARS408_ROS__ARS408_CONSTANTS_HPP_
#include <cstdint>

namespace ars408
{

// ── Tailles des messages (inchangées) ──────────────────────────────────────
const uint8_t RADAR_CFG_BYTES          = 8;
const uint8_t RADAR_STATE_BYTES        = 8;
const uint8_t FILTER_CFG_BYTES         = 8;
const uint8_t FILTER_STATE_HEADER_BYTES= 2;
const uint8_t FILER_STATE_CFG_BYTES    = 5;
const uint8_t COLL_DET_CFG_BYTES       = 2;
const uint8_t COLL_DET_REGION_CFG_BYTES= 8;
const uint8_t COLL_DET_STATE_BYTES     = 4;
const uint8_t COLL_DET_REGION_STATE_BYTES = 8;
const uint8_t SPEED_INFORMATION_BYTES  = 2;
const uint8_t YAW_RATE_INFORMATION_BYTES = 2;
const uint8_t CLUSTER_STATUS_BYTES     = 5;
const uint8_t CLUSTER_GENERAL_BYTES    = 8;
const uint8_t CLUSTER_QUALITY_BYTES    = 5;
const uint8_t OBJ_STATUS_BYTES         = 4;
const uint8_t OBJ_GENERAL_BYTES        = 8;
const uint8_t OBJ_QUALITY_BYTES        = 7;
const uint8_t OBJ_EXTENDED_BYTES       = 8;
const uint8_t OBJ_WARNING_BYTES        = 4;
const uint8_t VERSION_ID_BYTES         = 4;
const uint8_t COLL_DET_RELAY_CTRL_BYTES= 1;

//  CAN IDs  base : 
// Formule Continental : MsgId = MsgId_BASE + SensorId * 0x10
const uint32_t RADAR_CFG_BASE          = 0x200;
const uint32_t RADAR_STATE_BASE        = 0x201;
const uint32_t FILTER_CFG_BASE         = 0x202;
const uint32_t FILTER_STATE_HEADER_BASE= 0x203;
const uint32_t FILER_STATE_CFG_BASE    = 0x204;
const uint32_t COLL_DET_CFG_BASE       = 0x400;
const uint32_t COLL_DET_REGION_CFG_BASE= 0x401;
const uint32_t COLL_DET_STATE_BASE     = 0x408;
const uint32_t COLL_DET_REGION_STATE_BASE = 0x402;
const uint32_t SPEED_INFORMATION_BASE  = 0x300;
const uint32_t YAW_RATE_INFORMATION_BASE = 0x301;
const uint32_t CLUSTER_STATUS_BASE     = 0x600;
const uint32_t CLUSTER_GENERAL_BASE    = 0x701;
const uint32_t CLUSTER_QUALITY_BASE    = 0x702;
const uint32_t OBJ_STATUS_BASE         = 0x60A;
const uint32_t OBJ_GENERAL_BASE        = 0x60B;
const uint32_t OBJ_QUALITY_BASE        = 0x60C;
const uint32_t OBJ_EXTENDED_BASE       = 0x60D;
const uint32_t OBJ_WARNING_BASE        = 0x60E;
const uint32_t VERSION_ID_BASE         = 0x700;

/*exception is the relay control message which has the same
ID for all sensors(doc Continental pg 8)*/
const uint32_t COLL_DET_RELAY_CTRL     = 0x8;

}  // namespace ars408

#endif  // ARS408_ROS__ARS408_CONSTANTS_HPP_