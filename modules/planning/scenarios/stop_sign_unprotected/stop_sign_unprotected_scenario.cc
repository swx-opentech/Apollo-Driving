/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

/**
 * @file stop_sign_unprotected_scenario.cc
 **/

#include "modules/planning/scenarios/stop_sign_unprotected/stop_sign_unprotected_scenario.h"

#include "modules/common_msgs/perception_msgs/perception_obstacle.pb.h"
#include "modules/planning/planning_base/proto/planning_config.pb.h"
#include "cyber/common/log.h"
#include "modules/planning/planning_base/common/frame.h"
#include "modules/planning/planning_base/common/planning_context.h"
#include "modules/planning/planning_base/gflags/planning_gflags.h"
#include "modules/planning/scenarios/stop_sign_unprotected/stage_creep.h"
#include "modules/planning/scenarios/stop_sign_unprotected/stage_intersection_cruise.h"
#include "modules/planning/scenarios/stop_sign_unprotected/stage_pre_stop.h"
#include "modules/planning/scenarios/stop_sign_unprotected/stage_stop.h"

namespace apollo {
namespace planning {

using apollo::hdmap::HDMapUtil;
using apollo::hdmap::StopSignInfo;
using apollo::hdmap::StopSignInfoConstPtr;

using StopSignLaneVehicles =
    std::unordered_map<std::string, std::vector<std::string>>;

bool StopSignUnprotectedScenario::Init(
    std::shared_ptr<DependencyInjector> injector, const std::string& name) {
  if (!Scenario::Init(injector, name)) {
    AERROR << "Failed to init scenario" << Name();
    return false;
  }

  if (!Scenario::LoadConfig<ScenarioStopSignUnprotectedConfig>(
          &context_.scenario_config)) {
    AERROR << "Fail to get config of scenario " << Name();
    return false;
  }
  return true;
}

/*
 * get all the lanes associated/guarded by a stop sign
 */
int StopSignUnprotectedScenario::GetAssociatedLanes(
    const StopSignInfo& stop_sign_info) {
  context_.associated_lanes.clear(); // 清空当前上下文中的关联车道列表

  std::vector<StopSignInfoConstPtr> associated_stop_signs;
  // 获取与输入停车标志关联的其他停车标志
  HDMapUtil::BaseMap().GetStopSignAssociatedStopSigns(stop_sign_info.id(),
                                                      &associated_stop_signs);

  for (const auto stop_sign : associated_stop_signs) {
    // 遍历这些关联停车标志，提取其重叠的车道ID
    if (stop_sign == nullptr) {
      continue;
    }

    const auto& associated_lane_ids = stop_sign->OverlapLaneIds();
    // 遍历 associated_lane_ids 中的车道ID ?
    for (const auto& lane_id : associated_lane_ids) {
      const auto lane = HDMapUtil::BaseMap().GetLaneById(lane_id);
      if (lane == nullptr) {
        continue;
      }
      // 遍历当前车道（lane）上的所有停车标志重叠信息（stop_sign_overlap），
      // 检查每个停车标志是否与指定停车标志（stop_sign）存在重叠。若存在，则将该车道与停车标志的关联信息存入上下文（context_）中
      for (const auto& stop_sign_overlap : lane->stop_signs()) {
        // 获取当前停车标志与目标停车标志的重叠信息
        auto over_lap_info =
            stop_sign_overlap->GetObjectOverlapInfo(stop_sign.get()->id());
        if (over_lap_info != nullptr) {
          // 若重叠信息存在，记录车道与停车标志的关联关系
          context_.associated_lanes.push_back(
              std::make_pair(lane, stop_sign_overlap));
          ADEBUG << "stop_sign: " << stop_sign_info.id().id()
                 << "; associated_lane: " << lane_id.id()
                 << "; associated_stop_sign: " << stop_sign.get()->id().id();
        }
      }
    }
  }

  return 0;
}

bool StopSignUnprotectedScenario::IsTransferable(
    const Scenario* const other_scenario, const Frame& frame) {
  if (!frame.local_view().planning_command->has_lane_follow_command()) {
    // 检查frame中的规划命令是否包含车道跟随指令，若无则返回false
    return false;
  }
  if (other_scenario == nullptr || frame.reference_line_info().empty()) {
    // 若other_scenario为空或参考线信息为空，则返回false
    return false;
  }

  // 获取第一条参考线信息及其首个遇到的重叠区域
  const auto& reference_line_info = frame.reference_line_info().front();
  const auto& first_encountered_overlaps =
      reference_line_info.FirstEncounteredOverlaps();
  
  // note: first_encountered_overlaps already sorted
  hdmap::PathOverlap* stop_sign_overlap = nullptr;
  // 遍历first_encountered_overlaps容器，检查其中的重叠类型
  for (const auto& overlap : first_encountered_overlaps) {
    if (overlap.first == ReferenceLineInfo::SIGNAL ||
        overlap.first == ReferenceLineInfo::YIELD_SIGN) {
          // 如果遇到信号灯（SIGNAL）或让行标志（YIELD_SIGN），直接返回false
      return false;
    } else if (overlap.first == ReferenceLineInfo::STOP_SIGN) {
      // 如果遇到停车标志（STOP_SIGN），将对应的重叠信息保存到stop_sign_overlap指针中，并跳出循环
      stop_sign_overlap = const_cast<hdmap::PathOverlap*>(&overlap.second);
      break;
    }
  }


  if (stop_sign_overlap == nullptr) {
    // 若stop_sign_overlap为空，直接返回false
    return false;
  }
  const double adc_front_edge_s = reference_line_info.AdcSlBoundary().end_s();
  // 计算车辆前缘到停车标志的距离adc_distance_to_stop_sign
  const double adc_distance_to_stop_sign =
      stop_sign_overlap->start_s - adc_front_edge_s;
  ADEBUG << "adc_distance_to_stop_sign[" << adc_distance_to_stop_sign
         << "] stop_sign[" << stop_sign_overlap->object_id
         << "] stop_sign_overlap_start_s[" << stop_sign_overlap->start_s << "]";

  // 判断该距离是否在合理范围内（大于0且小于等于配置的距离），决定是否进入停车标志场景
  const bool stop_sign_scenario =
      (adc_distance_to_stop_sign > 0.0 &&
       adc_distance_to_stop_sign <=
           context_.scenario_config.start_stop_sign_scenario_distance());

  return stop_sign_scenario;
}

bool StopSignUnprotectedScenario::Exit(Frame* frame) {
  injector_->planning_context()
      ->mutable_planning_status()
      ->mutable_stop_sign()
      ->Clear();
  return true;
}

bool StopSignUnprotectedScenario::Enter(Frame* frame) {
  const auto& reference_line_info = frame->reference_line_info().front();
  std::string current_stop_sign_overlap_id;
  const auto& overlaps = reference_line_info.FirstEncounteredOverlaps();

  // 查找最近的停车标志：遍历参考线上的重叠区域，找到第一个停车标志的ID。
  for (auto overlap : overlaps) {
    if (overlap.first == ReferenceLineInfo::STOP_SIGN) {
      current_stop_sign_overlap_id = overlap.second.object_id;
      break;
    }
  }

  // 检查当前停车标志重叠ID是否为空
  if (current_stop_sign_overlap_id.empty()) {
    injector_->planning_context()
        ->mutable_planning_status()
        ->mutable_stop_sign()
        ->Clear();
    AERROR << "Can not find stop sign overlap in refline";
    return false;
  }

  // 获取参考线上的所有停车标志重叠区域（stop_sign_overlaps）
  // 使用 std::find_if 查找与当前停车标志 ID（current_stop_sign_overlap_id）匹配的重叠区域
  const std::vector<hdmap::PathOverlap>& stop_sign_overlaps =
      reference_line_info.reference_line().map_path().stop_sign_overlaps();
  auto stop_sign_overlap_itr = std::find_if(
      stop_sign_overlaps.begin(), stop_sign_overlaps.end(),
      [&current_stop_sign_overlap_id](const hdmap::PathOverlap& overlap) {
        return overlap.object_id == current_stop_sign_overlap_id;
      });

  // 更新规划上下文中的停车标志信息
  if (stop_sign_overlap_itr != stop_sign_overlaps.end()) {
    injector_->planning_context()
        ->mutable_planning_status()
        ->mutable_stop_sign()
        ->set_current_stop_sign_overlap_id(current_stop_sign_overlap_id);
    ADEBUG << "Update PlanningContext with first_encountered stop sign["
           << current_stop_sign_overlap_id << "] start_s["
           << stop_sign_overlap_itr->start_s << "]";
  } else {
    AERROR << "Can not find stop sign overlap " << current_stop_sign_overlap_id;
    return false;
  }

  hdmap::StopSignInfoConstPtr stop_sign = HDMapUtil::BaseMap().GetStopSignById(
      hdmap::MakeMapId(current_stop_sign_overlap_id));
  if (!stop_sign) {
    AERROR << "Could not find stop sign: " << current_stop_sign_overlap_id;
    return false;
  }
  context_.current_stop_sign_overlap_id = current_stop_sign_overlap_id;
  context_.watch_vehicles.clear();

  GetAssociatedLanes(*stop_sign);
  return true;
}

}  // namespace planning
}  // namespace apollo
