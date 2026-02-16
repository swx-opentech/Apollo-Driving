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
 * @file
 **/

#include "modules/planning/scenarios/traffic_light_unprotected_right_turn/traffic_light_unprotected_right_turn_scenario.h"

#include "modules/common_msgs/perception_msgs/perception_obstacle.pb.h"
#include "modules/common_msgs/perception_msgs/traffic_light_detection.pb.h"
#include "cyber/common/log.h"
#include "cyber/time/clock.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/planning/planning_base/common/frame.h"
#include "modules/planning/planning_base/common/planning_context.h"
#include "modules/planning/scenarios/traffic_light_unprotected_right_turn/stage_creep.h"
#include "modules/planning/scenarios/traffic_light_unprotected_right_turn/stage_intersection_cruise.h"
#include "modules/planning/scenarios/traffic_light_unprotected_right_turn/stage_stop.h"

namespace apollo {
namespace planning {

using apollo::hdmap::HDMapUtil;

bool TrafficLightUnprotectedRightTurnScenario::Init(
    std::shared_ptr<DependencyInjector> injector, const std::string& name) {
  if (init_) {
    return true;
  }

  if (!Scenario::Init(injector, name)) {
    AERROR << "failed to init scenario" << Name();
    return false;
  }

  if (!Scenario::LoadConfig<ScenarioTrafficLightUnprotectedRightTurnConfig>(
          &context_.scenario_config)) {
    AERROR << "fail to get specific config of scenario " << Name();
    return false;
  }
  init_ = true;
  return true;
}

bool TrafficLightUnprotectedRightTurnScenario::IsTransferable(
    const Scenario* const other_scenario, const Frame& frame) {
  // 进行条件检查:包含车道跟随指令
  if (!frame.local_view().planning_command->has_lane_follow_command()) {
    return false;
  }
  if (other_scenario == nullptr || frame.reference_line_info().empty()) {
    return false;
  }

  // 检查参考路径上是否存在特定类型的交通标志，并返回相应的重叠信息
  const auto& reference_line_info = frame.reference_line_info().front();
  const auto& first_encountered_overlaps =
      reference_line_info.FirstEncounteredOverlaps();
  hdmap::PathOverlap* traffic_sign_overlap = nullptr;
  for (const auto& overlap : first_encountered_overlaps) {
    // 若遇到停车标志或让行标志，直接返回 false
    if (overlap.first == ReferenceLineInfo::STOP_SIGN ||
        overlap.first == ReferenceLineInfo::YIELD_SIGN) {
      return false;
    } else if (overlap.first == ReferenceLineInfo::SIGNAL) {
      // 若遇到信号灯标志，记录其重叠信息并跳出循环
      traffic_sign_overlap = const_cast<hdmap::PathOverlap*>(&overlap.second);
      break;
    }
  }
  if (traffic_sign_overlap == nullptr) {
    // 如果未找到信号灯标志的重叠信息，返回 false
    return false;
  }

  // 获取交通信号灯相关的路径重叠信息，并计算自车前缘在参考线上的位置
  const std::vector<hdmap::PathOverlap>& traffic_light_overlaps =
      reference_line_info.reference_line().map_path().signal_overlaps();
  const double start_check_distance =
      context_.scenario_config.start_traffic_light_scenario_distance();
  const double adc_front_edge_s = reference_line_info.AdcSlBoundary().end_s();

  // find all the traffic light belong to
  // the same group as first encountered traffic light
  // 筛选出与交通标志重叠区域距离在 kTrafficLightGroupingMaxDist（2米）范围内的交通灯重叠区域，并将它们存储到 next_traffic_lights 向量中
  std::vector<hdmap::PathOverlap> next_traffic_lights;
  static constexpr double kTrafficLightGroupingMaxDist = 2.0;  // unit: m
  for (const auto& overlap : traffic_light_overlaps) {    // 遍历 traffic_light_overlaps 中的每个重叠区域 overlap
    const double dist = overlap.start_s - traffic_sign_overlap->start_s; // 计算 overlap 起始位置与 traffic_sign_overlap 起始位置的距离 dist
    if (fabs(dist) <= kTrafficLightGroupingMaxDist) {
      // 若距离绝对值小于等于 2 米，则将该 overlap 添加到 next_traffic_lights 中
      next_traffic_lights.push_back(overlap);
    }
  }

  // 判断车辆是否进入交通灯场景
  bool traffic_light_scenario = false;
  // note: need iterate all lights to check no RED/YELLOW/UNKNOWN
  for (const auto& overlap : next_traffic_lights) {
    // 遍历所有前方交通灯（next_traffic_lights），计算车辆与交通灯的距离
    const double adc_distance_to_traffic_light =
        overlap.start_s - adc_front_edge_s;
    ADEBUG << "traffic_light[" << overlap.object_id << "] start_s["
           << overlap.start_s << "] adc_distance_to_traffic_light["
           << adc_distance_to_traffic_light << "]";

    // enter traffic-light scenarios: based on distance only
    // 若距离小于等于0或超过检查范围（start_check_distance），跳过该交通灯
    if (adc_distance_to_traffic_light <= 0.0 ||
        adc_distance_to_traffic_light > start_check_distance) {
      continue;
    }

    const auto& signal_color = frame.GetSignal(overlap.object_id).color();
    ADEBUG << "traffic_light_id[" << overlap.object_id << "] start_s["
           << overlap.start_s << "] color[" << signal_color << "]";

    // 获取交通灯颜色，若为红色、黄色或未知，则标记为进入交通灯场景（traffic_light_scenario = true）并退出循环
    if (signal_color != perception::TrafficLight::GREEN &&
        signal_color != perception::TrafficLight::BLACK) {
      traffic_light_scenario = true;
      break;
    }
  }

  // 若traffic_light_scenario为空，则返回false
  if (!traffic_light_scenario) {
    return false;
  }

  // 获取路径在信号灯重叠起点处的转向类型，若不是右转（RIGHT_TURN），则返回false
  const auto& turn_type =
      reference_line_info.GetPathTurnType(traffic_sign_overlap->start_s);
  if (turn_type != hdmap::Lane::RIGHT_TURN) {
    return false;
  }
  context_.current_traffic_light_overlap_ids.clear(); // 清空当前交通信号灯重叠ID列表
  for (const auto& overlap : next_traffic_lights) {
    // 遍历下一个交通信号灯集合next_traffic_lights，将其对象ID添加到列表中
    context_.current_traffic_light_overlap_ids.push_back(overlap.object_id);
  }
  return true;
}

bool TrafficLightUnprotectedRightTurnScenario::Exit(Frame* frame) {
  injector_->planning_context()
      ->mutable_planning_status()
      ->mutable_traffic_light()
      ->Clear();
  return true;
}

bool TrafficLightUnprotectedRightTurnScenario::Enter(Frame* frame) {
  // 判断车辆是否进入“无保护右转通过红绿灯”场景
  const auto& reference_line_info = frame->reference_line_info().front();
  std::string current_traffic_light_overlap_id;
  const auto& overlaps = reference_line_info.FirstEncounteredOverlaps();
  for (auto overlap : overlaps) {
    // 遍历参考线上的重叠信息，查找类型为信号灯（ReferenceLineInfo::SIGNAL）的第一个重叠对象
    if (overlap.first == ReferenceLineInfo::SIGNAL) {
      current_traffic_light_overlap_id = overlap.second.object_id;
      break;
    }
  }

  // 检查交通灯是否存在
  if (current_traffic_light_overlap_id.empty()) {
    injector_->planning_context()
        ->mutable_planning_status()
        ->mutable_traffic_light()
        ->Clear();
    AERROR << "Can not find traffic light overlap in reference line!";
    return false;
  }

  // find all the traffic light at/within the same location/group
  // 查找与当前交通信号灯在同一位置或组的所有交通信号灯
  const std::vector<apollo::hdmap::PathOverlap>& traffic_light_overlaps =
      reference_line_info.reference_line().map_path().signal_overlaps(); // 获取参考线路中所有交通信号灯的重叠信息
  auto traffic_light_overlap_itr = std::find_if( // 使用 std::find_if 查找与 current_traffic_light_overlap_id 匹配的信号灯
      traffic_light_overlaps.begin(), traffic_light_overlaps.end(),
      [&current_traffic_light_overlap_id](const hdmap::PathOverlap& overlap) {
        return overlap.object_id == current_traffic_light_overlap_id;
      });
  if (traffic_light_overlap_itr == traffic_light_overlaps.end()) {
    injector_->planning_context()
        ->mutable_planning_status()
        ->mutable_traffic_light()
        ->Clear();
    return true;
  }

  // 历交通信号灯重叠区域，将距离当前信号灯在 kTrafficLightGroupingMaxDist（2米）范围内的信号灯加入规划上下文
  static constexpr double kTrafficLightGroupingMaxDist = 2.0;  // unit: m
  // 获取当前信号灯的起始位置 current_traffic_light_overlap_start_s
  const double current_traffic_light_overlap_start_s =
      traffic_light_overlap_itr->start_s;
  for (const auto& traffic_light_overlap : traffic_light_overlaps) {
    // 遍历所有信号灯重叠区域，计算与当前信号灯的距离 dist
    const double dist =
        traffic_light_overlap.start_s - current_traffic_light_overlap_start_s;
    if (fabs(dist) <= kTrafficLightGroupingMaxDist) {
      // 若距离小于等于2米，则将其ID添加到规划状态中，并记录日志
      injector_->planning_context()
          ->mutable_planning_status()
          ->mutable_traffic_light()
          ->add_current_traffic_light_overlap_id(
              traffic_light_overlap.object_id);
      ADEBUG << "Update PlanningContext with first_encountered traffic_light["
             << traffic_light_overlap.object_id << "] start_s["
             << traffic_light_overlap.start_s << "]";
    }
  }
  return true;
}

}  // namespace planning
}  // namespace apollo
