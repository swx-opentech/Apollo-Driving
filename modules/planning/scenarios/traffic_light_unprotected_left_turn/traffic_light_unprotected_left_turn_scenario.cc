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

#include "modules/planning/scenarios/traffic_light_unprotected_left_turn/traffic_light_unprotected_left_turn_scenario.h"

#include "modules/common_msgs/perception_msgs/perception_obstacle.pb.h"
#include "modules/common_msgs/perception_msgs/traffic_light_detection.pb.h"
#include "modules/planning/planning_base/proto/planning_config.pb.h"
#include "cyber/common/log.h"
#include "cyber/time/clock.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/planning/planning_base/common/frame.h"
#include "modules/planning/planning_base/common/planning_context.h"
#include "modules/planning/scenarios/traffic_light_unprotected_left_turn/stage_approach.h"
#include "modules/planning/scenarios/traffic_light_unprotected_left_turn/stage_creep.h"
#include "modules/planning/scenarios/traffic_light_unprotected_left_turn/stage_intersection_cruise.h"

namespace apollo {
namespace planning {

using apollo::hdmap::HDMapUtil;

bool TrafficLightUnprotectedLeftTurnScenario::Init(
    std::shared_ptr<DependencyInjector> injector, const std::string& name) {
  if (init_) {
    return true;
  }

  if (!Scenario::Init(injector, name)) {
    AERROR << "failed to init scenario" << Name();
    return false;
  }

  if (!Scenario::LoadConfig<ScenarioTrafficLightUnprotectedLeftTurnConfig>(
          &context_.scenario_config)) {
    AERROR << "fail to get specific config of scenario " << Name();
    return false;
  }
  init_ = true;
  return true;
}

bool TrafficLightUnprotectedLeftTurnScenario::IsTransferable(
    const Scenario* const other_scenario, const Frame& frame) {
  // 检查 frame.local_view().planning_command 是否包含车道跟随指令，若无则返回 false
  if (!frame.local_view().planning_command->has_lane_follow_command()) {
    return false;
  }
  if (other_scenario == nullptr || frame.reference_line_info().empty()) {
    return false;
  }

  // 检查参考线信息中是否存在首次遇到的重叠区域
  const auto& reference_line_info = frame.reference_line_info().front();
  const auto& first_encountered_overlaps =
      reference_line_info.FirstEncounteredOverlaps();
  if (first_encountered_overlaps.empty()) {
    return false;
  }

  // 筛选出信号灯相关的路径重叠区域
  hdmap::PathOverlap* traffic_sign_overlap = nullptr;
  for (const auto& overlap : first_encountered_overlaps) {
    // 遍历 first_encountered_overlaps，若遇到停车标志（STOP_SIGN）或让行标志（YIELD_SIGN），直接返回 false
    if (overlap.first == ReferenceLineInfo::STOP_SIGN ||
        overlap.first == ReferenceLineInfo::YIELD_SIGN) {
      return false;
    } else if (overlap.first == ReferenceLineInfo::SIGNAL) {
      // 若遇到信号灯标志（SIGNAL），记录其对应的重叠区域并跳出循环
      traffic_sign_overlap = const_cast<hdmap::PathOverlap*>(&overlap.second);
      break;
    }
  }
  if (traffic_sign_overlap == nullptr) {
    // 若未找到信号灯标志，则返回 false
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
  // 筛选出与交通标志重叠区域距离在2米以内的交通灯重叠区域，并将它们存储到next_traffic_lights向量中
  std::vector<hdmap::PathOverlap> next_traffic_lights;
  static constexpr double kTrafficLightGroupingMaxDist = 2.0;  // unit: m
  for (const auto& overlap : traffic_light_overlaps) {
    // 遍历traffic_light_overlaps中的每个重叠区域overlap
    const double dist = overlap.start_s - traffic_sign_overlap->start_s; // 计算当前交通灯重叠区域与交通标志重叠区域起点的距离dist
    if (fabs(dist) <= kTrafficLightGroupingMaxDist) {
      // 若距离绝对值小于等于2米，则将该交通灯重叠区域加入结果列表next_traffic_lights
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
    if (adc_distance_to_traffic_light <= 0.0 ||
        adc_distance_to_traffic_light > start_check_distance) {
        // 若距离小于等于0或超过检查范围（start_check_distance），跳过该交通灯
      continue;
    }

    // 获取交通灯颜色，若为红色、黄色或未知，则标记为进入交通灯场景（traffic_light_scenario = true）并退出循环
    const auto& signal_color = frame.GetSignal(overlap.object_id).color();
    ADEBUG << "traffic_light_id[" << overlap.object_id << "] start_s["
           << overlap.start_s << "] color[" << signal_color << "]";

    if (signal_color != perception::TrafficLight::GREEN &&
        signal_color != perception::TrafficLight::BLACK) {
      traffic_light_scenario = true;
      break;
    }
  }
  if (!traffic_light_scenario) {
    return false;
  }

  // 获取路径在交通标志重叠起点处的转向类型，若不是左转，则返回 false
  const auto& turn_type =
      reference_line_info.GetPathTurnType(traffic_sign_overlap->start_s);
  if (turn_type != hdmap::Lane::LEFT_TURN) {
    return false;
  }
  context_.current_traffic_light_overlap_ids.clear();
  for (const auto& overlap : next_traffic_lights) {
    // 遍历下一个交通灯集合，将每个交通灯的ID添加到列表中
    context_.current_traffic_light_overlap_ids.push_back(overlap.object_id);
  }

  return true;
}

bool TrafficLightUnprotectedLeftTurnScenario::Exit(Frame* frame) {
  injector_->planning_context()
      ->mutable_planning_status()
      ->mutable_traffic_light()
      ->Clear();
  return true;
}

bool TrafficLightUnprotectedLeftTurnScenario::Enter(Frame* frame) {
  const auto& reference_line_info = frame->reference_line_info().front(); // 获取当前参考线信息中的第一个重叠对象集合
  std::string current_traffic_light_overlap_id;
  const auto& overlaps = reference_line_info.FirstEncounteredOverlaps();
  for (auto overlap : overlaps) {
    // 遍历这些重叠对象，查找类型为信号灯（SIGNAL）的对象
    if (overlap.first == ReferenceLineInfo::SIGNAL) {
      current_traffic_light_overlap_id = overlap.second.object_id;
      break;
    }
  }
  // 找到后，记录其ID并跳出循环

  // 检查当前交通信号灯重叠区域ID是否为空
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
      reference_line_info.reference_line().map_path().signal_overlaps(); // 获取参考线上的所有交通信号灯重叠信息
  // 使用 std::find_if 查找与 current_traffic_light_overlap_id 匹配的信号灯
  auto traffic_light_overlap_itr = std::find_if(
      traffic_light_overlaps.begin(), traffic_light_overlaps.end(),
      [&current_traffic_light_overlap_id](const hdmap::PathOverlap& overlap) {
        return overlap.object_id == current_traffic_light_overlap_id;
      });
  // 如果未找到匹配项，则清空规划状态中的交通信号灯信息并返回 true
  if (traffic_light_overlap_itr == traffic_light_overlaps.end()) {
    injector_->planning_context()
        ->mutable_planning_status()
        ->mutable_traffic_light()
        ->Clear();
    return true;
  }

  // 将距离当前交通信号灯一定范围内的其他交通信号灯加入规划上下
  static constexpr double kTrafficLightGroupingMaxDist = 2.0;  // unit: m
  const double current_traffic_light_overlap_start_s =
      traffic_light_overlap_itr->start_s;
  for (const auto& traffic_light_overlap : traffic_light_overlaps) {
    // 遍历所有交通信号灯重叠区域（traffic_light_overlaps）
    // 计算每个信号灯与当前信号灯的距离（dist）
    const double dist =
        traffic_light_overlap.start_s - current_traffic_light_overlap_start_s;
    if (fabs(dist) <= kTrafficLightGroupingMaxDist) {
      // 若距离小于等于最大分组距离（kTrafficLightGroupingMaxDist），则将其ID添加到规划状态中，并记录日志
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
