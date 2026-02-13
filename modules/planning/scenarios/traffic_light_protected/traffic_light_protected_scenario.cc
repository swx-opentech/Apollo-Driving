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

#include "modules/planning/scenarios/traffic_light_protected/traffic_light_protected_scenario.h"

#include "modules/planning/planning_base/proto/planning_config.pb.h"

#include "cyber/common/log.h"
#include "modules/planning/planning_base/common/frame.h"
#include "modules/planning/planning_base/common/planning_context.h"
#include "modules/planning/scenarios/traffic_light_protected/stage_approach.h"
#include "modules/planning/scenarios/traffic_light_protected/stage_intersection_cruise.h"

namespace apollo {
namespace planning {

using apollo::hdmap::HDMapUtil;

bool TrafficLightProtectedScenario::Init(
    std::shared_ptr<DependencyInjector> injector, const std::string& name) {
  if (init_) {
    return true;
  }

  if (!Scenario::Init(injector, name)) {
    AERROR << "failed to init scenario" << Name();
    return false;
  }

  if (!Scenario::LoadConfig<ScenarioTrafficLightProtectedConfig>(
          &context_.scenario_config)) {
    AERROR << "fail to get specific config of scenario " << Name();
    return false;
  }
  init_ = true;
  return true;
}

bool TrafficLightProtectedScenario::IsTransferable(
    const Scenario* const other_scenario, const Frame& frame) {
  
  // 判断当前场景是否满足继续执行的条件
  if (!frame.local_view().planning_command->has_lane_follow_command()) {
    return false;
  }
  if (other_scenario == nullptr || frame.reference_line_info().empty()) {
    return false;
  }

  // 遍历路径上的重叠信息，检查是否存在停车标志、让行标志或信号灯
  const auto& reference_line_info = frame.reference_line_info().front();
  const auto& first_encountered_overlaps =
      reference_line_info.FirstEncounteredOverlaps();
  hdmap::PathOverlap* traffic_sign_overlap = nullptr;
  for (const auto& overlap : first_encountered_overlaps) {
    if (overlap.first == ReferenceLineInfo::STOP_SIGN ||
        overlap.first == ReferenceLineInfo::YIELD_SIGN) {
      return false;
    } else if (overlap.first == ReferenceLineInfo::SIGNAL) {
      traffic_sign_overlap = const_cast<hdmap::PathOverlap*>(&overlap.second);
      break;
    }
  }
  if (traffic_sign_overlap == nullptr) {
    return false;
  }
  
  // 获取交通信号灯相关的路径重叠信息，并计算自车前缘在参考线上的位置
  const std::vector<hdmap::PathOverlap>& traffic_light_overlaps =
      reference_line_info.reference_line().map_path().signal_overlaps(); // 从参考线中提取所有与交通信号灯重叠的路径段
  const double start_check_distance =
      context_.scenario_config.start_traffic_light_scenario_distance(); // 场景配置中读取开始检测交通信号灯的距离阈值
  const double adc_front_edge_s = reference_line_info.AdcSlBoundary().end_s(); // 获取自车（ADC）前缘在参考线上的纵向位置（s坐标）
  
  // find all the traffic light belong to
  // the same group as first encountered traffic light
  // 查找与第一个遇到的交通信号灯属于同一组的所有交通信号灯
  std::vector<hdmap::PathOverlap> next_traffic_lights;
  static constexpr double kTrafficLightGroupingMaxDist = 2.0;  // unit: m
  for (const auto& overlap : traffic_light_overlaps) {
    // 遍历 traffic_light_overlaps 中的所有交通信号灯重叠区域
    const double dist = overlap.start_s - traffic_sign_overlap->start_s;
    if (fabs(dist) <= kTrafficLightGroupingMaxDist) {
      // 距离小于等于阈值 kTrafficLightGroupingMaxDist（2米），则将其加入 next_traffic_lights 向量
      next_traffic_lights.push_back(overlap);
    }
  }

  // 判断车辆是否进入交通灯场景
  bool traffic_light_scenario = false;
  // note: need iterate all lights to check no RED/YELLOW/UNKNOWN
  for (const auto& overlap : next_traffic_lights) {    // 遍历所有前方交通灯（next_traffic_lights）
    // 计算车辆与交通灯的距离，若距离不在有效范围内则跳过(不进入交通灯场景)。
    const double adc_distance_to_traffic_light =
        overlap.start_s - adc_front_edge_s;
    ADEBUG << "traffic_light[" << overlap.object_id << "] start_s["
           << overlap.start_s << "] adc_distance_to_traffic_light["
           << adc_distance_to_traffic_light << "]";

    // enter traffic-light scenarios: based on distance only
    if (adc_distance_to_traffic_light <= 0.0 ||
        adc_distance_to_traffic_light > start_check_distance) {
      continue;
    }

    // 检查交通灯颜色，若为红色、黄色或未知，则标记为交通灯场景
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
    // 若未检测到需响应的交通灯，返回 false
    return false;
  }

  // 更新当前交通灯的重叠ID列表
  context_.current_traffic_light_overlap_ids.clear();
  for (const auto& overlap : next_traffic_lights) {
    context_.current_traffic_light_overlap_ids.push_back(overlap.object_id);
  }
  return true;
}

bool TrafficLightProtectedScenario::Exit(Frame* frame) {
  injector_->planning_context()
      ->mutable_planning_status()
      ->mutable_traffic_light()
      ->Clear();
  return true;
}

bool TrafficLightProtectedScenario::Enter(Frame* frame) {
  injector_->planning_context()
      ->mutable_planning_status()
      ->mutable_traffic_light()
      ->Clear();
  if (context_.current_traffic_light_overlap_ids.empty()) {
    AERROR << "Can not find traffic light overlap in reference line!";
    return false;
  }
  for (const auto& overlap_id : context_.current_traffic_light_overlap_ids) {
    injector_->planning_context()
        ->mutable_planning_status()
        ->mutable_traffic_light()
        ->add_current_traffic_light_overlap_id(overlap_id);
  }
  return true;
}

}  // namespace planning
}  // namespace apollo
