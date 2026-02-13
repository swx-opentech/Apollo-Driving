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

#include "modules/planning/scenarios/traffic_light_protected/stage_approach.h"

#include "cyber/common/log.h"
#include "modules/map/pnc_map/path.h"
#include "modules/planning/planning_base/common/frame.h"
#include "modules/planning/planning_base/common/planning_context.h"
#include "modules/planning/planning_base/common/util/util.h"
#include "modules/planning/scenarios/traffic_light_protected/context.h"

namespace apollo {
namespace planning {

using apollo::common::TrajectoryPoint;
using apollo::hdmap::PathOverlap;
using apollo::perception::TrafficLight;

StageResult TrafficLightProtectedStageApproach::Process(
    const TrajectoryPoint& planning_init_point, Frame* frame) {
  ADEBUG << "stage: Approach";
  CHECK_NOTNULL(frame);
  CHECK_NOTNULL(context_);

  // 执行交通灯保护场景下的规划任务
  auto context = GetContextAs<TrafficLightProtectedContext>();
  const ScenarioTrafficLightProtectedConfig& scenario_config =
      context->scenario_config;

  StageResult result = ExecuteTaskOnReferenceLine(planning_init_point, frame);
  if (result.HasError()) {
    AERROR << "TrafficLightProtectedStageApproach planning error";
  }

  // 检查 context->current_traffic_light_overlap_ids 是否为空，如果为空则调用 FinishScenario() 函数结束当前场景
  if (context->current_traffic_light_overlap_ids.empty()) {
    return FinishScenario();
  }

  const auto& reference_line_info = frame->reference_line_info().front();

  bool traffic_light_all_done = true;

  // 访问交通信号灯重叠区域的ID列表
  for (const auto& traffic_light_overlap_id :
       context->current_traffic_light_overlap_ids) {
    // get overlap along reference line
    // 获取参考线上与交通信号灯相关的重叠区域信息
    PathOverlap* current_traffic_light_overlap =
        reference_line_info.GetOverlapOnReferenceLine(
            traffic_light_overlap_id, ReferenceLineInfo::SIGNAL);
    if (!current_traffic_light_overlap) {
      // 如果未找到匹配的重叠区域（返回空指针），则跳过当前循环迭代
      continue;
    }

    // set right_of_way_status
    reference_line_info.SetJunctionRightOfWay(
        current_traffic_light_overlap->start_s, false); // 设置交叉口的路权状态为 false

    const double adc_front_edge_s = reference_line_info.AdcSlBoundary().end_s();
    const double distance_adc_to_stop_line =
        current_traffic_light_overlap->start_s - adc_front_edge_s; // 获取自车前缘位置 adc_front_edge_s，并计算其到停车线的距离
    auto signal_color = frame->GetSignal(traffic_light_overlap_id).color(); // 通过 frame->GetSignal 获取指定信号灯的颜色
    ADEBUG << "traffic_light_overlap_id[" << traffic_light_overlap_id
           << "] start_s[" << current_traffic_light_overlap->start_s
           << "] distance_adc_to_stop_line[" << distance_adc_to_stop_line
           << "] color[" << signal_color << "]";

    // check distance to stop line
    // 判断车辆与停止线的距离是否超过最大有效停车距离
    if (distance_adc_to_stop_line > scenario_config.max_valid_stop_distance()) {
      traffic_light_all_done = false;
      break;
    }

    // check on traffic light color
    // 检查交通信号灯的颜色
    if (signal_color != TrafficLight::GREEN) {
      traffic_light_all_done = false;
      break;
    }
  }

  if (traffic_light_all_done) {
    return FinishStage();
  }

  return result.SetStageStatus(StageStatusType::RUNNING);
}

StageResult TrafficLightProtectedStageApproach::FinishStage() {
  auto context = GetContextAs<TrafficLightProtectedContext>();  // 获取当前上下文 TrafficLightProtectedContext
  auto* traffic_light = injector_->planning_context()
                            ->mutable_planning_status()
                            ->mutable_traffic_light();
  traffic_light->clear_done_traffic_light_overlap_id(); // 清除已完成的交通信号灯重叠区域ID
  for (const auto& traffic_light_overlap_id :
       context->current_traffic_light_overlap_ids) {
    // 将当前上下文中的交通信号灯重叠区域ID添加到已完成列表中
    traffic_light->add_done_traffic_light_overlap_id(traffic_light_overlap_id);
  }

  // 设置下一阶段为 "TRAFFIC_LIGHT_PROTECTED_INTERSECTION_CRUISE"
  next_stage_ = "TRAFFIC_LIGHT_PROTECTED_INTERSECTION_CRUISE";
  return StageResult(StageStatusType::FINISHED);
}

}  // namespace planning
}  // namespace apollo
