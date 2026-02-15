/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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

#include "modules/planning/scenarios/traffic_light_unprotected_left_turn/stage_approach.h"

#include "modules/common_msgs/perception_msgs/perception_obstacle.pb.h"
#include "modules/common_msgs/perception_msgs/traffic_light_detection.pb.h"
#include "cyber/common/log.h"
#include "cyber/time/clock.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/planning/planning_base/common/frame.h"
#include "modules/planning/planning_base/common/planning_context.h"
#include "modules/planning/planning_base/common/speed_profile_generator.h"
#include "modules/planning/scenarios/traffic_light_unprotected_left_turn/context.h"

namespace apollo {
namespace planning {

using apollo::common::TrajectoryPoint;
using apollo::cyber::Clock;
using apollo::hdmap::PathOverlap;
using apollo::perception::TrafficLight;

StageResult TrafficLightUnprotectedLeftTurnStageApproach::Process(
    const TrajectoryPoint& planning_init_point, Frame* frame) {
  ADEBUG << "stage: Approach";
  CHECK_NOTNULL(frame);
  CHECK_NOTNULL(context_);

  // 获取当前场景的上下文对象 TrafficLightUnprotectedLeftTurnContext
  auto context = GetContextAs<TrafficLightUnprotectedLeftTurnContext>();
  const ScenarioTrafficLightUnprotectedLeftTurnConfig& scenario_config =
      context->scenario_config; // 提取配置信息 ScenarioTrafficLightUnprotectedLeftTurnConfig
  // 检查流水线配置是否启用
  if (!pipeline_config_.enabled()) {
    return FinishStage(frame);
  }

  /* 处理交通灯无保护左转场景中的巡航速度控制和路径规划 */
  // set cruise_speed to slow down
  // 通过LimitCruiseSpeed将巡航速度设为接近路口时的较低值
  frame->mutable_reference_line_info()->front().LimitCruiseSpeed(
      scenario_config.approach_cruise_speed());

  // 执行路径规划任务：调用ExecuteTaskOnReferenceLine进行参考线上的规划
  StageResult result = ExecuteTaskOnReferenceLine(planning_init_point, frame);
  if (result.HasError()) {
    AERROR << "TrafficLightUnprotectedLeftTurnStageApproach planning error";
  }

  if (context->current_traffic_light_overlap_ids.empty()) { // 交通灯重叠区域
    return FinishScenario();
  }

  const auto& reference_line_info = frame->reference_line_info().back(); // 获取参考线信息

  const double adc_front_edge_s = reference_line_info.AdcSlBoundary().end_s(); // 计算车辆前缘位置

  PathOverlap* traffic_light = nullptr; // 初始化指针 traffic_light 为 nullptr
  bool traffic_light_all_done = true;
  for (const auto& traffic_light_overlap_id :
       context->current_traffic_light_overlap_ids) {
    // 遍历 context->current_traffic_light_overlap_ids 中的每个 ID
    // get overlap along reference line
    PathOverlap* current_traffic_light_overlap =
        reference_line_info.GetOverlapOnReferenceLine(
            traffic_light_overlap_id, ReferenceLineInfo::SIGNAL); // 调用 GetOverlapOnReferenceLine 获取参考线上与该 ID 对应的信号灯重叠对象
    if (!current_traffic_light_overlap) {
      // 若未找到有效重叠对象（返回空指针），则跳过当前循环
      continue;
    }

    traffic_light = current_traffic_light_overlap; // 更新交通信号灯的状态，使其等于当前重叠区域的信号灯状态

    // set right_of_way_status
    reference_line_info.SetJunctionRightOfWay(
        current_traffic_light_overlap->start_s, false); // 标识当前交通灯控制的路口为非优先通行状态

    const double distance_adc_to_stop_line =
        current_traffic_light_overlap->start_s - adc_front_edge_s; // 自车前缘到停车线的距离
    auto signal_color = frame->GetSignal(traffic_light_overlap_id).color(); // 获取当前交通灯的颜色
    ADEBUG << "traffic_light_overlap_id[" << traffic_light_overlap_id
           << "] start_s[" << current_traffic_light_overlap->start_s
           << "] distance_adc_to_stop_line[" << distance_adc_to_stop_line
           << "] color[" << signal_color << "]";

    if (distance_adc_to_stop_line < 0) return FinishStage(frame); // 如果距离停止线小于0，调用FinishStage结束当前阶段
    // check on traffic light color and distance to stop line
    // 判断交通信号灯状态和车辆距离停车线的距离是否满足继续行驶的条件
    if (signal_color != TrafficLight::GREEN || // 如果信号灯不是绿色
        distance_adc_to_stop_line >=
            scenario_config.max_valid_stop_distance()) { // 车辆到停车线的距离大于允许的最大有效停车距离
      traffic_light_all_done = false; // 则将标志位 traffic_light_all_done 设为 false 并跳出循环
      break;
    }
  }

  if (traffic_light == nullptr) {
    return FinishScenario(); // 如果 traffic_light 为空，则调用 FinishScenario() 结束整个场景
  }

  if (traffic_light_all_done) {
    return FinishStage(frame); // 如果所有交通灯任务已完成（traffic_light_all_done 为真），则调用 FinishStage(frame) 结束当前阶段
  }

  return result.SetStageStatus(StageStatusType::RUNNING);
}

StageResult TrafficLightUnprotectedLeftTurnStageApproach::FinishStage(
    Frame* frame) {
  auto context = GetContextAs<TrafficLightUnprotectedLeftTurnContext>(); // 获取上下文对象
  const ScenarioTrafficLightUnprotectedLeftTurnConfig& scenario_config =
      context->scenario_config; // 访问配置信息

  // check speed at stop_stage
  // 根据车辆速度决定是否进入“缓行”（creep）阶段
  const double adc_speed = injector_->vehicle_state()->linear_velocity(); // 获取当前车速 adc_speed
  if (adc_speed > scenario_config.max_adc_speed_before_creep()) {
    // skip creep
    // 若车速超过设定阈值，则跳过缓行，直接进入巡航阶段
    next_stage_ = "TRAFFIC_LIGHT_UNPROTECTED_LEFT_TURN_INTERSECTION_CRUISE";
  } else {
    // 清空并更新交通灯相关的规划状态，并记录缓行开始时间，进入缓行阶段
    // creep
    // update PlanningContext
    injector_->planning_context()
        ->mutable_planning_status()
        ->mutable_traffic_light()
        ->mutable_done_traffic_light_overlap_id()
        ->Clear();
    for (const auto& traffic_light_overlap_id :
         context->current_traffic_light_overlap_ids) {
      injector_->planning_context()
          ->mutable_planning_status()
          ->mutable_traffic_light()
          ->add_done_traffic_light_overlap_id(traffic_light_overlap_id);
    }

    context->creep_start_time = Clock::NowInSeconds();
    next_stage_ = "TRAFFIC_LIGHT_UNPROTECTED_LEFT_TURN_CREEP";
  }

  // reset cruise_speed
  auto& reference_line_info = frame->mutable_reference_line_info()->front(); // 获取当前帧（frame）中的第一条参考线信息（reference_line_info）
  reference_line_info.LimitCruiseSpeed(FLAGS_default_cruise_speed); // 调用 LimitCruiseSpeed 方法，将巡航速度限制为默认值

  return StageResult(StageStatusType::FINISHED); // 返回 StageResult 对象，表示当前阶段已经完成
}

}  // namespace planning
}  // namespace apollo
