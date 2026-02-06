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

#include "modules/planning/scenarios/emergency_pull_over/stage_standby.h"

#include <memory>
#include <string>
#include <vector>

#include "cyber/common/log.h"
#include "modules/planning/planning_base/common/frame.h"
#include "modules/planning/planning_base/common/planning_context.h"
#include "modules/planning/planning_base/common/util/common.h"

namespace apollo {
namespace planning {

using apollo::common::TrajectoryPoint;
using apollo::common::VehicleConfigHelper;
using apollo::common::VehicleSignal;

StageResult EmergencyPullOverStageStandby::Process(
  // 获取场景上下文
    const TrajectoryPoint& planning_init_point, Frame* frame) {
  ADEBUG << "stage: Standby";
  CHECK_NOTNULL(frame);

  scenario_config_.CopyFrom(
      GetContextAs<EmergencyPullOverContext>()->scenario_config);

  auto& reference_line_info = frame->mutable_reference_line_info()->front();

  // set vehicle signal
  // 设置紧急灯信号并清除转向灯信号、限速
  reference_line_info.SetEmergencyLight();
  reference_line_info.SetTurnSignal(VehicleSignal::TURN_NONE);

  // reset cruise_speed
  reference_line_info.LimitCruiseSpeed(FLAGS_default_cruise_speed);

  // add a stop fence
  // 获取规划状态中的靠边停车位置信息
  const auto& pull_over_status =
      injector_->planning_context()->planning_status().pull_over();

  // 检查停车位置的坐标有效性（x、y坐标是否存在）
  if (pull_over_status.has_position() && pull_over_status.position().has_x() &&
      pull_over_status.position().has_y()) {
    // 获取当前参考线信息
    const auto& reference_line_info = frame->reference_line_info().front();
    const auto& reference_line = reference_line_info.reference_line();
    common::SLPoint pull_over_sl;

    // 笛卡尔坐标转经纬度坐标
    reference_line.XYToSL(pull_over_status.position(), &pull_over_sl);
    const double stop_distance = scenario_config_.stop_distance();
    // 计算停止线位置：当前位置+停止距离+车辆前缘到中心距离
    double stop_line_s =
        pull_over_sl.s() + stop_distance +
        VehicleConfigHelper::GetConfig().vehicle_param().front_edge_to_center();
    const double adc_front_edge_s = reference_line_info.AdcSlBoundary().end_s();
    double distance = stop_line_s - adc_front_edge_s;
    // 计算距离差值，如为负则调整停止线位置确保安全距离
    if (distance <= 0.0) {
      // push stop fence further
      stop_line_s = adc_front_edge_s + stop_distance;
    }

    // 定义虚拟障碍物ID为"EMERGENCY_PULL_OVER"；创建空的等待障碍物ID列表；
    const std::string virtual_obstacle_id = "EMERGENCY_PULL_OVER";
    const std::vector<std::string> wait_for_obstacle_ids;
    
    // 在指定位置设置停车线
    planning::util::BuildStopDecision(
        virtual_obstacle_id, stop_line_s, stop_distance,
        StopReasonCode::STOP_REASON_PULL_OVER, wait_for_obstacle_ids,
        "EMERGENCY_PULL_OVER-scenario", frame,
        &(frame->mutable_reference_line_info()->front()));

    ADEBUG << "Build a stop fence for emergency_pull_over: id["
           << virtual_obstacle_id << "] s[" << stop_line_s << "]";
  }

  // 执行参考线上的规划任务
  StageResult result = ExecuteTaskOnReferenceLine(planning_init_point, frame);
  if (result.HasError()) {
    AERROR << "EmergencyPullOverStageStandby planning error";
  }

  // 检查当前帧的驾驶动作是否为"靠边停车"
  const auto& pad_msg_driving_action = frame->GetPadMsgDrivingAction();
  if (pad_msg_driving_action != PadMessage::PULL_OVER) {
    return FinishStage();
  }

  return result.SetStageStatus(StageStatusType::RUNNING);
}

StageResult EmergencyPullOverStageStandby::FinishStage() {
  return FinishScenario();
}

}  // namespace planning
}  // namespace apollo
