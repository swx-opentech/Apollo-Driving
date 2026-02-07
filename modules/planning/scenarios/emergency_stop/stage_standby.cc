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

#include "modules/planning/scenarios/emergency_stop/stage_standby.h"

#include <string>
#include <vector>

#include "cyber/common/log.h"
#include "modules/planning/planning_base/common/frame.h"
#include "modules/planning/planning_base/common/planning_context.h"
#include "modules/planning/planning_base/common/util/common.h"

namespace apollo {
namespace planning {

using apollo::common::TrajectoryPoint;


// 获取场景上下文
StageResult EmergencyStopStageStandby::Process(
    const TrajectoryPoint& planning_init_point, Frame* frame) {
  ADEBUG << "stage: Standby";
  CHECK_NOTNULL(frame);

  scenario_config_.CopyFrom(
      GetContextAs<EmergencyStopContext>()->scenario_config);

  // add a stop fence
  // 获取车辆参考线信息
  const auto& reference_line_info = frame->reference_line_info().front();
  const auto& reference_line = reference_line_info.reference_line();
  const double adc_front_edge_s = reference_line_info.AdcSlBoundary().end_s();
  const double stop_distance = scenario_config_.stop_distance();

  bool stop_fence_exist = false; // 初始化，表示默认没有停止线。
  double stop_line_s;
  // 获取规划状态中的紧急停车信息
  const auto& emergency_stop_status =
      injector_->planning_context()->planning_status().emergency_stop();

  // 如果紧急停车状态包含停止线点，则将其从世界坐标系转换为参考线的 SL 坐标系
  if (emergency_stop_status.has_stop_fence_point()) {
    common::SLPoint stop_fence_sl;
    reference_line.XYToSL(emergency_stop_status.stop_fence_point(),
                          &stop_fence_sl);
    // 若转换后的停止线位置大于自车前缘位置，则标记存在停止线，并记录其 s 值
    if (stop_fence_sl.s() > adc_front_edge_s) {
      stop_fence_exist = true;
      stop_line_s = stop_fence_sl.s();
    }
  }

  // 设置紧急停车的停止线位置
  if (!stop_fence_exist) {
    static constexpr double kBuffer = 2.0;
    stop_line_s = adc_front_edge_s + stop_distance + kBuffer;
    const auto& stop_fence_point =
        reference_line.GetReferencePoint(stop_line_s);
    auto* emergency_stop_fence_point = injector_->planning_context()
                                           ->mutable_planning_status()
                                           ->mutable_emergency_stop()
                                           ->mutable_stop_fence_point();
    emergency_stop_fence_point->set_x(stop_fence_point.x());
    emergency_stop_fence_point->set_y(stop_fence_point.y());
  }

  // 构建一个紧急停车决策
  const std::string virtual_obstacle_id = "EMERGENCY_STOP";
  const std::vector<std::string> wait_for_obstacle_ids;
  planning::util::BuildStopDecision(
      virtual_obstacle_id, stop_line_s, stop_distance,
      StopReasonCode::STOP_REASON_EMERGENCY, wait_for_obstacle_ids,
      "EMERGENCY_STOP-scenario", frame,
      &(frame->mutable_reference_line_info()->front()));
  ADEBUG << "Build a stop fence for emergency_stop: id[" << virtual_obstacle_id
         << "] s[" << stop_line_s << "]";

  // 执行规划任务并根据结果决定是否记录错误信息
  StageResult result = ExecuteTaskOnReferenceLine(planning_init_point, frame);
  if (result.HasError()) {
    AERROR << "EmergencyStopStageStandby planning error";
  }

  const auto& pad_msg_driving_action = frame->GetPadMsgDrivingAction();
  if (pad_msg_driving_action != PadMessage::STOP) {
    return FinishStage();
  }

  return result.SetStageStatus(StageStatusType::RUNNING);
}

StageResult EmergencyStopStageStandby::FinishStage() {
  return FinishScenario();
}

}  // namespace planning
}  // namespace apollo
