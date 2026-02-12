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
 * @file stage_intersection_cruise.cc
 **/

#include "modules/planning/scenarios/stop_sign_unprotected/stage_intersection_cruise.h"

#include "cyber/common/log.h"
#include "modules/planning/planning_base/common/planning_context.h"
#include "modules/planning/scenarios/stop_sign_unprotected/context.h"

namespace apollo {
namespace planning {

StageResult StopSignUnprotectedStageIntersectionCruise::Process(
    const common::TrajectoryPoint& planning_init_point, Frame* frame) {
  ADEBUG << "stage: IntersectionCruise";
  CHECK_NOTNULL(frame);

  StageResult result = ExecuteTaskOnReferenceLine(planning_init_point, frame); // 执行参考线任务
  if (result.HasError()) {
    AERROR << "StopSignUnprotectedStageIntersectionCruise plan error";
  }

  // 调用 CheckDone 函数判断阶段是否完成，参数包括帧数据、规划上下文和一个布尔值
  bool stage_done = CheckDone(*frame, injector_->planning_context(), false);
  if (stage_done) {
    // 如果阶段已完成（stage_done 为真），调用 FinishStage() 结束当前阶段
    return FinishStage();
  }
  // 若未完成，则通过 result.SetStageStatus() 将阶段状态设为运行中（RUNNING）并返回
  return result.SetStageStatus(StageStatusType::RUNNING);
}

StageResult StopSignUnprotectedStageIntersectionCruise::FinishStage() {
  return FinishScenario();
}

hdmap::PathOverlap*
StopSignUnprotectedStageIntersectionCruise::GetTrafficSignOverlap(
    const ReferenceLineInfo& reference_line_info,
    const PlanningContext* context) const {
  // stop_sign scenarios
  const auto& stop_sign_status = context->planning_status().stop_sign();
  const std::string traffic_sign_overlap_id =
      stop_sign_status.current_stop_sign_overlap_id();
  hdmap::PathOverlap* traffic_sign_overlap =
      reference_line_info.GetOverlapOnReferenceLine(
          traffic_sign_overlap_id, ReferenceLineInfo::STOP_SIGN);
  return traffic_sign_overlap;
}

}  // namespace planning
}  // namespace apollo
