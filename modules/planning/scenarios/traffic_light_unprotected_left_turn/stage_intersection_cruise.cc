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

#include "modules/planning/scenarios/traffic_light_unprotected_left_turn/stage_intersection_cruise.h"

#include "cyber/common/log.h"
#include "modules/planning/planning_base/common/planning_context.h"
#include "modules/planning/scenarios/traffic_light_unprotected_left_turn/context.h"

namespace apollo {
namespace planning {

StageResult TrafficLightUnprotectedLeftTurnStageIntersectionCruise::Process(
    const common::TrajectoryPoint& planning_init_point, Frame* frame) {
  ADEBUG << "stage: IntersectionCruise";
  CHECK_NOTNULL(frame);

  // 调用 ExecuteTaskOnReferenceLine 执行规划任务，若出错则记录错误日志
  StageResult result = ExecuteTaskOnReferenceLine(planning_init_point, frame);
  if (result.HasError()) {
    AERROR << "TrafficLightUnprotectedLeftTurnStageIntersectionCruise "
           << "plan error";
  }

  // 调用 CheckDone 函数，传入帧数据、规划上下文和布尔值 true，判断当前阶段是否已完成，结果存储在 stage_done 中
  bool stage_done = CheckDone(*frame, injector_->planning_context(), true);
  if (stage_done) {
    return FinishStage();
  }
  return result.SetStageStatus(StageStatusType::RUNNING);
}

StageResult
TrafficLightUnprotectedLeftTurnStageIntersectionCruise::FinishStage() {
  return FinishScenario();
}

}  // namespace planning
}  // namespace apollo
