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
#include "modules/planning/scenarios/traffic_light_unprotected_right_turn/stage_intersection_cruise.h"

#include "cyber/common/log.h"
#include "modules/planning/scenarios/traffic_light_unprotected_right_turn/context.h"

namespace apollo {
namespace planning {

StageResult TrafficLightUnprotectedRightTurnStageIntersectionCruise::Process(
    const common::TrajectoryPoint& planning_init_point, Frame* frame) {
  // 输出调试信息并确保frame指针非空
  ADEBUG << "stage: IntersectionCruise";
  CHECK_NOTNULL(frame);

  // 调用ExecuteTaskOnReferenceLine在参考线上执行规划任务
  StageResult result = ExecuteTaskOnReferenceLine(planning_init_point, frame);
  if (result.HasError()) {
    AERROR << "TrafficLightUnprotectedRightTurnStageIntersectionCruise "
           << "plan error";
  }

  // 通过CheckDone判断当前阶段是否已完成
  bool stage_done = CheckDone(*frame, injector_->planning_context(), false);
  if (stage_done) {
    // 若阶段完成，调用FinishStage()结束该阶段
    return FinishStage();
  }
  // 否则返回运行状态（RUNNING）
  return result.SetStageStatus(StageStatusType::RUNNING);
}

StageResult
TrafficLightUnprotectedRightTurnStageIntersectionCruise::FinishStage() {
  return FinishScenario();
}

}  // namespace planning
}  // namespace apollo
