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

#include "modules/planning/scenarios/traffic_light_unprotected_right_turn/stage_creep.h"

#include <string>

#include "modules/common_msgs/perception_msgs/perception_obstacle.pb.h"
#include "modules/common_msgs/perception_msgs/traffic_light_detection.pb.h"
#include "cyber/common/log.h"
#include "cyber/time/clock.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/map/pnc_map/path.h"
#include "modules/planning/planning_base/common/frame.h"
#include "modules/planning/planning_base/common/planning_context.h"
#include "modules/planning/planning_base/common/speed_profile_generator.h"
#include "modules/planning/planning_base/common/util/util.h"
#include "modules/planning/scenarios/traffic_light_unprotected_right_turn/context.h"

namespace apollo {
namespace planning {

using apollo::common::TrajectoryPoint;
using apollo::cyber::Clock;
using apollo::hdmap::PathOverlap;

bool TrafficLightUnprotectedRightTurnStageCreep::Init(
    const StagePipeline& config,
    const std::shared_ptr<DependencyInjector>& injector,
    const std::string& config_dir, void* context) {
  CHECK_NOTNULL(context);
  bool ret = Stage::Init(config, injector, config_dir, context);
  if (!ret) {
    AERROR << Name() << "init failed!";
    return false;
  }
  return ret;
}

StageResult TrafficLightUnprotectedRightTurnStageCreep::Process(
    const TrajectoryPoint& planning_init_point, Frame* frame) {
  ADEBUG << "stage: Creep";
  CHECK_NOTNULL(frame);
  CHECK_NOTNULL(context_);

  auto context = GetContextAs<TrafficLightUnprotectedRightTurnContext>();
  const ScenarioTrafficLightUnprotectedRightTurnConfig& scenario_config =
      context->scenario_config;

  if (!pipeline_config_.enabled()) {
    return FinishStage();
  }

  // Run creep decider.
  // 遍历所有可行驶的参考线信息，并对每条参考线执行蠕行决策处理
  for (auto& reference_line_info : *frame->mutable_reference_line_info()) {
    if (!reference_line_info.IsDrivable()) {
      // 若某条参考线不可行驶（IsDrivable() 返回 false），则记录错误并跳出循环
      AERROR << "The generated path is not drivable";
      break;
    }

    // 对可行驶的参考线调用 ProcessCreep 函数进行蠕行决策处理
    const auto ret = ProcessCreep(frame, &reference_line_info);
    if (!ret.ok()) {
      // 若处理失败（ret.ok() 为 false），则记录错误信息并跳出循环
      AERROR << "Failed to run CreepDecider ], Error message: "
             << ret.error_message();
      break;
    }
  }

  // 执行参考线上的规划任务
  StageResult result = ExecuteTaskOnReferenceLine(planning_init_point, frame);
  if (result.HasError()) {
    AERROR << "TrafficLightUnprotectedRightTurnStageCreep planning error";
  }

  // 检查当前交通灯重叠ID是否为空
  if (context->current_traffic_light_overlap_ids.empty()) {
    return FinishScenario();
  }

  // 检查当前交通信号灯是否在参考路径上存在重叠区域，如果未找到重叠区域，则调用 FinishScenario() 结束当前场景
  const auto& reference_line_info = frame->reference_line_info().front();
  const std::string traffic_light_overlap_id =
      context->current_traffic_light_overlap_ids[0];
  PathOverlap* current_traffic_light_overlap =
      reference_line_info.GetOverlapOnReferenceLine(traffic_light_overlap_id,
                                                    ReferenceLineInfo::SIGNAL);
  if (!current_traffic_light_overlap) {
    return FinishScenario();
  }

  // set right_of_way_status
  reference_line_info.SetJunctionRightOfWay(
      current_traffic_light_overlap->start_s, false);

  // creep
  // note: don't check traffic light color while creeping on right turn
  // 通过当前时间和蠕行开始时间差值获取已等待时间
  const double wait_time = Clock::NowInSeconds() - context->creep_start_time;
  const double timeout_sec = scenario_config.creep_timeout_sec();

  double creep_stop_s = GetCreepFinishS(current_traffic_light_overlap->end_s,
                                        *frame, reference_line_info);
  // 计算蠕行停止位置与自车当前位置的距离
  const double distance =
      creep_stop_s - reference_line_info.AdcSlBoundary().end_s();
  if (distance <= 0.0) {
    // 生成固定距离的蠕行速度曲线
    auto& rfl_info = frame->mutable_reference_line_info()->front();
    *(rfl_info.mutable_speed_data()) =
        SpeedProfileGenerator::GenerateFixedDistanceCreepProfile(0.0, 0);
  }

  // 调用CheckCreepDone判断是否满足结束条件（如超时或到达目标）
  if (CheckCreepDone(*frame, reference_line_info,
                     current_traffic_light_overlap->end_s, wait_time,
                     timeout_sec)) {
    return FinishStage(); // 满足则调用FinishStage()结束阶段
  }

  return result.SetStageStatus(StageStatusType::RUNNING);
}

const CreepStageConfig&
TrafficLightUnprotectedRightTurnStageCreep::GetCreepStageConfig() const {
  return GetContextAs<TrafficLightUnprotectedRightTurnContext>()
      ->scenario_config.creep_stage_config();
}

StageResult TrafficLightUnprotectedRightTurnStageCreep::FinishStage() {
  next_stage_ = "TRAFFIC_LIGHT_UNPROTECTED_RIGHT_TURN_INTERSECTION_CRUISE";
  return StageResult(StageStatusType::FINISHED);
}

}  // namespace planning
}  // namespace apollo
