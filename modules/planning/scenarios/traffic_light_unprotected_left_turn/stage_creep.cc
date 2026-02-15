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

#include "modules/planning/scenarios/traffic_light_unprotected_left_turn/stage_creep.h"

#include <string>

#include "modules/common_msgs/perception_msgs/traffic_light_detection.pb.h"
#include "cyber/common/log.h"
#include "cyber/time/clock.h"
#include "modules/planning/planning_base/common/frame.h"
#include "modules/planning/planning_base/common/planning_context.h"
#include "modules/planning/planning_base/common/speed_profile_generator.h"
#include "modules/planning/scenarios/traffic_light_unprotected_left_turn/context.h"

namespace apollo {
namespace planning {

using apollo::common::TrajectoryPoint;
using apollo::cyber::Clock;
using apollo::hdmap::PathOverlap;

bool TrafficLightUnprotectedLeftTurnStageCreep::Init(
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

StageResult TrafficLightUnprotectedLeftTurnStageCreep::Process(
    const TrajectoryPoint& planning_init_point, Frame* frame) {
  ADEBUG << "stage: Creep";
  CHECK_NOTNULL(frame);
  CHECK_NOTNULL(context_);

  auto context = GetContextAs<TrafficLightUnprotectedLeftTurnContext>();
  const ScenarioTrafficLightUnprotectedLeftTurnConfig& scenario_config =
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

    const auto ret = ProcessCreep(frame, &reference_line_info); // 对可行驶的参考线调用 ProcessCreep 函数进行蠕行决策处理
    if (!ret.ok()) {
      // 若处理失败（ret.ok() 为 false），则记录错误信息并跳出循环
      AERROR << "Failed to run CreepDecider ], Error message: "
             << ret.error_message();
      break;
    }
  }

  // 调用ExecuteTaskOnReferenceLine函数在参考线上执行规划任务，返回结果存储在result中
  StageResult result = ExecuteTaskOnReferenceLine(planning_init_point, frame);
  if (result.HasError()) {
    AERROR << "TrafficLightUnprotectedLeftTurnStageCreep planning error";
  }

  // 检查当前交通灯重叠ID是否为空，若为空则调用FinishScenario结束当前场景
  if (context->current_traffic_light_overlap_ids.empty()) {
    return FinishScenario();
  }

  // 检查当前交通信号灯是否在参考路径上存在重叠区域
  const auto& reference_line_info = frame->reference_line_info().front(); // 获取当前帧的第一个参考路径信息 reference_line_info
  const std::string traffic_light_overlap_id =
      context->current_traffic_light_overlap_ids[0]; // 从上下文获取当前交通信号灯的ID traffic_light_overlap_id
  PathOverlap* current_traffic_light_overlap =
      reference_line_info.GetOverlapOnReferenceLine(traffic_light_overlap_id,
                                                    ReferenceLineInfo::SIGNAL); // 在参考路径上查找该信号灯的重叠区域 current_traffic_light_overlap
  if (!current_traffic_light_overlap) {
    return FinishScenario(); // 如果未找到重叠区域，则调用 FinishScenario() 结束场景
  }

  // set right_of_way_status
  reference_line_info.SetJunctionRightOfWay(
      current_traffic_light_overlap->start_s, false); // 标记当前交通灯控制的路口为无优先通行权状态



  // creep
  // note: don't check traffic light color while creeping on right turn
  // 计算等待时间和超时时间：通过当前时间和起始时间差计算已等待时间，并获取配置的超时时间
  const double wait_time = Clock::NowInSeconds() - context->creep_start_time;
  const double timeout_sec = scenario_config.creep_timeout_sec();

  // 确定缓行停止位置：调用GetCreepFinishS计算车辆应停止的位置，并与当前车位置比较距离
  double creep_stop_s = GetCreepFinishS(current_traffic_light_overlap->end_s,
                                        *frame, reference_line_info);
  const double distance =
      creep_stop_s - reference_line_info.AdcSlBoundary().end_s();
  if (distance <= 0.0) {
    // 若车辆已到达或超过目标位置，则生成固定距离的缓行速度曲线
    auto& rfl_info = frame->mutable_reference_line_info()->front();
    *(rfl_info.mutable_speed_data()) =
        SpeedProfileGenerator::GenerateFixedDistanceCreepProfile(0.0, 0);
  }

  // 用CheckCreepDone判断是否满足结束条件（如距离或时间达标），若满足则结束当前阶段
  if (CheckCreepDone(*frame, reference_line_info,
                     current_traffic_light_overlap->end_s, wait_time,
                     timeout_sec)) {
    return FinishStage();
  }

  return result.SetStageStatus(StageStatusType::RUNNING); // 设置为“运行中”（RUNNING）
}

const CreepStageConfig&
TrafficLightUnprotectedLeftTurnStageCreep::GetCreepStageConfig() const {
  return GetContextAs<TrafficLightUnprotectedLeftTurnContext>()
      ->scenario_config.creep_stage_config();
}

StageResult TrafficLightUnprotectedLeftTurnStageCreep::FinishStage() {
  next_stage_ = "TRAFFIC_LIGHT_UNPROTECTED_LEFT_TURN_INTERSECTION_CRUISE";
  return StageResult(StageStatusType::FINISHED);
}

}  // namespace planning
}  // namespace apollo
