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
 * @file stage_creep.cc
 **/
#include "modules/planning/scenarios/stop_sign_unprotected/stage_creep.h"

#include <string>

#include "modules/common_msgs/perception_msgs/perception_obstacle.pb.h"
#include "cyber/common/log.h"
#include "cyber/time/clock.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/map/pnc_map/path.h"
#include "modules/planning/planning_base/common/frame.h"
#include "modules/planning/planning_base/common/planning_context.h"
#include "modules/planning/planning_base/common/speed_profile_generator.h"
#include "modules/planning/planning_base/common/util/util.h"
#include "modules/planning/scenarios/stop_sign_unprotected/context.h"

namespace apollo {
namespace planning {

using apollo::common::TrajectoryPoint;
using apollo::cyber::Clock;
using apollo::hdmap::PathOverlap;

bool StopSignUnprotectedStageCreep::Init(
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

StageResult StopSignUnprotectedStageCreep::Process(
    const TrajectoryPoint& planning_init_point, Frame* frame) {
  ADEBUG << "stage: Creep";
  CHECK_NOTNULL(frame);
  CHECK_NOTNULL(context_);

  // 获取当前上下文对象 StopSignUnprotectedContext 的指针，并访问其配置信息。
  auto context = GetContextAs<StopSignUnprotectedContext>();
  const ScenarioStopSignUnprotectedConfig& scenario_config =
      context->scenario_config;

  if (!pipeline_config_.enabled()) {
    return FinishStage();
  }

  // Run creep decider.
  // 遍历所有可行驶的参考线信息，并对每条参考线执行ProcessCreep函数进行蠕行决策处理
  for (auto& reference_line_info : *frame->mutable_reference_line_info()) {
    if (!reference_line_info.IsDrivable()) {
      // 若某条参考线不可行驶，则记录错误并跳出循环
      AERROR << "The generated path is not drivable";
      break;
    }

    // 对可行驶的参考线调用ProcessCreep函数，若处理失败则记录错误并跳出循环
    const auto ret = ProcessCreep(frame, &reference_line_info);
    if (!ret.ok()) {
      AERROR << "Failed to run CreepDecider ], Error message: "
             << ret.error_message();
      break;
    }
  }

  // 执行参考线上的规划任务并处理结果
  StageResult result = ExecuteTaskOnReferenceLine(planning_init_point, frame);
  if (result.HasError()) {
    AERROR << "StopSignUnprotectedStageCreep planning error";
  }

  const auto& reference_line_info = frame->reference_line_info().front();

  std::string stop_sign_overlap_id = context->current_stop_sign_overlap_id;

  // get overlap along reference line
  // 当前路径上是否存在指定ID的停车标志重叠区域
  PathOverlap* current_stop_sign_overlap =
      reference_line_info.GetOverlapOnReferenceLine(
          stop_sign_overlap_id, ReferenceLineInfo::STOP_SIGN); // 调用 GetOverlapOnReferenceLine 函数，根据stop_sign_overlap_id查找停车标志重叠区域
  if (!current_stop_sign_overlap) {
    return FinishScenario();
  }

  // set right_of_way_status 路权
  // 设置停车让行标志的路权状态并计算等待时间
  const double stop_sign_start_s = current_stop_sign_overlap->start_s;
  reference_line_info.SetJunctionRightOfWay(stop_sign_start_s, false);

  const double stop_sign_end_s = current_stop_sign_overlap->end_s;
  const double wait_time = Clock::NowInSeconds() - context->creep_start_time;
  const double timeout_sec = scenario_config.creep_timeout_sec();

  // 处理车辆在停止标志前的缓行逻辑
  double creep_stop_s =
      GetCreepFinishS(stop_sign_end_s, *frame, reference_line_info); // 通过 GetCreepFinishS 获取车辆应在停止标志前完成缓行的位置
  const double distance =
      creep_stop_s - reference_line_info.AdcSlBoundary().end_s(); // 计算当前车辆后轴与缓行停止位置的距离 distance
  if (distance <= 0.0) {
    // 若距离小于等于0（即已到达或超过缓行停止位置），则生成一个固定距离的缓行速度剖面，并更新参考线信息中的速度数据
    auto& rfl_info = frame->mutable_reference_line_info()->front();
    *(rfl_info.mutable_speed_data()) =
        SpeedProfileGenerator::GenerateFixedDistanceCreepProfile(0.0, 0);
  }

  // 检查是否满足结束当前阶段的条件
  if (CheckCreepDone(*frame, reference_line_info, stop_sign_end_s, wait_time,
                     timeout_sec)) {
    // 如果 CheckCreepDone 返回 true（表示条件满足），则调用 FinishStage() 
    return FinishStage();
  }

  return result.SetStageStatus(StageStatusType::RUNNING);
}

const CreepStageConfig& StopSignUnprotectedStageCreep::GetCreepStageConfig()
    const {
  return GetContextAs<StopSignUnprotectedContext>()
      ->scenario_config.creep_stage_config();
  // 返回当前场景配置中的 creep_stage_config，通过上下文获取配置信息
}

bool StopSignUnprotectedStageCreep::GetOverlapStopInfo(
    Frame* frame, ReferenceLineInfo* reference_line_info, double* overlap_end_s,
    std::string* overlap_id) const {
  const std::string stop_sign_overlap_id = injector_->planning_context()
                                               ->planning_status()
                                               .stop_sign()
                                               .current_stop_sign_overlap_id();


  // 如果ID不为空，则在参考线上查找对应的停车标志重叠对象
  if (!stop_sign_overlap_id.empty()) {
    // get overlap along reference line
    PathOverlap* current_stop_sign_overlap =
        reference_line_info->GetOverlapOnReferenceLine(
            stop_sign_overlap_id, ReferenceLineInfo::STOP_SIGN);
    if (current_stop_sign_overlap) {
      // 若找到该对象，将其结束位置（end_s）和ID返回，并返回true；否则返回false
      *overlap_end_s = current_stop_sign_overlap->end_s;
      *overlap_id = current_stop_sign_overlap->object_id;
      return true;
    }
  }
  return false;
}

StageResult StopSignUnprotectedStageCreep::FinishStage() {
  next_stage_ = "STOP_SIGN_UNPROTECTED_INTERSECTION_CRUISE";
  return StageResult(StageStatusType::FINISHED);
}

}  // namespace planning
}  // namespace apollo
