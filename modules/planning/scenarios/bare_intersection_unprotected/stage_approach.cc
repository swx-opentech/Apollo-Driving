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

#include "modules/planning/scenarios/bare_intersection_unprotected/stage_approach.h"

#include <vector>

#include "cyber/common/log.h"
#include "modules/map/pnc_map/path.h"
#include "modules/planning/planning_base/common/frame.h"
#include "modules/planning/planning_base/common/planning_context.h"
#include "modules/planning/planning_base/common/util/common.h"

namespace apollo {
namespace planning {

using apollo::common::TrajectoryPoint;
using apollo::hdmap::PathOverlap;

StageResult BareIntersectionUnprotectedStageApproach::Process(
    const TrajectoryPoint& planning_init_point, Frame* frame) {
  ADEBUG << "stage: Approach";
  CHECK_NOTNULL(frame);

  scenario_config_.CopyFrom(
      GetContextAs<BareIntersectionUnprotectedContext>()->scenario_config);

  // 调用 ExecuteTaskOnReferenceLine 函数，传入规划初始点和帧数据，返回执行结果 result。
  StageResult result = ExecuteTaskOnReferenceLine(planning_init_point, frame);
  if (result.HasError()) {
    AERROR << "BareIntersectionUnprotectedStageApproach planning error";
  }

  const auto& reference_line_info = frame->reference_line_info().front();

  const std::string pnc_junction_overlap_id =
      GetContextAs<BareIntersectionUnprotectedContext>()
          ->current_pnc_junction_overlap_id;
  
  // 若该ID为空，则调用FinishScenario()结束当前场景。
  if (pnc_junction_overlap_id.empty()) {
    return FinishScenario();
  }

  // get overlap along reference line
  PathOverlap* current_pnc_junction =
      reference_line_info.GetOverlapOnReferenceLine(
          pnc_junction_overlap_id, ReferenceLineInfo::PNC_JUNCTION);
  if (!current_pnc_junction) {
    return FinishScenario();
  }


  // !!!!!! 判断自动驾驶车辆是否已通过PNC交叉路口的停止线：!!!!!!!
  // !!!!!! 判断自动驾驶车辆是否已通过PNC交叉路口的停止线：!!!!!!!
  static constexpr double kPassStopLineBuffer = 0.3;  // unit: m
  const double adc_front_edge_s = reference_line_info.AdcSlBoundary().end_s();
  const double distance_adc_to_pnc_junction =
      current_pnc_junction->start_s - adc_front_edge_s;
  ADEBUG << "pnc_junction_overlap_id[" << pnc_junction_overlap_id
         << "] start_s[" << current_pnc_junction->start_s
         << "] distance_adc_to_pnc_junction[" << distance_adc_to_pnc_junction
         << "]";
  
  // 超过停止线
  if (distance_adc_to_pnc_junction < -kPassStopLineBuffer) {
    // passed stop line
    return FinishStage(frame);
  }

  // set cruise_speed to slow down
  // 限制巡航速度：通过 LimitCruiseSpeed 将巡航速度设置为较慢的速度，以确保安全接近路口
  frame->mutable_reference_line_info()->front().LimitCruiseSpeed(
      scenario_config_.approach_cruise_speed());

  // set right_of_way_status
  // 设置路权状态：调用 SetJunctionRightOfWay 设置当前路口的路权状态，参数表明车辆在指定位置（start_s）不具有优先通行权
  reference_line_info.SetJunctionRightOfWay(current_pnc_junction->start_s,
                                            false);
  
  // 执行路径规划任务：调用 ExecuteTaskOnReferenceLine 执行参考线上的规划任务
  result = ExecuteTaskOnReferenceLine(planning_init_point, frame);
  if (result.HasError()) {
    AERROR << "BareIntersectionUnprotectedStageApproach planning error";
  }

  std::vector<std::string> wait_for_obstacle_ids;
  // 检查障碍物：调用 CheckClear 函数判断路径是否畅通，并获取需等待的障碍物ID列表
  bool clear = CheckClear(reference_line_info, &wait_for_obstacle_ids);

  if (scenario_config_.enable_explicit_stop()) { // 启用了显式停车（enable_explicit_stop）
    bool stop = false;
    static constexpr double kCheckClearDistance = 5.0;  // meter
    static constexpr double kStartWatchDistance = 2.0;  // meter
    
    // 距离在 [kStartWatchDistance, kCheckClearDistance] 范围内且路径不畅通时触发停车
    if (distance_adc_to_pnc_junction <= kCheckClearDistance &&
        distance_adc_to_pnc_junction >= kStartWatchDistance && !clear) {
      stop = true;
    } else if (distance_adc_to_pnc_junction < kStartWatchDistance) {
      // creeping area
      counter_ = clear ? counter_ + 1 : 0;

      // 若路径畅通则递增计数器，否则重置；计数器达到阈值后允许通行，否则继续停车。
      if (counter_ >= 5) {
        counter_ = 0;  // reset
      } else {
        stop = true;
      }
    }

    // 停车决策
    if (stop) {
      // build stop decision
      ADEBUG << "BuildStopDecision: bare pnc_junction["
             << pnc_junction_overlap_id << "] start_s["
             << current_pnc_junction->start_s << "]";
      const std::string virtual_obstacle_id =
          "PNC_JUNCTION_" + current_pnc_junction->object_id;

      // 调用停车决策函数：使用 planning::util::BuildStopDecision 构建停车决策
      planning::util::BuildStopDecision(
          virtual_obstacle_id, current_pnc_junction->start_s,
          scenario_config_.stop_distance(),
          StopReasonCode::STOP_REASON_STOP_SIGN, wait_for_obstacle_ids,
          "bare intersection", frame,
          &(frame->mutable_reference_line_info()->front()));
    }
  }

  // 将当前阶段的状态设置为“运行中”（RUNNING），并返回该操作的结果。
  return result.SetStageStatus(StageStatusType::RUNNING);
}

bool BareIntersectionUnprotectedStageApproach::CheckClear(
    const ReferenceLineInfo& reference_line_info,
    std::vector<std::string>* wait_for_obstacle_ids) {
  // TODO(all): move to conf
  static constexpr double kConf_min_boundary_t = 6.0;        // second
  static constexpr double kConf_ignore_max_st_min_t = 0.1;   // second
  static constexpr double kConf_ignore_min_st_min_s = 15.0;  // meter

  bool all_far_away = true;
  for (auto* obstacle :
       reference_line_info.path_decision().obstacles().Items()) {
    if (obstacle->IsVirtual() || obstacle->IsStatic()) {
      continue;
    }
    if (obstacle->reference_line_st_boundary().min_t() < kConf_min_boundary_t) {
      const double kepsilon = 1e-6;
      double obstacle_traveled_s =
          obstacle->reference_line_st_boundary().bottom_left_point().s() -
          obstacle->reference_line_st_boundary().bottom_right_point().s();
      ADEBUG << "obstacle[" << obstacle->Id() << "] obstacle_st_min_t["
             << obstacle->reference_line_st_boundary().min_t()
             << "] obstacle_st_min_s["
             << obstacle->reference_line_st_boundary().min_s()
             << "] obstacle_traveled_s[" << obstacle_traveled_s << "]";

      // ignore the obstacle which is already on reference line and moving
      // along the direction of ADC
      if (obstacle_traveled_s < kepsilon &&
          obstacle->reference_line_st_boundary().min_t() <
              kConf_ignore_max_st_min_t &&
          obstacle->reference_line_st_boundary().min_s() >
              kConf_ignore_min_st_min_s) {
        continue;
      }

      wait_for_obstacle_ids->push_back(obstacle->Id());
      all_far_away = false;
    }
  }
  return all_far_away;
}

// 完成当前阶段并准备进入下一阶段
// 设置下一阶段为 "BARE_INTERSECTION_UNPROTECTED_INTERSECTION_CRUISE"
// 重置参考线的巡航速度为默认值 FLAGS_default_cruise_speed
StageResult BareIntersectionUnprotectedStageApproach::FinishStage(
    Frame* frame) {
  next_stage_ = "BARE_INTERSECTION_UNPROTECTED_INTERSECTION_CRUISE";

  // reset cruise_speed
  auto& reference_line_info = frame->mutable_reference_line_info()->front();
  reference_line_info.LimitCruiseSpeed(FLAGS_default_cruise_speed);

  return StageResult(StageStatusType::FINISHED);
}

}  // namespace planning
}  // namespace apollo
