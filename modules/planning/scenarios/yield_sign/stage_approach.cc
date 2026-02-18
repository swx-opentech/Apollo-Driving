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

#include "modules/planning/scenarios/yield_sign/stage_approach.h"

#include "cyber/common/log.h"
#include "cyber/time/clock.h"
#include "modules/map/pnc_map/path.h"
#include "modules/planning/planning_base/common/frame.h"
#include "modules/planning/planning_base/common/planning_context.h"
#include "modules/planning/planning_base/common/util/util.h"

namespace apollo {
namespace planning {

using apollo::common::TrajectoryPoint;
using apollo::cyber::Clock;
using apollo::hdmap::PathOverlap;

StageResult YieldSignStageApproach::Process(
    const TrajectoryPoint& planning_init_point, Frame* frame) {
  ADEBUG << "stage: Approach";
  CHECK_NOTNULL(frame);

  auto scenario_context = GetContextAs<YieldSignContext>(); // 获取场景上下文：通过 GetContextAs<YieldSignContext>() 获取当前场景的上下文对象，并将其赋值给 scenario_context
  scenario_config_.CopyFrom(scenario_context->scenario_config); // 将 scenario_context 中的场景配置 (scenario_config) 复制到当前对象的 scenario_config_ 成员变量中

  StageResult result = ExecuteTaskOnReferenceLine(planning_init_point, frame);
  if (result.HasError()) {
    AERROR << "YieldSignStageApproach planning error";
  }

  const auto& reference_line_info = frame->reference_line_info().front();

  if (scenario_context->current_yield_sign_overlap_ids.empty()) {
    return FinishScenario();
  }

  // 一个让行（Yield Sign）场景的处理逻辑
  for (const auto& yield_sign_overlap_id :
       scenario_context->current_yield_sign_overlap_ids) {
    // 遍历当前路径上的所有让行标志重叠区域
    // get overlap along reference line
    PathOverlap* current_yield_sign_overlap =
        reference_line_info.GetOverlapOnReferenceLine(
            yield_sign_overlap_id, ReferenceLineInfo::YIELD_SIGN);
    // 获取当前路径上的所有让行标志重叠区域
    if (!current_yield_sign_overlap) {
      continue;
    }

    // set right_of_way_status
    // 置其优先权状态
    reference_line_info.SetJunctionRightOfWay(
        current_yield_sign_overlap->start_s, false);

    // 判断自车是否已通过停止线，若通过则结束当前阶段
    static constexpr double kPassStopLineBuffer = 0.3;  // unit: m
    const double adc_front_edge_s = reference_line_info.AdcSlBoundary().end_s();
    const double distance_adc_pass_stop_sign =
        adc_front_edge_s - current_yield_sign_overlap->start_s;
    if (distance_adc_pass_stop_sign > kPassStopLineBuffer) {
      // passed stop line
      return FinishStage();
    }

    // 若未通过停止线，则检查自车与停止线的距离是否足够近
    const double distance_adc_to_stop_line =
        current_yield_sign_overlap->start_s - adc_front_edge_s;
    ADEBUG << "yield_sign_overlap_id[" << yield_sign_overlap_id << "] start_s["
           << current_yield_sign_overlap->start_s
           << "] distance_adc_to_stop_line[" << distance_adc_to_stop_line
           << "]";
    bool yield_sign_done = false;
    if (distance_adc_to_stop_line <
        scenario_config_.max_valid_stop_distance()) {
      // close enough, check yield_sign clear
      yield_sign_done = true;

      // 当距离足够近时，进一步检测是否有障碍物影响通行
      const auto& path_decision = reference_line_info.path_decision();
      for (const auto* obstacle : path_decision.obstacles().Items()) {
        // 获取障碍物ID和类型名称
        const std::string& obstacle_id = obstacle->Id();
        std::string obstacle_type_name =
            PerceptionObstacle_Type_Name(obstacle->Perception().type());
        ADEBUG << "yield_sign[" << yield_sign_overlap_id << "] obstacle_id["
               << obstacle_id << "] type[" << obstacle_type_name << "]";
        if (obstacle->IsVirtual()) {
          // 若障碍物是虚拟的（IsVirtual()为真），则跳过该障碍物
          continue;
        }

        if (obstacle->reference_line_st_boundary().IsEmpty()) {
          // 若障碍物边界信息为空，则跳过该障碍物
          continue;
        }

        static constexpr double kMinSTBoundaryT = 6.0;  // sec
        if (obstacle->reference_line_st_boundary().min_t() > kMinSTBoundaryT) {
          // 若障碍物的最小时间超过阈值（6秒）则忽略
          continue;
        }
        const double kepsilon = 1e-6;
        // 计算障碍物在ST图中左下角点与右下角点的s坐标差值，存储为 obstacle_traveled_s
        double obstacle_traveled_s =
            obstacle->reference_line_st_boundary().bottom_left_point().s() -
            obstacle->reference_line_st_boundary().bottom_right_point().s();
        ADEBUG << "obstacle[" << obstacle->Id() << "] obstacle_st_min_t["
               << obstacle->reference_line_st_boundary().min_t()
               << "] obstacle_st_min_s["
               << obstacle->reference_line_st_boundary().min_s()
               << "] obstacle_traveled_s[" << obstacle_traveled_s << "]";

        // 过滤掉满足特定条件的障碍物，避免对其进行进一步处理
        // ignore the obstacle which is already on reference line and moving
        // along the direction of ADC
        // max st_min_t(sec) to ignore
        static constexpr double kIgnoreMaxSTMinT = 0.1;
        // min st_min_s(m) to ignore
        static constexpr double kIgnoreMinSTMinS = 15.0;
        if (obstacle_traveled_s < kepsilon && // 障碍物在参考线上且正在向ADC移动
            obstacle->reference_line_st_boundary().min_t() < kIgnoreMaxSTMinT && // 障碍物的最小时间（min_t）小于阈值 kIgnoreMaxSTMinT（0.1秒）
            obstacle->reference_line_st_boundary().min_s() > kIgnoreMinSTMinS) { // 障碍物的最小距离（min_s）大于阈值 kIgnoreMinSTMinS（15米）
          continue;
        }

        // 将当前障碍物的ID添加到规划状态中的让行标志等待列表中
        injector_->planning_context()
            ->mutable_planning_status()
            ->mutable_yield_sign()
            ->add_wait_for_obstacle_id(obstacle->Id());

        yield_sign_done = false;
      }
    }

    if (yield_sign_done) {
      // 如果 yield_sign_done 为真，则调用 FinishStage() 函数结束阶段
      return FinishStage();
    }
  }

  // 设置阶段状态为 RUNNING 并返回结果
  return result.SetStageStatus(StageStatusType::RUNNING);
}

StageResult YieldSignStageApproach::FinishStage() {
  // update PlanningContext
  // 清空已完成的让行标志ID，并将当前场景中的让行标志ID添加到已完成列表中
  auto* yield_sign_status = injector_->planning_context()
                                ->mutable_planning_status()
                                ->mutable_yield_sign();
  yield_sign_status->mutable_done_yield_sign_overlap_id()->Clear();
  auto scenario_context = GetContextAs<YieldSignContext>();
  for (const auto& yield_sign_overlap_id :
       scenario_context->current_yield_sign_overlap_ids) {
    yield_sign_status->add_done_yield_sign_overlap_id(yield_sign_overlap_id);
  }
  yield_sign_status->clear_wait_for_obstacle_id(); // 清除等待障碍物的记录

  scenario_context->creep_start_time = Clock::NowInSeconds();

  // 将下一阶段设置为YIELD_SIGN_CREEP，并返回阶段完成状态
  next_stage_ = "YIELD_SIGN_CREEP";
  return StageResult(StageStatusType::FINISHED);
}

}  // namespace planning
}  // namespace apollo
