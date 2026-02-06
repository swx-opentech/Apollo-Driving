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

#include "modules/planning/scenarios/emergency_pull_over/stage_approach.h"

#include <memory>
#include <string>
#include <vector>

#include "cyber/common/log.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/planning/planning_base/common/frame.h"
#include "modules/planning/planning_base/common/planning_context.h"
#include "modules/planning/planning_base/common/util/common.h"

namespace apollo {
namespace planning {

using apollo::common::TrajectoryPoint;
using apollo::common::VehicleConfigHelper;
using apollo::common::VehicleSignal;

StageResult EmergencyPullOverStageApproach::Process(
    const TrajectoryPoint& planning_init_point, Frame* frame) {
  ADEBUG << "stage: Approach";
  CHECK_NOTNULL(frame);

  // 获取场景上下文和配置信息
  auto scenario_context = GetContextAs<EmergencyPullOverContext>();
  const auto& scenario_config = scenario_context->scenario_config;

  // 获取参考线信息并限制巡航速度为目标减速速度
  auto& reference_line_info = frame->mutable_reference_line_info()->front();
  reference_line_info.LimitCruiseSpeed(
      scenario_context->target_slow_down_speed);
  
  // set vehicle signal
  // 设置车辆右转信号灯
  reference_line_info.SetTurnSignal(VehicleSignal::TURN_RIGHT);

  double stop_line_s = 0.0;

  // add a stop fence
  const auto& pull_over_status =
      injector_->planning_context()->planning_status().pull_over();
  
  // 从规划上下文中获取紧急靠边停车的位置信息
  if (pull_over_status.has_position() && pull_over_status.position().has_x() &&
      pull_over_status.position().has_y()) {

    //从笛卡尔坐标系(XY)转换为参考线的纵向横向坐标系(SL)，存储在pull_over_sl变量中
    const auto& reference_line = reference_line_info.reference_line();
    common::SLPoint pull_over_sl;
    reference_line.XYToSL(pull_over_status.position(), &pull_over_sl);
    
    // 计算停车线位置
    const double stop_distance = scenario_config.stop_distance();
    stop_line_s =
        pull_over_sl.s() + stop_distance +
        VehicleConfigHelper::GetConfig().vehicle_param().front_edge_to_center();
    
    // 定义虚拟障碍物ID为EMERGENCY_PULL_OVER，创建空的等待障碍物ID列表
    const std::string virtual_obstacle_id = "EMERGENCY_PULL_OVER";
    const std::vector<std::string> wait_for_obstacle_ids;
    
    // 将决策应用到当前帧的参考线信息中，实现紧急靠边停车场景的规划决策
    planning::util::BuildStopDecision(
        virtual_obstacle_id, stop_line_s, stop_distance,
        StopReasonCode::STOP_REASON_PULL_OVER, wait_for_obstacle_ids,
        "EMERGENCY_PULL_OVER-scenario", frame,
        &(frame->mutable_reference_line_info()->front()));

    ADEBUG << "Build a stop fence for emergency_pull_over: id["
           << virtual_obstacle_id << "] s[" << stop_line_s << "]";
  }

  // 在参考线上的规划初始点执行任务，并将执行结果存储在result变量中。
  StageResult result = ExecuteTaskOnReferenceLine(planning_init_point, frame);
  if (result.HasError()) {
    AERROR << "EmergencyPullOverStageApproach planning error";
  }

  // 检查车辆是否在停止线前正确停车
  if (stop_line_s > 0.0) {
    const double adc_front_edge_s = reference_line_info.AdcSlBoundary().end_s(); // 获取自车前缘在参考线上的位置
    double distance = stop_line_s - adc_front_edge_s; // 计算到停止线的距离
    const double adc_speed = injector_->vehicle_state()->linear_velocity(); // 获取当前车速
    const double max_adc_stop_speed = common::VehicleConfigHelper::Instance() // 获取车辆静止时的最大允许速度阈值
                                          ->GetConfig()
                                          .vehicle_param()
                                          .max_abs_speed_when_stopped();
    ADEBUG << "adc_speed[" << adc_speed << "] distance[" << distance << "]";
    static constexpr double kStopSpeedTolerance = 0.4; // 停车速度容差值为0.4
    static constexpr double kStopDistanceTolerance = 3.0; // 停车距离容差值为3.0
    
    // 当车速小于等于最大停车速度加上容差值，并且距离误差在允许范围内时，执行停车完成操作。
    if (adc_speed <= max_adc_stop_speed + kStopSpeedTolerance &&
        std::fabs(distance) <= kStopDistanceTolerance) {
      return FinishStage();
    }
  }

  return result.SetStageStatus(StageStatusType::RUNNING);
}

StageResult EmergencyPullOverStageApproach::FinishStage() {
  next_stage_ = "EMERGENCY_PULL_OVER_STANDBY";
  return StageResult(StageStatusType::FINISHED);
}

}  // namespace planning
}  // namespace apollo
