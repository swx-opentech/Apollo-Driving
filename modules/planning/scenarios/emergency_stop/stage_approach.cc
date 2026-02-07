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

#include "modules/planning/scenarios/emergency_stop/stage_approach.h"

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

StageResult EmergencyStopStageApproach::Process(
    const TrajectoryPoint& planning_init_point, Frame* frame) {
  ADEBUG << "stage: Approach";
  CHECK_NOTNULL(frame);

  // 获取场景上下文
  scenario_config_.CopyFrom(
      GetContextAs<EmergencyStopContext>()->scenario_config);

  // set vehicle signal
  // 设置车辆的紧急信号灯
  frame->mutable_reference_line_info()->front().SetEmergencyLight();

  // add a stop fence
  const auto& reference_line_info = frame->reference_line_info().front(); // 获取参考路径信息
  const auto& reference_line = reference_line_info.reference_line();      // 路径几何信息
  const double adc_speed = injector_->vehicle_state()->linear_velocity(); // 获取车辆速度
  const double adc_front_edge_s = reference_line_info.AdcSlBoundary().end_s(); // 计算车辆前缘位置
  const double stop_distance = scenario_config_.stop_distance(); // 获取停车距离配置

  bool stop_fence_exist = false;
  double stop_line_s;
  
  // 获取规划状态中的紧急停车信息emergency_stop_status
  const auto& emergency_stop_status =
      injector_->planning_context()->planning_status().emergency_stop();
  
  // 如果存在停止栅栏点，则将其从XY坐标转换为SL坐标（reference_line.XYToSL）
  if (emergency_stop_status.has_stop_fence_point()) {
    common::SLPoint stop_fence_sl;
    reference_line.XYToSL(emergency_stop_status.stop_fence_point(),
                          &stop_fence_sl);
    
    // 若转换后的s值大于自车前缘位置adc_front_edge_s，则标记存在停止线
    if (stop_fence_sl.s() > adc_front_edge_s) {
      stop_fence_exist = true;
      stop_line_s = stop_fence_sl.s();
    }
  }

  // 需要紧急停车
  if (!stop_fence_exist) {
    const double deceleration = scenario_config_.max_stop_deceleration(); // 从配置中获取最大减速度值
    // 计算停车距离，并向上取整以确保安全。
    const double travel_distance =
        std::ceil(std::pow(adc_speed, 2) / (2 * deceleration));

    // 计算车辆停止线位置 stop_line_s，公式为...
    static constexpr double kBuffer = 2.0;
    stop_line_s = adc_front_edge_s + travel_distance + stop_distance + kBuffer;
    ADEBUG << "travel_distance[" << travel_distance << "] [" << adc_speed
           << "] adc_front_edge_s[" << adc_front_edge_s << "] stop_line_s["
           << stop_line_s << "]";
    
    // 从参考线中获取停车线位置的点 stop_fence_point
    const auto& stop_fence_point =
        reference_line.GetReferencePoint(stop_line_s);
    auto* emergency_stop_fence_point = injector_->planning_context()
                                           ->mutable_planning_status()
                                           ->mutable_emergency_stop()
                                           ->mutable_stop_fence_point();
    // 将获取到的停车点坐标（x, y）设置到紧急停车围栏点中
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

         
  // 调用ExecuteTaskOnReferenceLine函数执行规划任务，
  StageResult result = ExecuteTaskOnReferenceLine(planning_init_point, frame);
  if (result.HasError()) {
    AERROR << "EmergencyPullOverStageApproach planning error";
  }

  const double max_adc_stop_speed = common::VehicleConfigHelper::Instance()
                                        ->GetConfig()
                                        .vehicle_param()
                                        .max_abs_speed_when_stopped();

  // 若当前车速adc_speed小于等于该阈值，则调用FinishStage()结束当前阶段
  if (adc_speed <= max_adc_stop_speed) {
    return FinishStage();
  }

  // 否则返回运行中状态
  return result.SetStageStatus(StageStatusType::RUNNING);
}


// 紧急停止接近阶段结束，并指定下一个阶段为待命状态
StageResult EmergencyStopStageApproach::FinishStage() {
  next_stage_ = "EMERGENCY_STOP_STANDBY";
  return StageResult(StageStatusType::FINISHED);
}

}  // namespace planning
}  // namespace apollo
