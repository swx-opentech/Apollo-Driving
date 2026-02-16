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

#include "modules/planning/scenarios/traffic_light_unprotected_right_turn/stage_stop.h"

#include "cyber/common/log.h"
#include "cyber/time/clock.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/map/pnc_map/path.h"
#include "modules/planning/planning_base/common/frame.h"
#include "modules/planning/planning_base/common/planning_context.h"
#include "modules/planning/planning_base/common/util/util.h"
#include "modules/planning/scenarios/traffic_light_unprotected_right_turn/context.h"

namespace apollo {
namespace planning {

using apollo::common::TrajectoryPoint;
using apollo::cyber::Clock;
using apollo::hdmap::HDMapUtil;
using apollo::hdmap::PathOverlap;
using apollo::perception::TrafficLight;

StageResult TrafficLightUnprotectedRightTurnStageStop::Process(
    const TrajectoryPoint& planning_init_point, Frame* frame) {
  ADEBUG << "stage: Stop";
  CHECK_NOTNULL(frame);
  CHECK_NOTNULL(context_);

  // 获取上下文对象，访问配置信息
  auto context = GetContextAs<TrafficLightUnprotectedRightTurnContext>();
  const auto& scenario_config = context->scenario_config;

  // 执行参考线上的规划任务
  StageResult result = ExecuteTaskOnReferenceLine(planning_init_point, frame);
  if (result.HasError()) {
    AERROR << "TrafficLightRightTurnUnprotectedStop planning error";
  }

  if (context->current_traffic_light_overlap_ids.empty()) {
    return FinishScenario();
  }

  const auto& reference_line_info = frame->reference_line_info().front(); // 第一条参考线
  const double adc_front_edge_s = reference_line_info.AdcSlBoundary().end_s(); // 当前车辆所在位置

  bool traffic_light_all_stop = true; // 车辆已经行驶到足够接近交通灯停止线的位置
  bool traffic_light_all_green = true; // 所有交通灯是否都为绿灯
  bool traffic_light_no_right_turn_on_red = false; // 是否禁止右转红灯
  PathOverlap* current_traffic_light_overlap = nullptr;

  /* 遍历当前交通信号灯重叠区域，检查车辆与停止线的距离以及信号灯颜色，以判断是否满足通行条件 */
  for (const auto& traffic_light_overlap_id :
       context->current_traffic_light_overlap_ids) {
    // get overlap along reference line
    // 通过traffic_light_overlap_id在参考线上查找对应的信号灯重叠区域
    current_traffic_light_overlap =
        reference_line_info.GetOverlapOnReferenceLine(
            traffic_light_overlap_id, ReferenceLineInfo::SIGNAL);
    if (!current_traffic_light_overlap) {
      continue;
    }

    // set right_of_way_status
    // 表示取消该路口的优先通行权
    reference_line_info.SetJunctionRightOfWay(
        current_traffic_light_overlap->start_s, false);

    // 判断自车与交通信号灯停止线的距离是否在有效范围内
    const double distance_adc_to_stop_line =
        current_traffic_light_overlap->start_s - adc_front_edge_s;
    auto signal_color = frame->GetSignal(traffic_light_overlap_id).color();
    ADEBUG << "traffic_light_overlap_id[" << traffic_light_overlap_id
           << "] start_s[" << current_traffic_light_overlap->start_s
           << "] distance_adc_to_stop_line[" << distance_adc_to_stop_line
           << "] color[" << signal_color << "]";

    // check distance to stop line
    if (distance_adc_to_stop_line > scenario_config.max_valid_stop_distance()) {
      // 超过配置的最大有效停车距离，则标记 traffic_light_all_stop 为 false 并跳出循环
      traffic_light_all_stop = false;
      break;
    }

    // check on traffic light color
    // 检查交通信号灯颜色
    if (signal_color != TrafficLight::GREEN) {
      // 如果信号灯颜色不是绿色（TrafficLight::GREEN），则
      traffic_light_all_green = false;
      traffic_light_no_right_turn_on_red =
          CheckTrafficLightNoRightTurnOnRed(traffic_light_overlap_id); // 调用 CheckTrafficLightNoRightTurnOnRed 函数检查是否禁止右转红灯
      break;
    }
  }

  // 当所有交通灯都处于绿灯状态，且车辆已经行驶到足够接近交通灯停止线的位置，结束当前阶段
  if (traffic_light_all_stop && traffic_light_all_green) {
    return FinishStage(true);
  }

  // 判断当前是否禁止右转红灯
  if (traffic_light_no_right_turn_on_red) {
    // 车辆已经行驶到足够接近交通灯停止线的位置，且并非所有交通灯都是绿灯
    if (traffic_light_all_stop && !traffic_light_all_green) {
      // check distance pass stop line
      // 判断自车是否已越过停止线
      const double distance_adc_pass_stop_line =
          adc_front_edge_s - current_traffic_light_overlap->end_s;
      ADEBUG << "distance_adc_pass_stop_line[" << distance_adc_pass_stop_line
             << "]";
      // 大于配置的最小通过距离，则调用 FinishStage(false) 结束当前阶段
      if (distance_adc_pass_stop_line > scenario_config.min_pass_s_distance()) {
        return FinishStage(false);
      }

      // 判断是否启用右转红灯功能
      if (scenario_config.enable_right_turn_on_red()) {
        // when right_turn_on_red is enabled
        // check on wait-time
        if (context->stop_start_time == 0.0) {
          context->stop_start_time = Clock::NowInSeconds(); // 记录停止开始时间
        } else {
          auto start_time = context->stop_start_time;
          const double wait_time = Clock::NowInSeconds() - start_time; // 计算等待时间
          ADEBUG << "stop_start_time[" << start_time << "] wait_time["
                 << wait_time << "]";
          if (wait_time >
              scenario_config.red_light_right_turn_stop_duration_sec()) {
            // 若等待时间超过配置的最长等待时间（red_light_right_turn_stop_duration_sec），则调用 FinishStage(false) 结束当前阶段
            return FinishStage(false);
          }
        }
      }
    }
  } else {
    return FinishStage(false);
  }

  return result.SetStageStatus(StageStatusType::RUNNING);
}

bool TrafficLightUnprotectedRightTurnStageStop::
    CheckTrafficLightNoRightTurnOnRed(const std::string& traffic_light_id) { //检查指定交通信号灯是否禁止右转

  // 通过traffic_light_id获取信号灯信息，若不存在则返回false
  hdmap::SignalInfoConstPtr traffic_light_ptr =
      HDMapUtil::BaseMap().GetSignalById(hdmap::MakeMapId(traffic_light_id));
  if (!traffic_light_ptr) {
    return false;
  }
  ADEBUG << "Stop when right turn: Begin to check!";
  const auto& signal = traffic_light_ptr->signal();
  for (int i = 0; i < signal.sign_info_size(); i++) {
    // 遍历信号灯的标志信息，若存在NO_RIGHT_TURN_ON_RED标志，则返回true
    if (signal.sign_info(i).type() == hdmap::SignInfo::NO_RIGHT_TURN_ON_RED) {
      ADEBUG << "Stop reason when right turn: NO_RIGHT_TURN_ON_RED";
      return true;
    }
  }
  for (auto& subsignal : signal.subsignal()) {
    // 若未找到上述标志，再遍历子信号灯，若存在右转箭头（ARROW_RIGHT），也返回true
    if (subsignal.type() == hdmap::Subsignal::ARROW_RIGHT) {
        ADEBUG << "Stop reason when right turn:: ARROW_RIGHT";
        return true;
    }
  }
  ADEBUG << "Stop when right turn: no stop";
  // 若均不满足，则返回false
  return false;
}

StageResult TrafficLightUnprotectedRightTurnStageStop::FinishStage(
    const bool protected_mode) {
  auto context = GetContextAs<TrafficLightUnprotectedRightTurnContext>();
  if (protected_mode) {
    // 根据protected_mode决定是否进入巡航阶段
    // intersection_cruise
    next_stage_ = "TRAFFIC_LIGHT_UNPROTECTED_RIGHT_TURN_INTERSECTION_CRUISE";
  } else {
    // check speed at stop_stage
    const double adc_speed = injector_->vehicle_state()->linear_velocity();
    if (adc_speed > context->scenario_config.max_adc_speed_before_creep()) {
      // 若非保护模式，检查车辆速度是否超过阈值
      // skip creep
      // 超过则直接进入巡航阶段
      next_stage_ = "TRAFFIC_LIGHT_UNPROTECTED_RIGHT_TURN_INTERSECTION_CRUISE";
    } else {
      // creep
      // update PlanningContext
      injector_->planning_context()
          ->mutable_planning_status()
          ->mutable_traffic_light()
          ->mutable_done_traffic_light_overlap_id()
          ->Clear();
      for (const auto& traffic_light_overlap_id :
           context->current_traffic_light_overlap_ids) {
        injector_->planning_context()
            ->mutable_planning_status()
            ->mutable_traffic_light()
            ->add_done_traffic_light_overlap_id(traffic_light_overlap_id);
      }

      context->creep_start_time = Clock::NowInSeconds();
      // 未超过则进入蠕行（creep）阶段，并更新交通灯状态和开始时间
      next_stage_ = "TRAFFIC_LIGHT_UNPROTECTED_RIGHT_TURN_CREEP";
    }
  }
  return StageResult(StageStatusType::FINISHED);
}

}  // namespace planning
}  // namespace apollo
