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

#include "modules/planning/scenarios/bare_intersection_unprotected/bare_intersection_unprotected_scenario.h"

#include <memory>

#include "modules/planning/planning_base/proto/planning_config.pb.h"
#include "cyber/common/log.h"
#include "modules/planning/planning_base/common/frame.h"
#include "modules/planning/planning_base/common/planning_context.h"
#include "modules/planning/scenarios/bare_intersection_unprotected/stage_approach.h"
#include "modules/planning/scenarios/bare_intersection_unprotected/stage_intersection_cruise.h"

namespace apollo {
namespace planning {

bool BareIntersectionUnprotectedScenario::Init(
    std::shared_ptr<DependencyInjector> injector, const std::string& name) {
  if (init_) {
    return true;
  }

  if (!Scenario::Init(injector, name)) {
    AERROR << "failed to init scenario " << name;
    return false;
  }

  if (!Scenario::LoadConfig<ScenarioBareIntersectionUnprotectedConfig>(
          &context_.scenario_config)) {
    AERROR << "fail to get scenario specific config in" << name;
    return false;
  }
  init_ = true;
  return true;
}

/*
 * read scenario specific configs and set in context_ for stages to read
 */

bool BareIntersectionUnprotectedScenario::IsTransferable(
    const Scenario* const other_scenario, const Frame& frame) {
  if (!frame.local_view().planning_command->has_lane_follow_command()) {
    return false;
  }
  if (other_scenario == nullptr || frame.reference_line_info().empty()) {
    return false;
  }
  const auto& reference_line_info = frame.reference_line_info().front();


  // first_encountered_overlaps 是一个容器，存储了车辆即将遇到的所有重叠区域信息。
  // overlap.first：表示重叠区域的类型（例如信号灯、停车标志、PNC 路口等）。
  // overlap.second：表示具体的重叠区域对象（如 hdmap::PathOverlap 结构体，包含起始位置、结束位置等信息）。

  const auto& first_encountered_overlaps =
      reference_line_info.FirstEncounteredOverlaps();
  hdmap::PathOverlap* pnc_junction_overlap = nullptr;
  hdmap::PathOverlap* traffic_sign_overlap = nullptr;
  // note: first_encountered_overlaps already sorted
  if (first_encountered_overlaps.empty()) {
    return false;
  }


  // 遍历所有重叠区域，first_encountered_overlaps容器，查找特定类型的路径重叠区域：
  // 1. 找到与车辆即将进入的 PNC 交叉路口重叠的区域。
  // 2. 找到与车辆即将进入的标志重叠的区域。
  for (const auto& overlap : first_encountered_overlaps) {
    // 如果遇到信号灯、停车标志或让行标志，且traffic_sign_overlap未被赋值，则将其设为当前重叠区域并跳出循环。
    if ((overlap.first == ReferenceLineInfo::SIGNAL ||
         overlap.first == ReferenceLineInfo::STOP_SIGN ||
         overlap.first == ReferenceLineInfo::YIELD_SIGN) &&
        traffic_sign_overlap == nullptr) {
      traffic_sign_overlap = const_cast<hdmap::PathOverlap*>(&overlap.second);
      break;
    } // 如果遇到PNC路口（规划与控制交叉口），且pnc_junction_overlap未被赋值，则将其设为当前重叠区域。
    else if (overlap.first == ReferenceLineInfo::PNC_JUNCTION &&
               pnc_junction_overlap == nullptr) {
      pnc_junction_overlap = const_cast<hdmap::PathOverlap*>(&overlap.second);
    }
  }


  // 若差值大于等于预设阈值kJunctionDelta，比较两者的起始位置，保留更靠前的那个，将另一个置为nullptr。
  if (traffic_sign_overlap && pnc_junction_overlap) {
    static constexpr double kJunctionDelta = 10.0;
    double s_diff = std::fabs(traffic_sign_overlap->start_s -
                              pnc_junction_overlap->start_s);
    if (s_diff >= kJunctionDelta) {
      if (pnc_junction_overlap->start_s > traffic_sign_overlap->start_s) {
        pnc_junction_overlap = nullptr;
      } else {
        traffic_sign_overlap = nullptr;
      }
    }
  }

  // 没有路口，也没有标志
  if (traffic_sign_overlap || !pnc_junction_overlap) {
    return false;
  }

  // 若车辆在交叉路口有通行权，则函数提前返回 false，表示不需要进一步处理让行逻辑。
  if (reference_line_info.GetIntersectionRightofWayStatus(
          *pnc_junction_overlap)) {
    return false;
  }


  // 这段代码用于判断车辆是否处于“裸露路口”（bare junction）场景。
  // 若距离在合理范围内（大于0且小于配置阈值），则返回 true，表示进入裸露路口场景。
  const double adc_front_edge_s = reference_line_info.AdcSlBoundary().end_s();
  const double adc_distance_to_pnc_junction =
      pnc_junction_overlap->start_s - adc_front_edge_s;
  ADEBUG << "adc_distance_to_pnc_junction[" << adc_distance_to_pnc_junction
         << "] pnc_junction[" << pnc_junction_overlap->object_id
         << "] pnc_junction_overlap_start_s[" << pnc_junction_overlap->start_s
         << "]";

  const bool bare_junction_scenario =
      (adc_distance_to_pnc_junction > 0.0 &&
       adc_distance_to_pnc_junction <=
           context_.scenario_config
               .start_bare_intersection_scenario_distance());

  return bare_junction_scenario;
}

// 判断车辆是否进入无保护裸露交叉路口场景，并更新规划上下文信息。

bool BareIntersectionUnprotectedScenario::Enter(Frame* frame) {
  // set to first_encountered pnc_junction
  // 获取当前参考线上的首个相遇重叠区域 first_encountered_overlaps。
  const auto& first_encountered_overlaps =
      frame->reference_line_info().front().FirstEncounteredOverlaps();
  for (const auto& overlap : first_encountered_overlaps) {
    // 遍历这些重叠区域，查找类型为 PNC_JUNCTION 的对象
    if (overlap.first == ReferenceLineInfo::PNC_JUNCTION) {
      // 找到！记录该路口的 ID 和起始位置（start_s），并更新到上下文 context_
      context_.current_pnc_junction_overlap_id = overlap.second.object_id;
      ADEBUG << "Update PlanningContext with first_encountered pnc_junction["
             << overlap.second.object_id << "] start_s["
             << overlap.second.start_s << "]";
      break;
    }
  }
  return true;
}

}  // namespace planning
}  // namespace apollo
