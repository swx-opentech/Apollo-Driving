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

#include "modules/planning/scenarios/pull_over/pull_over_scenario.h"

#include "cyber/common/log.h"
#include "modules/map/hdmap/hdmap_util.h"
#include "modules/planning/scenarios/pull_over/stage_approach.h"
#include "modules/planning/scenarios/pull_over/stage_retry_approach_parking.h"
#include "modules/planning/scenarios/pull_over/stage_retry_parking.h"

namespace apollo {
namespace planning {

using apollo::hdmap::HDMapUtil;

bool PullOverScenario::Init(std::shared_ptr<DependencyInjector> injector,
                            const std::string& name) {
  if (init_) {
    return true;
  }

  if (!Scenario::Init(injector, name)) {
    AERROR << "failed to init scenario" << Name();
    return false;
  }

  if (!Scenario::LoadConfig<ScenarioPullOverConfig>(
          &context_.scenario_config)) {
    AERROR << "fail to get config of scenario" << Name();
    return false;
  }

  init_ = true;
  return true;
}

bool PullOverScenario::IsTransferable(const Scenario* const other_scenario,
                                      const Frame& frame) {

  // 检查是否满足进入某种场景（如靠边停车）的条件
  if (!frame.local_view().planning_command->has_lane_follow_command()) {
    return false;
  }
  if (other_scenario == nullptr || frame.reference_line_info().empty()) {
    return false;
  }
  if (!FLAGS_enable_pull_over_at_destination) {
    return false;
  }
  const auto routing_end = frame.local_view().end_lane_way_point;
  if (nullptr == routing_end) {
    return false;
  }
  common::SLPoint dest_sl;
  const auto& reference_line_info = frame.reference_line_info().front(); // 参考线信息
  const auto& reference_line = reference_line_info.reference_line(); // 参考线
  reference_line.XYToSL(routing_end->pose(), &dest_sl);
  // 检查目标点 dest_sl 是否在参考线 reference_line 的车道上
  if (!reference_line.IsOnLane(dest_sl)) {
    return false;
  }
  // 计算自动驾驶车辆（ADC）前缘到目标点的距离
  const double adc_front_edge_s = reference_line_info.AdcSlBoundary().end_s();

  const double adc_distance_to_dest = dest_sl.s() - adc_front_edge_s;
  ADEBUG << "adc_distance_to_dest[" << adc_distance_to_dest
         << "] destination_s[" << dest_sl.s() << "] adc_front_edge_s["
         << adc_front_edge_s << "]";

  // 当前只有一条参考线 + 车辆到终点的距离 adc_distance_to_dest 需在配置的最小缓冲距离
  bool pull_over_scenario =
      (frame.reference_line_info().size() == 1 &&  // NO, while changing lane
       adc_distance_to_dest >=
           context_.scenario_config.pull_over_min_distance_buffer() &&
       adc_distance_to_dest <=
           context_.scenario_config.start_pull_over_scenario_distance());
  // too close to destination + not found pull-over position
  // 若以上条件均满足，则 pull_over_scenario 为 true，触发靠边停车场景。
  if (pull_over_scenario) {
    if (adc_distance_to_dest <
        context_.scenario_config.max_distance_stop_search()) {
      pull_over_scenario = false;
    }
  }

  // check around junction
  // 获取当前参考线上的首个重叠区域信息（如路口、信号灯等）
  auto first_encountered_overlaps =
      frame.reference_line_info().front().FirstEncounteredOverlaps();
  if (pull_over_scenario) {
    static constexpr double kDistanceToAvoidJunction = 8.0;  // meter
    for (const auto& overlap : first_encountered_overlaps) {
      // 遍历这些重叠区域，检查其类型是否为路口、信号灯、停车标志或让行标志
      if (overlap.first == ReferenceLineInfo::PNC_JUNCTION ||
          overlap.first == ReferenceLineInfo::SIGNAL ||
          overlap.first == ReferenceLineInfo::STOP_SIGN ||
          overlap.first == ReferenceLineInfo::YIELD_SIGN) {
        // 计算目标点到重叠区域的距离以及已通过的距离
        const double distance_to = overlap.second.start_s - dest_sl.s();
        const double distance_passed = dest_sl.s() - overlap.second.end_s;
        // 若任一距离小于阈值 kDistanceToAvoidJunction（8米），则取消靠边停车场景
        if ((distance_to > 0.0 && distance_to < kDistanceToAvoidJunction) ||
            (distance_passed > 0.0 &&
             distance_passed < kDistanceToAvoidJunction)) {
          pull_over_scenario = false;
          break;
        }
      }
    }
  }

  // check rightmost driving lane along pull-over path
  // 检查自动驾驶车辆在靠边停车场景
  if (pull_over_scenario) {
    double check_s = adc_front_edge_s;
    static constexpr double kDistanceUnit = 5.0;
    while (check_s < dest_sl.s()) {
      // 从车辆前端位置开始，每隔5米检查一次车道信息
      check_s += kDistanceUnit;

      std::vector<hdmap::LaneInfoConstPtr> lanes;
      // 调用了 reference_line 对象的 GetLaneFromS 方法，功能是根据给定的路径长度 check_s 获取对应的车道信息
      reference_line.GetLaneFromS(check_s, &lanes);
      if (lanes.empty()) {
        ADEBUG << "check_s[" << check_s << "] can't find a lane";
        continue;
      }
      const hdmap::LaneInfoConstPtr lane = lanes[0];
      const std::string lane_id = lane->lane().id().id();
      ADEBUG << "check_s[" << check_s << "] lane[" << lane_id << "]";

      // check neighbor lanes type: NONE/CITY_DRIVING/BIKING/SIDEWALK/PARKING
      bool rightmost_driving_lane = true;
      
      // 检查当前车道右侧前方相邻车道是否适合停车
      for (const auto& neighbor_lane_id :
           lane->lane().right_neighbor_forward_lane_id()) {
        const auto hdmap_ptr = HDMapUtil::BaseMapPtr();
        CHECK_NOTNULL(hdmap_ptr);
        // 获取指定ID的邻近车道信息
        const auto neighbor_lane = hdmap_ptr->GetLaneById(neighbor_lane_id);
        if (neighbor_lane == nullptr) {
          AWARN << "Failed to find neighbor lane[" << neighbor_lane_id.id()
                << "]";
          continue;
        }
        // 检查当前车道右侧相邻车道的类型是否为城市驾驶车道（CITY_DRIVING）
        const auto& lane_type = neighbor_lane->lane().type();
        if (lane_type == hdmap::Lane::CITY_DRIVING) {
          AWARN << "lane[" << lane_id << "]'s right neighbor forward lane["
                << neighbor_lane_id.id() << "] type["
                << Lane_LaneType_Name(lane_type) << "] can't pull over";
          rightmost_driving_lane = false;
          break;
        }
      }
      // 不适合向右停车
      if (!rightmost_driving_lane) {
        pull_over_scenario = false;
        break;
      }
    }
  }
  return pull_over_scenario;
}

bool PullOverScenario::Exit(Frame* frame) {
  injector_->planning_context()->mutable_planning_status()->clear_pull_over();
  return true;
}

bool PullOverScenario::Enter(Frame* frame) {
  auto* pull_over = injector_->planning_context()
                        ->mutable_planning_status()
                        ->mutable_pull_over();
  pull_over->set_pull_over_type(PullOverStatus::PULL_OVER);
  pull_over->set_plan_pull_over_path(true);
  return true;
}

}  // namespace planning
}  // namespace apollo
