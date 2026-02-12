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
 * @file stage_stop.cc
 **/

#include "modules/planning/scenarios/stop_sign_unprotected/stage_stop.h"

#include <algorithm>
#include <utility>

#include "modules/common_msgs/perception_msgs/perception_obstacle.pb.h"
#include "cyber/common/log.h"
#include "cyber/time/clock.h"
#include "modules/common/util/point_factory.h"
#include "modules/map/pnc_map/path.h"
#include "modules/planning/planning_base/common/frame.h"
#include "modules/planning/planning_base/common/planning_context.h"
#include "modules/planning/planning_base/common/util/util.h"
#include "modules/planning/scenarios/stop_sign_unprotected/context.h"

namespace apollo {
namespace planning {

using apollo::common::TrajectoryPoint;
using apollo::cyber::Clock;
using apollo::hdmap::HDMapUtil;
using apollo::hdmap::LaneInfoConstPtr;
using apollo::hdmap::OverlapInfoConstPtr;
using apollo::hdmap::PathOverlap;
using apollo::perception::PerceptionObstacle;

using StopSignLaneVehicles =
    std::unordered_map<std::string, std::vector<std::string>>;

StageResult StopSignUnprotectedStageStop::Process(
    const TrajectoryPoint& planning_init_point, Frame* frame) {
  ADEBUG << "stage: Stop";
  CHECK_NOTNULL(frame);
  CHECK_NOTNULL(context_);

  auto context = GetContextAs<StopSignUnprotectedContext>(); // 获取上下文
  const ScenarioStopSignUnprotectedConfig& scenario_config =
      context->scenario_config;

  StageResult result = ExecuteTaskOnReferenceLine(planning_init_point, frame); // 调用 ExecuteTaskOnReferenceLine 函数，在参考线上执行规划任务，
  if (result.HasError()) {
    AERROR << "StopSignUnprotectedPreStop planning error";
  }

  const auto& reference_line_info = frame->reference_line_info().front(); // 获取参考线信息
  std::string stop_sign_overlap_id = context->current_stop_sign_overlap_id; // 获取停车标志ID

  // refresh overlap along reference line
  // 检查参考线上是否存在指定的停车标志重叠区域
  PathOverlap* current_stop_sign_overlap =
      reference_line_info.GetOverlapOnReferenceLine(
          stop_sign_overlap_id, ReferenceLineInfo::STOP_SIGN); // 调用 GetOverlapOnReferenceLine 获取当前停车标志在参考线上的重叠信息
  if (!current_stop_sign_overlap) {
    return FinishScenario();
  }

  // set right_of_way_status
  const double stop_sign_start_s = current_stop_sign_overlap->start_s; // 获取当前停车标志的起始位置 stop_sign_start_s
  reference_line_info.SetJunctionRightOfWay(stop_sign_start_s, false); // 调用 SetJunctionRightOfWay 函数，将该位置对应的路口设置为无优先通行权（false）

  static constexpr double kPassStopLineBuffer = 1.0;  // unit: m
  const double adc_front_edge_s = reference_line_info.AdcSlBoundary().end_s(); // 辆前缘在参考线上的位置
  const double distance_adc_pass_stop_sign =
      adc_front_edge_s - stop_sign_start_s; // 计算车辆前缘与停止标志起始位置的距离
  // passed stop line too far
  if (distance_adc_pass_stop_sign > kPassStopLineBuffer) {
    // 如果该距离超过允许范围，则调用 FinishStage() 结束当前阶段
    return FinishStage();
  }

  // check on wait-time
  // 检查等待时间是否满足预设的停车时长要求
  auto start_time = context->stop_start_time; // 获取当前上下文中的停车开始时间 start_time
  const double wait_time = Clock::NowInSeconds() - start_time; // 计算从停车开始到当前时刻的等待时间 wait_time
  ADEBUG << "stop_start_time[" << start_time << "] wait_time[" << wait_time
         << "]";
  if (wait_time < scenario_config.stop_duration_sec()) {
    // 如果等待时间小于配置文件中设定的最小停车时长（stop_duration_sec），则返回阶段状态为“运行中”（RUNNING）
    return result.SetStageStatus(StageStatusType::RUNNING);
  }

  // check on watch_vehicles
  // 检查 watch_vehicles 容器是否为空
  auto& watch_vehicles = context->watch_vehicles;
  if (watch_vehicles.empty()) {
    return FinishStage();
  }

  // get all vehicles currently watched
  std::vector<std::string> watch_vehicle_ids;
  for (const auto& watch_vehicle : watch_vehicles) {
    std::copy(watch_vehicle.second.begin(), watch_vehicle.second.end(),
              std::back_inserter(watch_vehicle_ids)); // 将 watch_vehicle.second 中的所有元素复制到 watch_vehicle_ids 中
    // for debug
    std::string s;
    for (const std::string& vehicle : watch_vehicle.second) {
      s = s.empty() ? vehicle : s + "," + vehicle;
    }
    const std::string& associated_lane_id = watch_vehicle.first;
    ADEBUG << "watch_vehicles: lane_id[" << associated_lane_id << "] vehicle["
           << s << "]";
  }

  // remove duplicates (caused when same vehicle on mutiple lanes)
  // 使用 unique 和 erase 删除 watch_vehicle_ids 中相邻的重复元素（需先排序）
  watch_vehicle_ids.erase(
      unique(watch_vehicle_ids.begin(), watch_vehicle_ids.end()),
      watch_vehicle_ids.end());

  if (watch_vehicle_ids.empty()) {
    return FinishStage();
  }

  // pass vehicles being watched to DECIDER_RULE_BASED_STOP task
  // for visualization
  // 将被监控的车辆ID传递给DECIDER_RULE_BASED_STOP任务，用于可视化显示
  for (const auto& perception_obstacle_id : watch_vehicle_ids) {
    injector_->planning_context()
        ->mutable_planning_status()
        ->mutable_stop_sign()
        ->add_wait_for_obstacle_id(perception_obstacle_id);
  }

  // check timeout while waiting for only one vehicle
  // 等待时间超过配置的超时时间且监控车辆数≤1，则结束当前阶段
  if (wait_time > scenario_config.stop_timeout_sec() &&
      watch_vehicle_ids.size() <= 1) {
    return FinishStage();
  }

  const PathDecision& path_decision = reference_line_info.path_decision();
  RemoveWatchVehicle(path_decision, &watch_vehicles); // 从路径决策中移除不再需要监控的车辆

  return result.SetStageStatus(StageStatusType::RUNNING); // 设置并返回当前阶段为“运行中”状态
}

/**
 * @brief: remove a watch vehicle which not stopping at stop sign any more
 */
int StopSignUnprotectedStageStop::RemoveWatchVehicle(
    const PathDecision& path_decision, StopSignLaneVehicles* watch_vehicles) {
  CHECK_NOTNULL(watch_vehicles);
  auto context = GetContextAs<StopSignUnprotectedContext>();
  for (auto& vehicle : *watch_vehicles) {
    // associated_lane/stop_sign info
    std::string associated_lane_id = vehicle.first; // 从 vehicle.first 获取关联车道 ID（associated_lane_id）
    
    // 在 context->associated_lanes 中查找与该 ID 匹配的车道信息
    auto assoc_lane_it = std::find_if(
        context->associated_lanes.begin(), context->associated_lanes.end(),
        [&associated_lane_id](
            std::pair<LaneInfoConstPtr, OverlapInfoConstPtr>& assc_lane) {
          return assc_lane.first.get()->id().id() == associated_lane_id;
        });
    if (assoc_lane_it == context->associated_lanes.end()) {
      continue;
    }

    // 获取与指定车道ID关联的停车标志重叠信息
    auto stop_sign_over_lap_info =
        assoc_lane_it->second.get()->GetObjectOverlapInfo(
            hdmap::MakeMapId(associated_lane_id));
    if (stop_sign_over_lap_info == nullptr) {
      AERROR << "can't find stop_sign_over_lap_info for id: "
             << associated_lane_id;
      continue;
    }

    const double stop_line_end_s =
        stop_sign_over_lap_info->lane_overlap_info().end_s(); // 获取指定车道的停车线结束位置 stop_line_end_s

    const auto lane =
        HDMapUtil::BaseMap().GetLaneById(hdmap::MakeMapId(associated_lane_id)); // 获取指定车道ID对应的车道信息 lane
    if (lane == nullptr) {
      continue;
    }
    auto stop_sign_point = lane.get()->GetSmoothPoint(stop_line_end_s); // 获取指定车道ID对应车道的停车线结束位置对应的点 stop_sign_point

    std::vector<std::string> remove_vehicles;
    auto& vehicles = vehicle.second;

    // 遍历车辆列表 vehicles，检查每辆车是否在感知障碍物中存在，并根据距离判断是否将其标记为移除
    for (const auto& perception_obstacle_id : vehicles) {
      // watched-vehicle info
      const PerceptionObstacle* perception_obstacle =
          path_decision.FindPerceptionObstacle(perception_obstacle_id); // 通过 path_decision.FindPerceptionObstacle 查找指定 ID 的感知障碍物
      if (!perception_obstacle) {
        ADEBUG << "mark ERASE obstacle_id[" << perception_obstacle_id
               << "] not exist";
        remove_vehicles.push_back(perception_obstacle_id); // 该障碍物 ID 加入待移除列表 remove_vehicles
        continue;
      }

      PerceptionObstacle::Type obstacle_type = perception_obstacle->type(); // 获取障碍物类型，转为字符串
      std::string obstacle_type_name =
          PerceptionObstacle_Type_Name(obstacle_type);
      auto obstacle_point = common::util::PointFactory::ToPointENU(
          perception_obstacle->position()); // 将障碍物的位置信息转换为ENU坐标系下的点

      double distance =
          common::util::DistanceXY(stop_sign_point, obstacle_point); // 使用 DistanceXY 函数计算障碍物与停车标志点之间的水平距离
      ADEBUG << "obstacle_id[" << perception_obstacle_id << "] distance["
             << distance << "]";

      // TODO(all): move 10.0 to conf
      if (distance > 10.0) {
        // 若距离超过10.0，则记录需移除的障碍物ID到remove_vehicles
        ADEBUG << "mark ERASE obstacle_id[" << perception_obstacle_id << "]";
        remove_vehicles.push_back(perception_obstacle_id);
      }
    }
    for (const auto& perception_obstacle_id : remove_vehicles) {
      ADEBUG << "ERASE obstacle_id[" << perception_obstacle_id << "]";
      // 遍历remove_vehicles，从vehicles容器中删除对应ID的障碍物
      vehicles.erase(
          std::remove(vehicles.begin(), vehicles.end(), perception_obstacle_id),
          vehicles.end());
    }
  }

  return 0;
}

StageResult StopSignUnprotectedStageStop::FinishStage() {
  auto context = GetContextAs<StopSignUnprotectedContext>();
  // update PlanningContext
  // 将当前停车标志的ID标记为已完成，并清空等待障碍物的ID
  injector_->planning_context()
      ->mutable_planning_status()
      ->mutable_stop_sign()
      ->set_done_stop_sign_overlap_id(context->current_stop_sign_overlap_id);
  injector_->planning_context()
      ->mutable_planning_status()
      ->mutable_stop_sign()
      ->clear_wait_for_obstacle_id();

  // 保存当前时间为蠕行阶段的起始时间
  context->creep_start_time = Clock::NowInSeconds();

  // 将下一阶段设为STOP_SIGN_UNPROTECTED_CREEP（停车标志无保护蠕行）
  next_stage_ = "STOP_SIGN_UNPROTECTED_CREEP";

  // 返回阶段完成状态
  return StageResult(StageStatusType::FINISHED);
}

}  // namespace planning
}  // namespace apollo
