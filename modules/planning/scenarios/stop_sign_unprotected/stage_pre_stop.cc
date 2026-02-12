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
 * @file stage_pre_stop.cc
 **/

#include "modules/planning/scenarios/stop_sign_unprotected/stage_pre_stop.h"

#include <algorithm>
#include <utility>

#include "modules/common_msgs/perception_msgs/perception_obstacle.pb.h"

#include "cyber/common/log.h"
#include "cyber/time/clock.h"
#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/util/point_factory.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
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

StageResult StopSignUnprotectedStagePreStop::Process(
    const TrajectoryPoint& planning_init_point, Frame* frame) {
  ADEBUG << "stage: PreStop";
  CHECK_NOTNULL(frame);
  CHECK_NOTNULL(context_);

  StageResult result = ExecuteTaskOnReferenceLine(planning_init_point, frame); // 自动驾驶规划模块中的参考线任务
  if (result.HasError()) {
    AERROR << "StopSignUnprotectedStagePreStop planning error";
  }

  const auto& reference_line_info = frame->reference_line_info().front(); // 获取参考线信息
  auto context = GetContextAs<StopSignUnprotectedContext>(); // 获取场景上下文
  std::string stop_sign_overlap_id = context->current_stop_sign_overlap_id; // 获取当前停止标志的ID

  // get overlap along reference line
  // 检查当前路径上是否存在指定的停车标志重叠
  PathOverlap* current_stop_sign_overlap =
      reference_line_info.GetOverlapOnReferenceLine(
          stop_sign_overlap_id, ReferenceLineInfo::STOP_SIGN);
  if (!current_stop_sign_overlap) {
    return FinishScenario();
  }

  // 计算自动驾驶车辆（ADC）前缘与停车标志之间的距离
  static constexpr double kPassStopLineBuffer = 0.3;  // unit: m
  const double adc_front_edge_s = reference_line_info.AdcSlBoundary().end_s();
  const double distance_adc_pass_stop_sign =
      adc_front_edge_s - current_stop_sign_overlap->start_s;
  if (distance_adc_pass_stop_sign <= kPassStopLineBuffer) {
    // not passed stop line, check valid stop
    // 距离小于等于缓冲距离 未越过停止线
    if (CheckADCStop(adc_front_edge_s, current_stop_sign_overlap->start_s)) {
      // CheckADCStop 函数检查车辆是否在指定位置（current_stop_sign_overlap->start_s）合法停车
      return FinishStage();
    }
  } else {
    // passed stop line
    return FinishStage();
  }

  // PRE-STOP
  const PathDecision& path_decision = reference_line_info.path_decision(); // 取当前参考线的路径决策对象
  auto& watch_vehicles = context->watch_vehicles; // 访问观测车辆集合

  std::vector<std::string> watch_vehicle_ids;

  // 遍历 watch_vehicles 容器，将每个车道关联的车辆ID收集到 watch_vehicle_ids
  for (const auto& vehicle : watch_vehicles) {
    // 将当前车道的所有车辆ID追加到 watch_vehicle_ids
    std::copy(vehicle.second.begin(), vehicle.second.end(),
              std::back_inserter(watch_vehicle_ids));

    // for debug string
    std::string associated_lane_id = vehicle.first;
    std::string s;
    for (const std::string& vehicle_id : vehicle.second) {
      // 构造一个逗号分隔的字符串 s，表示当前车道关联的所有车辆ID
      s = s.empty() ? vehicle_id : s + "," + vehicle_id;
    }
    ADEBUG << "watch_vehicles: lane_id[" << associated_lane_id << "] vehicle["
           << s << "]";
  }

  // pass vehicles being watched to DECIDER_RULE_BASED_STOP task
  // for visualization
  // 遍历watch_vehicle_ids容器中的每个元素
  for (const auto& perception_obstacle_id : watch_vehicle_ids) {
    injector_->planning_context()
        ->mutable_planning_status()
        ->mutable_stop_sign()
        ->add_wait_for_obstacle_id(perception_obstacle_id);
  }

  // 在车辆接近停车标志时，将相关障碍物加入监控列表
  for (const auto* obstacle : path_decision.obstacles().Items()) {
    // add to watch_vehicles if adc is still proceeding to stop sign
    AddWatchVehicle(*obstacle, &watch_vehicles);
  }

  return result.SetStageStatus(StageStatusType::RUNNING);
}

/**
 * @brief: add a watch vehicle which arrives at stop sign ahead of adc
 */
int StopSignUnprotectedStagePreStop::AddWatchVehicle(
    const Obstacle& obstacle, StopSignLaneVehicles* watch_vehicles) {
  CHECK_NOTNULL(watch_vehicles);

  const PerceptionObstacle& perception_obstacle = obstacle.Perception(); // 获取感知障碍物的引用
  const std::string& perception_obstacle_id =
      std::to_string(perception_obstacle.id()); // 获取障碍物ID
  PerceptionObstacle::Type obstacle_type = perception_obstacle.type(); // 获取障碍物枚举值
  std::string obstacle_type_name = PerceptionObstacle_Type_Name(obstacle_type); // 枚举值转换为对应的名称字符串

  // check type
  if (obstacle_type != PerceptionObstacle::UNKNOWN &&
      obstacle_type != PerceptionObstacle::UNKNOWN_MOVABLE &&
      obstacle_type != PerceptionObstacle::BICYCLE &&
      obstacle_type != PerceptionObstacle::VEHICLE) {
    ADEBUG << "obstacle_id[" << perception_obstacle_id << "] type["
           << obstacle_type_name << "]. skip";
    return 0;
  }

  const auto point =
      common::util::PointFactory::ToPointENU(perception_obstacle.position()); // 工厂类方法，将位置信息转换为ENU坐标系下的点
  double obstacle_s = 0.0;
  double obstacle_l = 0.0;
  LaneInfoConstPtr obstacle_lane;

  // 在地图中查找与给定位置和朝向最匹配的车道
  if (HDMapUtil::BaseMap().GetNearestLaneWithHeading(
          point, 5.0, perception_obstacle.theta(), M_PI / 3.0, &obstacle_lane,
          &obstacle_s, &obstacle_l) != 0) {
    ADEBUG << "obstacle_id[" << perception_obstacle_id << "] type["
           << obstacle_type_name
           << "]: Failed to find nearest lane from map for position: "
           << point.DebugString() << "; heading[" << perception_obstacle.theta()
           << "]";
    return -1;
  }

  // 检查感知到的障碍物是否在车道上
  if (!obstacle_lane->IsOnLane(common::util::PointFactory::ToVec2d(
          perception_obstacle.position()))) {
    ADEBUG << "obstacle_id[" << perception_obstacle_id << "] type["
           << obstacle_type_name << "]: is off road. " << point.DebugString()
           << "; heading[" << perception_obstacle.theta() << "]";
    return -1;
  }

  // check obstacle is on an associate lane guarded by stop sign
  std::string obstable_lane_id = obstacle_lane.get()->id().id();
  auto context = GetContextAs<StopSignUnprotectedContext>();
  // 查找与障碍物车道ID匹配的关联车道
  auto assoc_lane_it = std::find_if(
      context->associated_lanes.begin(), context->associated_lanes.end(),
      [&obstable_lane_id](
          std::pair<LaneInfoConstPtr, OverlapInfoConstPtr>& assc_lane) {
        return assc_lane.first.get()->id().id() == obstable_lane_id;
      });

  // 检查障碍物是否与当前停车标志关联的车道相关联
  if (assoc_lane_it == context->associated_lanes.end()) {
    ADEBUG << "obstacle_id[" << perception_obstacle_id << "] type["
           << obstacle_type_name << "] lane_id[" << obstable_lane_id
           << "] not associated with current stop_sign. skip";
    return -1;
  }

  // check a valid stop for stop line of the stop_sign
  // 检查车辆是否在停止线前有效停车
  auto over_lap_info = assoc_lane_it->second.get()->GetObjectOverlapInfo(
      obstacle_lane.get()->id()); // 根据障碍物车道ID获取重叠信息
  if (over_lap_info == nullptr) {
    AERROR << "can't find over_lap_info for id: " << obstable_lane_id;
    return -1;
  }
  const double stop_line_s = over_lap_info->lane_overlap_info().start_s();
  const double obstacle_end_s = obstacle_s + perception_obstacle.length() / 2;
  const double distance_to_stop_line = stop_line_s - obstacle_end_s; // 计算障碍物尾部到停止线的距离
  if (distance_to_stop_line >
      GetContextAs<StopSignUnprotectedContext>()
          ->scenario_config.watch_vehicle_max_valid_stop_distance()) {
            // 若距离超过配置的最大允许值，则认为停车无效，跳过处理。
    ADEBUG << "obstacle_id[" << perception_obstacle_id << "] type["
           << obstacle_type_name << "] distance_to_stop_line["
           << distance_to_stop_line << "]; stop_line_s" << stop_line_s
           << "]; obstacle_end_s[" << obstacle_end_s
           << "] too far from stop line. skip";
    return -1;
  }

  // use a vector since motocycles/bicycles can be more than one
  // 向指定车道添加观测车辆ID
  std::vector<std::string> vehicles =
      (*watch_vehicles)[obstacle_lane->id().id()]; // 从watch_vehicles中获取当前车道已记录的车辆列表
  if (std::find(vehicles.begin(), vehicles.end(), perception_obstacle_id) ==
      vehicles.end()) {
    // 检查目标车辆ID是否已在列表中，若不存在则添加
    ADEBUG << "AddWatchVehicle: lane[" << obstacle_lane->id().id()
           << "] obstacle_id[" << perception_obstacle_id << "]";
    // 更新watch_vehicles
    (*watch_vehicles)[obstacle_lane->id().id()].push_back(
        perception_obstacle_id);
  }

  return 0;
}

/**
 * @brief: check valid stop_sign stop
 */
bool StopSignUnprotectedStagePreStop::CheckADCStop(
    const double adc_front_edge_s, const double stop_line_s) {
  const double adc_speed = injector_->vehicle_state()->linear_velocity();
  const double max_adc_stop_speed = common::VehicleConfigHelper::Instance()
                                        ->GetConfig()
                                        .vehicle_param()
                                        .max_abs_speed_when_stopped();
  if (adc_speed > max_adc_stop_speed) {
    ADEBUG << "ADC not stopped: speed[" << adc_speed << "]";
    return false;
  }

  // check stop close enough to stop line of the stop_sign
  // 判断车辆是否在停止标志前有效停车
  const double distance_stop_line_to_adc_front_edge =
      stop_line_s - adc_front_edge_s;
  ADEBUG << "distance_stop_line_to_adc_front_edge["
         << distance_stop_line_to_adc_front_edge << "]";

  if (distance_stop_line_to_adc_front_edge >
      GetContextAs<StopSignUnprotectedContext>()
          ->scenario_config.max_valid_stop_distance()) {
    ADEBUG << "not a valid stop. too far from stop line.";
    return false;
  }

  // TODO(all): check no BICYCLE in between.

  return true;
}

StageResult StopSignUnprotectedStagePreStop::FinishStage() {
  auto scenario_context = GetContextAs<StopSignUnprotectedContext>();
  scenario_context->stop_start_time = Clock::NowInSeconds();
  next_stage_ = "STOP_SIGN_UNPROTECTED_STOP";

  return StageResult(StageStatusType::FINISHED);
}

}  // namespace planning
}  // namespace apollo
