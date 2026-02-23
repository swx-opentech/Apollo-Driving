/******************************************************************************
 * Copyright 2023 The Apollo Authors. All Rights Reserved.
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

#include "modules/planning/tasks/lane_change_path/lane_change_path.h"

#include <algorithm>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "cyber/time/clock.h"
#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/planning/planning_base/common/planning_context.h"
#include "modules/planning/planning_interface_base/task_base/common/path_generation.h"
#include "modules/planning/planning_interface_base/task_base/common/path_util/path_assessment_decider_util.h"
#include "modules/planning/planning_interface_base/task_base/common/path_util/path_bounds_decider_util.h"
#include "modules/planning/planning_interface_base/task_base/common/path_util/path_optimizer_util.h"

namespace apollo {
namespace planning {

using apollo::common::ErrorCode;
using apollo::common::Status;
using apollo::common::VehicleConfigHelper;
using apollo::common::math::Box2d;
using apollo::common::math::Polygon2d;
using apollo::common::math::Vec2d;
using apollo::cyber::Clock;

constexpr double kIntersectionClearanceDist = 20.0;
constexpr double kJunctionClearanceDist = 15.0;

bool LaneChangePath::Init(const std::string& config_dir,
                          const std::string& name,
                          const std::shared_ptr<DependencyInjector>& injector) {
  if (!Task::Init(config_dir, name, injector)) {
    return false;
  }
  // Load the config this task.
  return Task::LoadConfig<LaneChangePathConfig>(&config_);
}

apollo::common::Status LaneChangePath::Process(
    Frame* frame, ReferenceLineInfo* reference_line_info) {
  // 是Apollo自动驾驶系统中处理车道变换路径规划的函数
  // 1. 更新车道变换状态
  UpdateLaneChangeStatus();
  const auto& status = injector_->planning_context()
                           ->mutable_planning_status()
                           ->mutable_change_lane()
                           ->status();
  // 2. 是否为变道路径或路径可复用，若不满足则直接返回成功
  if (!reference_line_info->IsChangeLanePath() ||
      reference_line_info->path_reusable()) {
    ADEBUG << "Skip this time" << reference_line_info->IsChangeLanePath()
           << "path reusable" << reference_line_info->path_reusable();
    return Status::OK();
  }
  // 3. 确保当前处于变道过程中（IN_CHANGE_LANE），否则返回错误
  if (status != ChangeLaneStatus::IN_CHANGE_LANE) {
    ADEBUG << injector_->planning_context()
                  ->mutable_planning_status()
                  ->mutable_change_lane()
                  ->DebugString();
    return Status(ErrorCode::PLANNING_ERROR,
                  "Not satisfy lane change  conditions");
  }
  std::vector<PathBoundary> candidate_path_boundaries;
  std::vector<PathData> candidate_path_data;

  GetStartPointSLState();

  // 4. 通过DecidePathBounds()确定候选路径边界
  if (!DecidePathBounds(&candidate_path_boundaries)) {
    return Status(ErrorCode::PLANNING_ERROR, "lane change path bounds failed");
  }

  // 5. 使用OptimizePath()对候选路径进行优化
  if (!OptimizePath(candidate_path_boundaries, &candidate_path_data)) {
    return Status(ErrorCode::PLANNING_ERROR,
                  "lane change path optimize failed");
  }
  // 6. 通过AssessPath()评估优化后的路径是否有效
  if (!AssessPath(&candidate_path_data,
                  reference_line_info->mutable_path_data())) {
    return Status(ErrorCode::PLANNING_ERROR, "No valid lane change path");
  }

  return Status::OK();
}

bool LaneChangePath::DecidePathBounds(std::vector<PathBoundary>* boundary) {
  boundary->emplace_back();
  auto& path_bound = boundary->back();
  double path_narrowest_width = 0;
  // 1. Initialize the path boundaries to be an indefinitely large area.
  // 1. 初始化路径边界为极大区域
  if (!PathBoundsDeciderUtil::InitPathBoundary(*reference_line_info_,
                                               &path_bound, init_sl_state_)) {
    const std::string msg = "Failed to initialize path boundaries.";
    AERROR << msg;
    return false;
  }
  // 2. Decide a rough boundary based on lane info and ADC's position
  // 2. 根据自车所在车道和位置确定粗略边界
  if (!PathBoundsDeciderUtil::GetBoundaryFromSelfLane(
          *reference_line_info_, init_sl_state_, &path_bound)) {
    AERROR << "Failed to decide a rough boundary based on self lane.";
    return false;
  }
  
  // 扩展边界以考虑车辆尺寸缓冲 -> ExtendBoundaryByADC
  if (!PathBoundsDeciderUtil::ExtendBoundaryByADC(
          *reference_line_info_, init_sl_state_, config_.extend_adc_buffer(),
          &path_bound)) {
    AERROR << "Failed to decide a rough boundary based on adc.";
    return false;
  }

  // 3. Remove the S-length of target lane out of the path-bound.
  // 3. 移除目标车道的禁入区域
  GetBoundaryFromLaneChangeForbiddenZone(&path_bound);

  path_bound.set_label("regular/lane_change");

  PathBound temp_path_bound = path_bound; //  临时路径边界
  std::string blocking_obstacle_id;
  std::vector<SLPolygon> obs_sl_polygons;
  // 调用 GetSLPolygons 获取参考线上的障碍物 SL 坐标多边形
  PathBoundsDeciderUtil::GetSLPolygons(*reference_line_info_, &obs_sl_polygons,
                                       init_sl_state_);

  // 通过 GetBoundaryFromStaticObstacles 结合障碍物信息和初始状态，精细化调整路径边界。
  if (!PathBoundsDeciderUtil::GetBoundaryFromStaticObstacles(
          *reference_line_info_, &obs_sl_polygons, init_sl_state_, &path_bound,
          &blocking_obstacle_id, &path_narrowest_width)) {
    AERROR << "Failed to decide fine tune the boundaries after "
              "taking into consideration all static obstacles.";
    return false;
  }

  // Append some extra path bound points to avoid zero-length path data.
  // 补充路径边界点：通过循环将temp_path_bound中的额外点添加到path_bound中，避免路径数据长度为零
  int counter = 0;
  // 动态扩展path_bound，直到达到预设的最大额外点数或条件不满足为止
  while (!blocking_obstacle_id.empty() &&
         path_bound.size() < temp_path_bound.size() &&
         counter < FLAGS_num_extra_tail_bound_point) {
    path_bound.push_back(temp_path_bound[path_bound.size()]);
    counter++;
  }

  path_bound.set_blocking_obstacle_id(blocking_obstacle_id);
  // 调用RecordDebugInfo函数记录路径边界信息
  RecordDebugInfo(path_bound, path_bound.label(), reference_line_info_);
  return true;
}
bool LaneChangePath::OptimizePath(
    const std::vector<PathBoundary>& path_boundaries,
    std::vector<PathData>* candidate_path_data) {
  // 实现了一个路径优化功能
  const auto& config = config_.path_optimizer_config();
  const ReferenceLine& reference_line = reference_line_info_->reference_line();
  std::array<double, 3> end_state = {0.0, 0.0, 0.0};
  for (const auto& path_boundary : path_boundaries) {
    // 遍历path_boundaries，检查每个边界是否有效（至少包含两个点）
    size_t path_boundary_size = path_boundary.boundary().size();
    if (path_boundary_size <= 1U) {
      AERROR << "Get invalid path boundary with size: " << path_boundary_size;
      return false;
    }
    std::vector<double> opt_l, opt_dl, opt_ddl;
    std::vector<std::pair<double, double>> ddl_bounds;
    // 调用CalculateAccBound计算加速度边界
    PathOptimizerUtil::CalculateAccBound(path_boundary, reference_line,
                                         &ddl_bounds);
    // 使用EstimateJerkBoundary估计抖动边界
    const double jerk_bound = PathOptimizerUtil::EstimateJerkBoundary(
        std::fmax(init_sl_state_.first[1], 1e-12));
    // 初始化参考路径ref_l和权重weight_ref_l
    std::vector<double> ref_l(path_boundary_size, 0);
    std::vector<double> weight_ref_l(path_boundary_size, 0);

    // 调用OptimizePath进行路径优化，生成最优的横向位置、速度和加速度序列
    bool res_opt = PathOptimizerUtil::OptimizePath(
        init_sl_state_, end_state, ref_l, weight_ref_l, path_boundary,
        ddl_bounds, jerk_bound, config, &opt_l, &opt_dl, &opt_ddl);
    if (res_opt) {
      // 调用 PathOptimizerUtil::ToPiecewiseJerkPath 生成Frenet坐标系下的路径
      auto frenet_frame_path = PathOptimizerUtil::ToPiecewiseJerkPath(
          opt_l, opt_dl, opt_ddl, path_boundary.delta_s(),
          path_boundary.start_s());
      
      // 创建 PathData 对象，关联参考线和Frenet路径
      PathData path_data;
      path_data.SetReferenceLine(&reference_line);
      path_data.SetFrenetPath(std::move(frenet_frame_path));
      if (FLAGS_use_front_axe_center_in_path_planning) {
        // 若启用前轴中心规划，将路径从后轴转换为前轴表示
        auto discretized_path = DiscretizedPath(
            PathOptimizerUtil::ConvertPathPointRefFromFrontAxeToRearAxe(
                path_data));
        path_data.SetDiscretizedPath(discretized_path);
      }
      // 设置路径标签和障碍物ID
      path_data.set_path_label(path_boundary.label());
      path_data.set_blocking_obstacle_id(path_boundary.blocking_obstacle_id());
      // 将处理后的路径数据移入 candidate_path_data 容器
      candidate_path_data->push_back(std::move(path_data));
    }
  }
  if (candidate_path_data->empty()) {
    return false;
  }
  return true;
}

bool LaneChangePath::AssessPath(std::vector<PathData>* candidate_path_data,
                                PathData* final_path) { // 这段代码的功能是评估候选路径并选择一条有效路径作为最终路径
  std::vector<PathData> valid_path_data;
  for (auto& curr_path_data : *candidate_path_data) {
    // 遍历候选路径，检查每条路径是否为有效常规路径（通过IsValidRegularPath判断）
    if (PathAssessmentDeciderUtil::IsValidRegularPath(*reference_line_info_,
                                                      curr_path_data)) {
      SetPathInfo(&curr_path_data);
      if (reference_line_info_->SDistanceToDestination() <
          FLAGS_path_trim_destination_threshold) {
        // 对有效路径设置路径信息，并根据条件裁剪路径尾部超出车道的部分（通过TrimTailingOutLanePoints）
        PathAssessmentDeciderUtil::TrimTailingOutLanePoints(&curr_path_data);
      }

      // 若裁剪后路径为空，则跳过该路径；否则将其加入有效路径列表
      if (curr_path_data.Empty()) {
        AINFO << "lane change path is empty after trimed";
        continue;
      }
      valid_path_data.push_back(curr_path_data);
    }
  }

  // 如果没有有效路径，返回false；否则将第一条有效路径设为最终路径，并记录调试信息后返回true
  if (valid_path_data.empty()) {
    AINFO << "All lane change path are not valid";
    return false;
  }

  *final_path = valid_path_data[0];
  RecordDebugInfo(*final_path, final_path->path_label(), reference_line_info_);
  return true;
}

void LaneChangePath::UpdateLaneChangeStatus() { // 这段代码用于更新车道变更状态
  std::string change_lane_id;
  auto* prev_status = injector_->planning_context()
                          ->mutable_planning_status()
                          ->mutable_change_lane();
  double now = Clock::NowInSeconds();
  // Init lane change status
  // 若无先前状态，则设置为CHANGE_LANE_FINISHED
  if (!prev_status->has_status()) {
    UpdateStatus(now, ChangeLaneStatus::CHANGE_LANE_FINISHED, "");
    return;
  }

  // 若当前帧无变道路径且之前在变道中，则标记变道完成
  bool has_change_lane = frame_->reference_line_info().size() > 1;
  if (!has_change_lane) {
    if (prev_status->status() == ChangeLaneStatus::IN_CHANGE_LANE) {
      UpdateStatus(now, ChangeLaneStatus::CHANGE_LANE_FINISHED,
                   prev_status->path_id());
    }
    return;
  }
  // has change lane
  // 若当前路径为变道路径，检查上一帧是否成功
  if (reference_line_info_->IsChangeLanePath()) {
    const auto* history_frame = injector_->frame_history()->Latest();
    if (!CheckLastFrameSucceed(history_frame)) {
      // 调用CheckLastFrameSucceed()验证历史帧是否成功，失败则更新状态为CHANGE_LANE_FAILED并重置标志位
      UpdateStatus(now, ChangeLaneStatus::CHANGE_LANE_FAILED, change_lane_id);
      is_exist_lane_change_start_position_ = false;
      return;
    }
    // 调用IsClearToChangeLane()判断当前是否满足变道安全条件，并获取目标车道ID
    is_clear_to_change_lane_ = IsClearToChangeLane(reference_line_info_);
    change_lane_id = reference_line_info_->Lanes().Id();
    ADEBUG << "change_lane_id" << change_lane_id;
    if (prev_status->status() == ChangeLaneStatus::CHANGE_LANE_FAILED) {
      // 若上次变道失败且超时，则更新状态为IN_CHANGE_LANE，允许重新尝试变道
      if (now - prev_status->timestamp() >
          config_.change_lane_fail_freeze_time()) {
        UpdateStatus(now, ChangeLaneStatus::IN_CHANGE_LANE, change_lane_id);
        ADEBUG << "change lane again after failed";
      }
      return;
    } else if (prev_status->status() ==
               ChangeLaneStatus::CHANGE_LANE_FINISHED) {
      // 变道完成后的冻结判断：若前一状态为CHANGE_LANE_FINISHED且当前时间超过冻结时间
      // 则更新状态为IN_CHANGE_LANE并重新开始变道
      if (now - prev_status->timestamp() >
          config_.change_lane_success_freeze_time()) {
        UpdateStatus(now, ChangeLaneStatus::IN_CHANGE_LANE, change_lane_id);
        AINFO << "change lane again after success";
      }
    } else if (prev_status->status() == ChangeLaneStatus::IN_CHANGE_LANE) {
      // 若当前状态为IN_CHANGE_LANE但路径ID与记录不符，则将状态更新为CHANGE_LANE_FINISHED，结束当前变道
      if (prev_status->path_id() != change_lane_id) {
        AINFO << "change_lane_id" << change_lane_id << "prev"
              << prev_status->path_id();
        UpdateStatus(now, ChangeLaneStatus::CHANGE_LANE_FINISHED,
                     prev_status->path_id());
      }
    }
  }
}

bool LaneChangePath::IsClearToChangeLane( // 这段代码用于判断自动驾驶车辆是否可以安全变道
    ReferenceLineInfo* reference_line_info) {
  // 计算自车在参考线上的起始和结束位置（ego_start_s, ego_end_s）以及速度（ego_v）
  double ego_start_s = reference_line_info->AdcSlBoundary().start_s();
  double ego_end_s = reference_line_info->AdcSlBoundary().end_s();
  double ego_v =
      std::abs(reference_line_info->vehicle_state().linear_velocity());

  for (const auto* obstacle :
       reference_line_info->path_decision()->obstacles().Items()) {
    // 检查路径决策中的所有障碍物，跳过虚拟或静态障碍物
    if (obstacle->IsVirtual() || obstacle->IsStatic()) {
      ADEBUG << "skip one virtual or static obstacle";
      continue;
    }

    // 计算障碍物在SL坐标系中的纵向（s）和横向（l）范围
    // 初始化s和l的起始与结束值为极值
    double start_s = std::numeric_limits<double>::max();
    double end_s = -std::numeric_limits<double>::max();
    double start_l = std::numeric_limits<double>::max();
    double end_l = -std::numeric_limits<double>::max();

    for (const auto& p : obstacle->PerceptionPolygon().points()) {
      // 遍历障碍物感知多边形的每个点，将其从XY坐标转换为SL坐标
      apollo::common::SLPoint sl_point;
      reference_line_info->reference_line().XYToSL(p, &sl_point);

      // 更新s和l的最小/最大值，以确定障碍物在SL坐标系中的覆盖范围
      start_s = std::fmin(start_s, sl_point.s());
      end_s = std::fmax(end_s, sl_point.s());

      start_l = std::fmin(start_l, sl_point.l());
      end_l = std::fmax(end_l, sl_point.l());
    }

    // 若为变道路径，进一步判断车辆在变道路径上是否超出当前车道边界
    if (reference_line_info->IsChangeLanePath()) {
      // 获取车道中点的左右宽度
      double left_width(0), right_width(0);
      reference_line_info->mutable_reference_line()->GetLaneWidth(
          (start_s + end_s) * 0.5, &left_width, &right_width);
      // 检查起始横向位置 start_l 和结束横向位置 end_l 是否超出车道范围
      //（左宽 left_width、右宽 right_width）
      if (end_l < -right_width || start_l > left_width) {
        continue;
      }
    }

    // Raw estimation on whether same direction with ADC or not based on prediction trajectory.
    // 根据预测轨迹判断障碍物与自车运动方向是否一致
    bool same_direction = true;
    if (obstacle->HasTrajectory()) {
      // obstacle_moving_direction：障碍物轨迹的初始方向角
      double obstacle_moving_direction =
          obstacle->Trajectory().trajectory_point(0).path_point().theta();
      const auto& vehicle_state = reference_line_info->vehicle_state();
      // vehicle_moving_direction：车辆当前前进方向角
      double vehicle_moving_direction = vehicle_state.heading();
      if (vehicle_state.gear() == canbus::Chassis::GEAR_REVERSE) {
        // 通过 NormalizeAngle 将两方向角差值归一化到 ([-π, π])，取绝对值得到最小夹角
        vehicle_moving_direction =
            common::math::NormalizeAngle(vehicle_moving_direction + M_PI);
      }
      double heading_difference = std::abs(common::math::NormalizeAngle(
          obstacle_moving_direction - vehicle_moving_direction));
      // 若夹角小于 (π/2)（90度），则认为同向（same_direction = true）
      same_direction = heading_difference < (M_PI / 2.0);
    }

    // TODO(All) move to confs
    static constexpr double kSafeTimeOnSameDirection = 3.0;
    static constexpr double kSafeTimeOnOppositeDirection = 5.0;
    static constexpr double kForwardMinSafeDistanceOnSameDirection = 10.0;
    static constexpr double kBackwardMinSafeDistanceOnSameDirection = 10.0;
    static constexpr double kForwardMinSafeDistanceOnOppositeDirection = 50.0;
    static constexpr double kBackwardMinSafeDistanceOnOppositeDirection = 1.0;
    static constexpr double kDistanceBuffer = 0.5;

    double kForwardSafeDistance = 0.0;
    double kBackwardSafeDistance = 0.0;
    // 根据车辆与障碍物的相对运动方向（同向或反向），分别计算前向和后向的安全距离
    if (same_direction) {
      kForwardSafeDistance =
          std::fmax(kForwardMinSafeDistanceOnSameDirection,
                    (ego_v - obstacle->speed()) * kSafeTimeOnSameDirection);
      kBackwardSafeDistance =
          std::fmax(kBackwardMinSafeDistanceOnSameDirection,
                    (obstacle->speed() - ego_v) * kSafeTimeOnSameDirection);
    } else {
      kForwardSafeDistance =
          std::fmax(kForwardMinSafeDistanceOnOppositeDirection,
                    (ego_v + obstacle->speed()) * kSafeTimeOnOppositeDirection);
      kBackwardSafeDistance = kBackwardMinSafeDistanceOnOppositeDirection;
    }

    // 通过HysteresisFilter函数检测车辆与障碍物的距离是否满足安全条件
    // 若不满足安全条件，则标记该障碍物为变道阻挡，并返回false；否则清除阻挡状态，返回true
    if (HysteresisFilter(ego_start_s - end_s, kBackwardSafeDistance,
                         kDistanceBuffer, obstacle->IsLaneChangeBlocking()) &&
        HysteresisFilter(start_s - ego_end_s, kForwardSafeDistance,
                         kDistanceBuffer, obstacle->IsLaneChangeBlocking())) {
      reference_line_info->path_decision()
          ->Find(obstacle->Id())
          ->SetLaneChangeBlocking(true);
      ADEBUG << "Lane Change is blocked by obstacle" << obstacle->Id();
      return false;
    } else {
      reference_line_info->path_decision()
          ->Find(obstacle->Id())
          ->SetLaneChangeBlocking(false);
    }
  }
  return true;
}

void LaneChangePath::GetLaneChangeStartPoint( // 这段代码的功能是计算车道变换的起始点坐标
    const ReferenceLine& reference_line, double adc_frenet_s,
    common::math::Vec2d* start_xy) {
  // 根据当前车辆位置 adc_frenet_s 和预设的准备长度，计算车道变换开始的s坐标 lane_change_start_s
  double lane_change_start_s =
      config_.lane_change_prepare_length() + adc_frenet_s;
  common::SLPoint lane_change_start_sl;
  // 将该s坐标和横向偏移量（设为0）封装成 SLPoint 对象
  lane_change_start_sl.set_s(lane_change_start_s);
  lane_change_start_sl.set_l(0.0);
  // 调用 reference_line.SLToXY() 方法将SL坐标转换为XY坐标，结果存储在 start_xy 中
  reference_line.SLToXY(lane_change_start_sl, start_xy);
}

void LaneChangePath::GetBoundaryFromLaneChangeForbiddenZone(
    PathBoundary* const path_bound) { // 这段代码的功能是根据车道变换禁入区域更新路径边界
  // Sanity checks.
  CHECK_NOTNULL(path_bound);

  // 1. 检查是否允许变道：若允许，则重置状态并返回
  if (is_clear_to_change_lane_) {
    is_exist_lane_change_start_position_ = false;
    return;
  }
  double lane_change_start_s = 0.0;
  const ReferenceLine& reference_line = reference_line_info_->reference_line();
  // If there is a pre-determined lane-change starting position, then use it;
  // otherwise, decide one.
  // 2.1 若已存在预设起始点，则转换为S坐标
  if (is_exist_lane_change_start_position_) {
    common::SLPoint point_sl;
    reference_line.XYToSL(lane_change_start_xy_, &point_sl);
    lane_change_start_s = point_sl.s();
  } else {
    // 2.2 否则基于配置和初始状态计算起始S值，并更新对应XY坐标
    // TODO(jiacheng): train ML model to learn this.
    lane_change_start_s =
        config_.lane_change_prepare_length() + init_sl_state_.first[0];

    // Update the lane_change_start_xy_ decided by lane_change_start_s
    GetLaneChangeStartPoint(reference_line, init_sl_state_.first[0],
                            &lane_change_start_xy_);
  }

  // 调整路径边界以移除目标车道的影响
  // Remove the target lane out of the path-boundary, up to the decided S.
  if (lane_change_start_s < init_sl_state_.first[0]) {
    // 若当前S值已超过lane_change_start_s，则直接返回
    // If already passed the decided S, then return.
    return;
  }
  // 获取车辆宽度的一半用于后续边界调整
  double adc_half_width =
      VehicleConfigHelper::GetConfig().vehicle_param().width() / 2.0;
  for (size_t i = 0; i < path_bound->size(); ++i) {
    double curr_s = (*path_bound)[i].s;
    if (curr_s > lane_change_start_s) {
      break;
    }
    double curr_lane_left_width = 0.0;
    double curr_lane_right_width = 0.0;
    double offset_to_map = 0.0;
    reference_line.GetOffsetToMap(curr_s, &offset_to_map);
    if (reference_line.GetLaneWidth(curr_s, &curr_lane_left_width,
                                    &curr_lane_right_width)) {
      double offset_to_lane_center = 0.0;
      reference_line.GetOffsetToMap(curr_s, &offset_to_lane_center);
      curr_lane_left_width += offset_to_lane_center;
      curr_lane_right_width -= offset_to_lane_center;
    }
    curr_lane_left_width -= offset_to_map;
    curr_lane_right_width += offset_to_map;

    (*path_bound)[i].l_lower.l = init_sl_state_.second[0] > curr_lane_left_width
                                     ? curr_lane_left_width + adc_half_width
                                     : (*path_bound)[i].l_lower.l;
    (*path_bound)[i].l_lower.l =
        std::fmin((*path_bound)[i].l_lower.l, init_sl_state_.second[0] - 0.1);
    (*path_bound)[i].l_upper.l =
        init_sl_state_.second[0] < -curr_lane_right_width
            ? -curr_lane_right_width - adc_half_width
            : (*path_bound)[i].l_upper.l;
    (*path_bound)[i].l_upper.l =
        std::fmax((*path_bound)[i].l_upper.l, init_sl_state_.second[0] + 0.1);
  }
}

void LaneChangePath::UpdateStatus(double timestamp,
                                  ChangeLaneStatus::Status status_code,
                                  const std::string& path_id) { // 这段代码的功能是更新车道变换的状态信息
  // 获取当前车道变换状态对象 lane_change_status
  auto* lane_change_status = injector_->planning_context()
                                 ->mutable_planning_status()
                                 ->mutable_change_lane();
  AINFO << "lane change update from" << lane_change_status->DebugString()
        << "to";
  lane_change_status->set_timestamp(timestamp);
  lane_change_status->set_path_id(path_id);
  lane_change_status->set_status(status_code);
  AINFO << lane_change_status->DebugString();
}

bool LaneChangePath::HysteresisFilter(const double obstacle_distance,
                                      const double safe_distance,
                                      const double distance_buffer,
                                      const bool is_obstacle_blocking) {
  // 这段代码实现了一个滞后滤波器（Hysteresis Filter），用于判断障碍物是否影响车道变换路径
  // 若当前处于阻挡状态（is_obstacle_blocking = true）
  // 当障碍物距离小于 safe_distance + distance_buffer 时返回 true                                
  if (is_obstacle_blocking) {
    return obstacle_distance < safe_distance + distance_buffer;
  } else {
    return obstacle_distance < safe_distance - distance_buffer;
  }
}

void LaneChangePath::SetPathInfo(PathData* const path_data) { // 这段代码的功能是为路径点设置车道内/外的信息
  std::vector<PathPointDecision> path_decision;
  // 调用InitPathPointDecision初始化路径点决策数组
  PathAssessmentDeciderUtil::InitPathPointDecision(
      *path_data, PathData::PathPointType::IN_LANE, &path_decision);
  // Go through every path_point, and add in-lane/out-of-lane info.
  const auto& discrete_path = path_data->discretized_path();
  SLBoundary ego_sl_boundary;
  for (size_t i = 0; i < discrete_path.size(); ++i) {
    // 对每个路径点计算其相对于参考线的SL边界
    if (!GetSLBoundary(*path_data, i, reference_line_info_, &ego_sl_boundary)) {
      ADEBUG << "Unable to get SL-boundary of ego-vehicle.";
      continue;
    }
    double lane_left_width = 0.0;
    double lane_right_width = 0.0;
    double middle_s =
        (ego_sl_boundary.start_s() + ego_sl_boundary.end_s()) / 2.0;
    if (reference_line_info_->reference_line().GetLaneWidth(
            middle_s, &lane_left_width, &lane_right_width)) {
      // Rough sl boundary estimate using single point lane width
      double back_to_inlane_extra_buffer = 0.2;
      // For lane-change path, only transitioning part is labeled as
      // out-of-lane.
      // 通过ego_sl_boundary的起始和结束s坐标计算中间位置s
      if (ego_sl_boundary.start_l() > lane_left_width ||
          ego_sl_boundary.end_l() < -lane_right_width) {
        // This means that ADC hasn't started lane-change yet.
        // 若车辆完全在当前车道内，则标记为IN_LANE
        std::get<1>((path_decision)[i]) = PathData::PathPointType::IN_LANE;
      } else if (ego_sl_boundary.start_l() >
                     -lane_right_width + back_to_inlane_extra_buffer &&
                 ego_sl_boundary.end_l() <
                     lane_left_width - back_to_inlane_extra_buffer) {
        // This means that ADC has safely completed lane-change with margin.
        // 若车辆已完成变道且有安全余量，也标记为IN_LANE；
        std::get<1>((path_decision)[i]) = PathData::PathPointType::IN_LANE;
      } else {
        // ADC is right across two lanes.
        // 否则标记为OUT_ON_FORWARD_LANE（跨车道）
        std::get<1>((path_decision)[i]) =
            PathData::PathPointType::OUT_ON_FORWARD_LANE;
      }
    } else {
      AERROR << "reference line not ready when setting path point guide";
      return;
    }
  }
  path_data->SetPathPointDecisionGuide(std::move(path_decision));
}

bool LaneChangePath::CheckLastFrameSucceed(
    const apollo::planning::Frame* const last_frame) {
  if (last_frame) {
    for (const auto& reference_line_info : last_frame->reference_line_info()) {
      // 遍历上一帧的所有参考线信息（reference_line_info）
      if (!reference_line_info.IsChangeLanePath()) {
        // 如果某条参考线不是变道路径（IsChangeLanePath()为false），则跳过
        continue;
      }
      // 检查该路径的轨迹类型（trajectory_type），若为SPEED_FALLBACK（速度回退），说明变道失败，返回false
      const auto history_trajectory_type =
          reference_line_info.trajectory_type();
      if (history_trajectory_type == ADCTrajectory::SPEED_FALLBACK) {
        return false;
      }
    }
  }
  return true;
}

}  // namespace planning
}  // namespace apollo
