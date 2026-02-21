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

#include "modules/planning/tasks/fallback_path/fallback_path.h"
#include <memory>
#include <string>
#include <utility>
#include <vector>
#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/planning/planning_interface_base/task_base/common/path_generation.h"
#include "modules/planning/planning_interface_base/task_base/common/path_util/path_assessment_decider_util.h"
#include "modules/planning/planning_interface_base/task_base/common/path_util/path_bounds_decider_util.h"
#include "modules/planning/planning_interface_base/task_base/common/path_util/path_optimizer_util.h"

namespace apollo {
namespace planning {

using apollo::common::Status;
using apollo::common::VehicleConfigHelper;

bool FallbackPath::Init(const std::string& config_dir, const std::string& name,
                        const std::shared_ptr<DependencyInjector>& injector) {
  // 调用父类 Task 的 Init 方法初始化基础任务配置，若失败则返回 false
  if (!Task::Init(config_dir, name, injector)) {
    return false;
  }
  // Load the config this task.
  // 加载当前任务的特定配置 FallbackPathConfig 到成员变量 config_ 中，并返回加载结果
  return Task::LoadConfig<FallbackPathConfig>(&config_);
}

apollo::common::Status FallbackPath::Process(
    Frame* frame, ReferenceLineInfo* reference_line_info) {
  if (!reference_line_info->path_data().Empty() ||
      reference_line_info->IsChangeLanePath()) {
    return Status::OK();
  }
  std::vector<PathBoundary> candidate_path_boundaries;
  std::vector<PathData> candidate_path_data;

  GetStartPointSLState();
  if (!DecidePathBounds(&candidate_path_boundaries)) {
    return Status::OK();
  }
  if (!OptimizePath(candidate_path_boundaries, &candidate_path_data)) {
    return Status::OK();
  }
  if (!AssessPath(&candidate_path_data,
                  reference_line_info->mutable_path_data())) {
    AERROR << "Path assessment failed";
  }

  return Status::OK();
}

bool FallbackPath::DecidePathBounds(std::vector<PathBoundary>* boundary) {
  // 函数通过对自车道、车辆位置分析，来决定车辆行驶的路径边界
  boundary->emplace_back();
  auto& path_bound = boundary->back();
  // 1. Initialize the path boundaries to be an indefinitely large area.
  // 1. 将路径边界初始化为一个极大的区域
  if (!PathBoundsDeciderUtil::InitPathBoundary(*reference_line_info_,
                                               &path_bound, init_sl_state_)) {
    const std::string msg = "Failed to initialize path boundaries.";
    AERROR << msg;
    return false;
  }

  // 2. Decide a rough boundary based on lane info and ADC's position
  // 2. 根据当前车道信息和车辆位置，生成一个初步的路径边界
  if (!PathBoundsDeciderUtil::GetBoundaryFromSelfLane(
          *reference_line_info_, init_sl_state_, &path_bound)) {
    AERROR << "Failed to decide a rough boundary based on self lane.";
    return false;
  }
  // 进一步扩展边界以确保包含车辆的实际占用空间
  if (!PathBoundsDeciderUtil::ExtendBoundaryByADC(
          *reference_line_info_, init_sl_state_, config_.extend_buffer(),
          &path_bound)) {
    AERROR << "Failed to decide a rough boundary based on adc.";
    return false;
  }
  // 为路径边界添加标签，并记录相关调试信息
  path_bound.set_label(absl::StrCat("fallback/", "self"));
  RecordDebugInfo(path_bound, path_bound.label(), reference_line_info_);
  return true;
}

bool FallbackPath::OptimizePath(
  // 函数用于优化路径
    const std::vector<PathBoundary>& path_boundaries,  // @param path_boundaries（路径边界）
    std::vector<PathData>* candidate_path_data) { // @param candidate_path_data（存储候选路径数据）
  
  const auto& config = config_.path_optimizer_config();
  const ReferenceLine& reference_line = reference_line_info_->reference_line();
  std::array<double, 3> end_state = {0.0, 0.0, 0.0};
  
  for (const auto& path_boundary : path_boundaries) {
    size_t path_boundary_size = path_boundary.boundary().size();
    // 遍历每个路径边界，若边界点数小于等于1则报错并返回失败
    if (path_boundary_size <= 1U) {
      AERROR << "Get invalid path boundary with size: " << path_boundary_size;
      return false;
    }

    // 计算加速度边界 ddl_bounds 和抖动边界 jerk_bound
    std::vector<double> opt_l, opt_dl, opt_ddl;
    std::vector<std::pair<double, double>> ddl_bounds;
    PathOptimizerUtil::CalculateAccBound(path_boundary, reference_line,
                                         &ddl_bounds);
    const double jerk_bound = PathOptimizerUtil::EstimateJerkBoundary(
        std::fmax(init_sl_state_.first[1], 1e-12)); // 基于初始状态估计加加速度约束
    std::vector<double> ref_l(path_boundary_size, 0);
    std::vector<double> weight_ref_l(path_boundary_size,
                                     config.path_reference_l_weight());
    
    // 调用 PathOptimizerUtil::OptimizePath 进行路径优化，生成最优路径点 opt_l、速度 opt_dl 和加速度 opt_ddl
    bool res_opt = PathOptimizerUtil::OptimizePath(
        init_sl_state_, end_state, ref_l, weight_ref_l, path_boundary,
        ddl_bounds, jerk_bound, config, &opt_l, &opt_dl, &opt_ddl);
    
    if (res_opt) {
      // 若优化成功，将结果转换为 Frenet 坐标系下的路径 frenet_frame_path
      auto frenet_frame_path = PathOptimizerUtil::ToPiecewiseJerkPath(
          opt_l, opt_dl, opt_ddl, path_boundary.delta_s(),
          path_boundary.start_s());
      // 创建 PathData 对象，关联参考线和 Frenet 路径，并根据配置调整车辆前后轴中心位置
      PathData path_data;
      path_data.SetReferenceLine(&reference_line);
      path_data.SetFrenetPath(std::move(frenet_frame_path)); // 调用 SetFrenetPath 并传入移动语义的 frenet_frame_path 对象
      if (FLAGS_use_front_axe_center_in_path_planning) {
        // 若启用前轴中心规划（FLAGS_use_front_axe_center_in_path_planning），则将路径点从车头坐标系转换为后轴坐标系，并更新离散化路径
        auto discretized_path = DiscretizedPath(
            PathOptimizerUtil::ConvertPathPointRefFromFrontAxeToRearAxe(
                path_data));
        path_data.SetDiscretizedPath(discretized_path);
      }
      // 分别调用 set_path_label 和 set_blocking_obstacle_id 设置路径标签及阻塞障碍物ID
      path_data.set_path_label(path_boundary.label());
      path_data.set_blocking_obstacle_id(path_boundary.blocking_obstacle_id());
      // 处理后的路径数据加入候选列表 candidate_path_data
      candidate_path_data->push_back(std::move(path_data));
    }
  }
  // 若无有效路径则返回 false，否则返回 true
  if (candidate_path_data->empty()) {
    return false;
  }
  return true;
}

bool FallbackPath::AssessPath(std::vector<PathData>* candidate_path_data,
                              PathData* final_path) {
  // 该函数用于评估生成路径是否可行。针对OptimizePath生成路径，会判断其是否为空、路径是否远离参考线、路径是否远离道路。
  // 从candidate_path_data中取出最后一条路径数据
  PathData curr_path_data = candidate_path_data->back();
  // 调用RecordDebugInfo记录当前路径的调试信息
  RecordDebugInfo(curr_path_data, curr_path_data.path_label(),
                  reference_line_info_);
  if (curr_path_data.Empty()) {
    // 若路径为空，则返回false
    ADEBUG << "Fallback Path: path data is empty.";
    return false;
  }
  // Check if the path is greatly off the reference line.
  // 使用工具函数判断路径是否严重偏离[IsGreatlyOff]参考线，若是则返回false
  if (PathAssessmentDeciderUtil::IsGreatlyOffReferenceLine(curr_path_data)) {
    ADEBUG << "Fallback Path: ADC is greatly off reference line.";
    return false;
  }
  // Check if the path is greatly off the road.
  // 使用工具函数判断路径是否严重偏离道路，若是则返回false
  if (PathAssessmentDeciderUtil::IsGreatlyOffRoad(*reference_line_info_,
                                                  curr_path_data)) {
    ADEBUG << "Fallback Path: ADC is greatly off road.";
    return false;
  }

  if (curr_path_data.Empty()) {
    // 若修剪后路径仍为空，则返回false
    AINFO << "Lane follow path is empty after trimed";
    return false;
  }
  // 将当前路径赋值给final_path并返回true
  *final_path = curr_path_data;
  return true;
}

}  // namespace planning
}  // namespace apollo
