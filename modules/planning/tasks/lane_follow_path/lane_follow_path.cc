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

#include "modules/planning/tasks/lane_follow_path/lane_follow_path.h"

#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/planning/planning_base/common/util/print_debug_info.h"
#include "modules/planning/planning_interface_base/task_base/common/path_generation.h"
#include "modules/planning/planning_interface_base/task_base/common/path_util/path_assessment_decider_util.h"
#include "modules/planning/planning_interface_base/task_base/common/path_util/path_bounds_decider_util.h"
#include "modules/planning/planning_interface_base/task_base/common/path_util/path_optimizer_util.h"
namespace apollo {
namespace planning {

using apollo::common::Status;
using apollo::common::VehicleConfigHelper;

bool LaneFollowPath::Init(
        const std::string& config_dir,
        const std::string& name,
        const std::shared_ptr<DependencyInjector>& injector) {
    // 调用父类Task的Init方法进行基础初始化，若失败则返回false
    if (!Task::Init(config_dir, name, injector)) {
        return false;
    }
    // Load the config this task.
    return Task::LoadConfig<LaneFollowPathConfig>(&config_);
}

apollo::common::Status LaneFollowPath::Process(Frame* frame, ReferenceLineInfo* reference_line_info) {
    // 1. 若路径数据非空Empty或可复用path_reusable，则直接返回成功状态
    if (!reference_line_info->path_data().Empty() || reference_line_info->path_reusable()) {
        ADEBUG << "Skip this time path empty:" << reference_line_info->path_data().Empty()
               << "path reusable: " << reference_line_info->path_reusable();
        return Status::OK();
    }
    ReferenceLine* reference_line = reference_line_info->mutable_reference_line(); // 获取参考线
    const auto& obstacles = reference_line_info->path_decision()->obstacles().Items(); // 获取路径规划中的障碍物
    if (obstacles.size() > 30) {
        // 遍历所有障碍物
        // 每个障碍物，在其前后扩展区域（start_s - 5 到 end_s + 10）添加限速 13
        for (const auto& obstacle : obstacles) {
            const double speed_limit = 13;  // 5.5
            const double start_s = obstacle->PerceptionSLBoundary().start_s() - 5;
            const double end_s = obstacle->PerceptionSLBoundary().end_s() + 10;
            reference_line->AddSpeedLimit(start_s, end_s, speed_limit);
        }
    } else if (obstacles.size() > 10 && obstacles.size() < 30) {
        // 遍历所有障碍物
        // 同样处理，但限速设为 7.5
        for (const auto& obstacle : obstacles) {
            const double speed_limit = 7.5;
            const double start_s = obstacle->PerceptionSLBoundary().start_s() - 5;
            const double end_s = obstacle->PerceptionSLBoundary().end_s() + 10;
            reference_line->AddSpeedLimit(start_s, end_s, speed_limit);
        }
    } else if (obstacles.size() <= 3) {
        // 限速设为 4.5
        for (const auto& obstacle : obstacles) {
            const double speed_limit = 4.5;  // 4.5
            const double start_s = obstacle->PerceptionSLBoundary().start_s() - 5;
            const double end_s = obstacle->PerceptionSLBoundary().end_s() + 10;
            reference_line->AddSpeedLimit(start_s, end_s, speed_limit);
        }
    }
    std::vector<PathBoundary> candidate_path_boundaries;
    std::vector<PathData> candidate_path_data;

    // 划路径并评估其可行性
    GetStartPointSLState(); // 调用 GetStartPointSLState() 获取车辆起点的SL坐标状态
    if (!DecidePathBounds(&candidate_path_boundaries)) {
        // 通过 DecidePathBounds() 计算候选路径边界，若失败则返回错误
        AERROR << "Decide path bound failed";
        return Status::OK();
    }
    if (!OptimizePath(candidate_path_boundaries, &candidate_path_data)) {
        // 使用 OptimizePath() 对路径进行优化，生成候选路径数据，若失败则返回错误
        AERROR << "Optmize path failed";
        return Status::OK();
    }
    if (!AssessPath(&candidate_path_data, reference_line_info->mutable_path_data())) {1
        // 调用 AssessPath() 评估候选路径的优劣，并更新参考线信息中的路径数据
        AERROR << "Path assessment failed";
    }

    return Status::OK();
}

bool LaneFollowPath::DecidePathBounds(std::vector<PathBoundary>* boundary) {
    boundary->emplace_back();
    auto& path_bound = boundary->back();
    std::string blocking_obstacle_id = "";
    std::string lane_type = "";
    double path_narrowest_width = 0;
    // 1. Initialize the path boundaries to be an indefinitely large area.
    // 1. 将边界初始化为一个极大区域
    if (!PathBoundsDeciderUtil::InitPathBoundary(*reference_line_info_, &path_bound, init_sl_state_)) {
        const std::string msg = "Failed to initialize path boundaries.";
        AERROR << msg;
        return false;
    }
    std::string borrow_lane_type;
    bool is_include_adc = config_.is_extend_lane_bounds_to_include_adc()
            && !injector_->planning_context()->planning_status().path_decider().is_in_path_lane_borrow_scenario();

    // 2. Decide a rough boundary based on lane info and ADC's position
    // 2. 根据自车所在车道和位置设定初步边界
    if (!PathBoundsDeciderUtil::GetBoundaryFromSelfLane(*reference_line_info_, init_sl_state_, &path_bound)) {
        // 调用 GetBoundaryFromSelfLane 获取基于自车道的粗略路径边界，失败则返回错误
        AERROR << "Failed to decide a rough boundary based on self lane.";
        return false;
    }
    if (is_include_adc) {
        // 若 is_include_adc 为真，调用 ExtendBoundaryByADC 根据 ADC（自动驾驶车辆）状态扩展边界
        PathBoundsDeciderUtil::ExtendBoundaryByADC(
                *reference_line_info_, init_sl_state_, config_.extend_buffer(), &path_bound);
    }
    PrintCurves print_curve; // 创建 PrintCurves 对象
    auto indexed_obstacles = reference_line_info_->path_decision()->obstacles();
    for (const auto* obs : indexed_obstacles.Items()) {
        // 遍历所有障碍物，
        const auto& sl_bound = obs->PerceptionSLBoundary();
        // 提取其 SL 坐标边界点并添加到打印曲线中
        for (int i = 0; i < sl_bound.boundary_point_size(); i++) {
            std::string name = obs->Id() + "_obs_sl_boundary";
            print_curve.AddPoint(name, sl_bound.boundary_point(i).s(), sl_bound.boundary_point(i).l());
        }
    }
    print_curve.PrintToLog();

    path_bound.set_label(absl::StrCat("regular/", "self")); // 为路径边界设置标签 "regular/self"

    // 3. Fine-tune the boundary based on static obstacles
    // 3. 结合障碍物信息进一步优化边界
    // 将当前路径边界 path_bound 复制到临时变量 temp_path_bound
    PathBound temp_path_bound = path_bound;
    std::vector<SLPolygon> obs_sl_polygons;
    // 调用 GetSLPolygons 获取参考线上的障碍物 SL 坐标多边形
    PathBoundsDeciderUtil::GetSLPolygons(*reference_line_info_, &obs_sl_polygons, init_sl_state_);
    // 通过 GetBoundaryFromStaticObstacles 考虑所有静态障碍物，更新路径边界 path_bound，并记录阻挡障碍物 ID 和最窄宽度
    if (!PathBoundsDeciderUtil::GetBoundaryFromStaticObstacles(
                *reference_line_info_,
                &obs_sl_polygons,
                init_sl_state_,
                &path_bound,
                &blocking_obstacle_id,
                &path_narrowest_width)) {
        const std::string msg
                = "Failed to decide fine tune the boundaries after "
                  "taking into consideration all static obstacles.";
        AERROR << msg;
        return false;
    }
    // 4. Append some extra path bound points to avoid zero-length path data.
    // 向路径边界添加额外的点以避免路径数据长度为零
    int counter = 0;
    // blocking_obstacle_id 不为空/path_bound 的大小小于 temp_path_bound 的大小/循环次数未超过 FLAGS_num_extra_tail_bound_point
    while (!blocking_obstacle_id.empty() && path_bound.size() < temp_path_bound.size()
           && counter < FLAGS_num_extra_tail_bound_point) {
        // 每次循环将 temp_path_bound 中对应位置的点追加到 path_bound 中，并递增计数器 counter
        path_bound.push_back(temp_path_bound[path_bound.size()]);
        counter++;
    }

    // lane_follow_status update
    // 更新车道跟随状态
    auto* lane_follow_status = injector_->planning_context()->mutable_planning_status()->mutable_lane_follow(); // 通过 injector_ 获取可变的车道跟随状态
    if (!blocking_obstacle_id.empty()) {
        // 若存在阻塞障碍物（blocking_obstacle_id 非空）
        double current_time = ::apollo::cyber::Clock::NowInSeconds(); // 获取当前时间
        lane_follow_status->set_block_obstacle_id(blocking_obstacle_id); // 设置阻塞障碍物 ID
        if (lane_follow_status->lane_follow_block()) {
            // 若已处于阻塞状态，累加阻塞持续时间；否则初始化阻塞状态
            lane_follow_status->set_block_duration(
                    lane_follow_status->block_duration() + current_time - lane_follow_status->last_block_timestamp());
        } else {            // 否则初始化阻塞状态
            lane_follow_status->set_block_duration(0);
            lane_follow_status->set_lane_follow_block(true);
        }
        lane_follow_status->set_last_block_timestamp(current_time); // 更新上次阻塞时间戳
    } else {
        // 若无阻塞障碍物且当前为阻塞状态，则重置阻塞相关字段
        if (lane_follow_status->lane_follow_block()) {
            lane_follow_status->set_block_duration(0);
            lane_follow_status->set_lane_follow_block(false);
            lane_follow_status->set_last_block_timestamp(0);
        }
    }

    ADEBUG << "Completed generating path boundaries.";
    // 判断初始横向位置 init_sl_state_.second[0] 是否超出当前路径边界 [l_lower.l, l_upper.l]，若超出则返回 false 并记录日志
    if (init_sl_state_.second[0] > path_bound[0].l_upper.l || init_sl_state_.second[0] < path_bound[0].l_lower.l) {
        AINFO << "not in self lane maybe lane borrow or out of road. init l : " << init_sl_state_.second[0]
              << ", path_bound l: [ " << path_bound[0].l_lower.l << "," << path_bound[0].l_upper.l << " ]";
        return false;
    }
    // std::vector<std::pair<double, double>> regular_path_bound_pair;
    // for (size_t i = 0; i < path_bound.size(); ++i) {
    //   regular_path_bound_pair.emplace_back(std::get<1>(path_bound[i]),
    //                                        std::get<2>(path_bound[i]));
    // }

    // 为路径边界设置阻挡障碍物ID
    path_bound.set_blocking_obstacle_id(blocking_obstacle_id);
    RecordDebugInfo(path_bound, path_bound.label(), reference_line_info_);
    return true;
}

bool LaneFollowPath::OptimizePath(
        const std::vector<PathBoundary>& path_boundaries,
        std::vector<PathData>* candidate_path_data) {
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
        PathOptimizerUtil::CalculateAccBound(path_boundary, reference_line, &ddl_bounds); // 调用CalculateAccBound计算加速度边界
        PrintCurves print_debug;
        // 遍历路径边界点，根据弧长s获取参考线的曲率 kappa，并记录到 print_debug 中用于调试
        for (size_t i = 0; i < path_boundary_size; ++i) {
            double s = static_cast<double>(i) * path_boundary.delta_s() + path_boundary.start_s();
            double kappa = reference_line.GetNearestReferencePoint(s).kappa();
            print_debug.AddPoint("ref_kappa", s, kappa);
        }
        print_debug.PrintToLog();
        // 调用EstimateJerkBoundary函数，基于初始状态估计加加速度约束
        const double jerk_bound = PathOptimizerUtil::EstimateJerkBoundary(std::fmax(init_sl_state_.first[1], 1e-12));
        std::vector<double> ref_l(path_boundary_size, 0);
        std::vector<double> weight_ref_l(path_boundary_size, 0);

        // 过UpdatePathRefWithBound更新参考路径及其权重
        PathOptimizerUtil::UpdatePathRefWithBound(
                path_boundary, config.path_reference_l_weight(), &ref_l, &weight_ref_l);
        // 调用OptimizePath进行路径优化，生成纵向位置、速度和加速度序列
        bool res_opt = PathOptimizerUtil::OptimizePath(
                init_sl_state_,
                end_state,
                ref_l,
                weight_ref_l,
                path_boundary,
                ddl_bounds,
                jerk_bound,
                config,
                &opt_l,
                &opt_dl,
                &opt_ddl);
        if (res_opt) {
            // 通过ToPiecewiseJerkPath将优化后的横向位置、速度、加速度转换为Frenet路径
            auto frenet_frame_path = PathOptimizerUtil::ToPiecewiseJerkPath(
                    opt_l, opt_dl, opt_ddl, path_boundary.delta_s(), path_boundary.start_s());

            // 创建PathData对象，关联参考线和Frenet路径，并根据配置决定是否将前轴中心转换为后轴中心
            PathData path_data;
            path_data.SetReferenceLine(&reference_line);
            path_data.SetFrenetPath(std::move(frenet_frame_path));
            if (FLAGS_use_front_axe_center_in_path_planning) {
                auto discretized_path
                        = DiscretizedPath(PathOptimizerUtil::ConvertPathPointRefFromFrontAxeToRearAxe(path_data));
                path_data.SetDiscretizedPath(discretized_path);
            }
            // 设置路径标签和阻挡障碍物ID
            path_data.set_path_label(path_boundary.label());
            path_data.set_blocking_obstacle_id(path_boundary.blocking_obstacle_id());
            candidate_path_data->push_back(std::move(path_data)); // 将处理后的路径数据加入候选列表
            PrintCurves print_path_kappa;
            for (const auto& p : candidate_path_data->back().discretized_path()) {
                // 遍历离散化路径点，收集并输出曲率数据用于调试
                print_path_kappa.AddPoint(
                        path_boundary.label() + "_path_kappa", p.s() + init_sl_state_.first[0], p.kappa());
            }
            print_path_kappa.PrintToLog();
        }
    }
    // 若无有效路径则返回false，否则返回true
    if (candidate_path_data->empty()) {
        return false;
    }
    return true;
}

bool LaneFollowPath::AssessPath(std::vector<PathData>* candidate_path_data, PathData* final_path) {
    // 从candidate_path_data中取出最后一条路径数据
    PathData& curr_path_data = candidate_path_data->back();
    RecordDebugInfo(curr_path_data, curr_path_data.path_label(), reference_line_info_);
    if (!PathAssessmentDeciderUtil::IsValidRegularPath(*reference_line_info_, curr_path_data)) {
        // 调用工具函数检查路径是否有效，无效则返回false
        AINFO << "Lane follow path is invalid";
        return false;
    }

    // 为路径点设置决策信息，并更新路径数据
    std::vector<PathPointDecision> path_decision;
    PathAssessmentDeciderUtil::InitPathPointDecision(curr_path_data, PathData::PathPointType::IN_LANE, &path_decision);
    curr_path_data.SetPathPointDecisionGuide(std::move(path_decision));

    if (curr_path_data.Empty()) {
        // 若路径为空（被修剪后），记录日志并返回false
        AINFO << "Lane follow path is empty after trimed";
        return false;
    }
    // 将当前路径设为最终路径，记录调试信息，并更新参考线信息
    *final_path = curr_path_data;
    AINFO << final_path->path_label() << final_path->blocking_obstacle_id();
    reference_line_info_->MutableCandidatePathData()->push_back(*final_path);
    reference_line_info_->SetBlockingObstacle(curr_path_data.blocking_obstacle_id());
    return true;
}

}  // namespace planning
}  // namespace apollo
