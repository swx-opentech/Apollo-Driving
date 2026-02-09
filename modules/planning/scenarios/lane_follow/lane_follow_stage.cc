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

#include "modules/planning/scenarios/lane_follow/lane_follow_stage.h"

#include <utility>

#include "cyber/common/log.h"
#include "cyber/time/clock.h"
#include "modules/common/math/math_utils.h"
#include "modules/common/util/point_factory.h"
#include "modules/common/util/string_util.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/map/hdmap/hdmap.h"
#include "modules/map/hdmap/hdmap_common.h"
#include "modules/planning/planning_base/common/ego_info.h"
#include "modules/planning/planning_base/common/frame.h"
#include "modules/planning/planning_base/common/speed_profile_generator.h"
#include "modules/planning/planning_base/gflags/planning_gflags.h"
#include "modules/planning/planning_base/math/constraint_checker/constraint_checker.h"
#include "modules/planning/planning_interface_base/task_base/task.h"

namespace apollo {
namespace planning {

using apollo::common::ErrorCode;
using apollo::common::SLPoint;
using apollo::common::Status;
using apollo::common::TrajectoryPoint;
using apollo::common::util::PointFactory;
using apollo::cyber::Clock;

namespace {
constexpr double kStraightForwardLineCost = 10.0;
}  // namespace

void LaneFollowStage::RecordObstacleDebugInfo(ReferenceLineInfo* reference_line_info) {
    if (!FLAGS_enable_record_debug) {
        ADEBUG << "Skip record debug info";
        return;
    }
    auto ptr_debug = reference_line_info->mutable_debug();

    const auto path_decision = reference_line_info->path_decision();
    for (const auto obstacle : path_decision->obstacles().Items()) {
        auto obstacle_debug = ptr_debug->mutable_planning_data()->add_obstacle();
        obstacle_debug->set_id(obstacle->Id());
        obstacle_debug->mutable_sl_boundary()->CopyFrom(obstacle->PerceptionSLBoundary());
        const auto& decider_tags = obstacle->decider_tags();
        const auto& decisions = obstacle->decisions();
        if (decider_tags.size() != decisions.size()) {
            AERROR << "decider_tags size: " << decider_tags.size()
                   << " different from decisions size:" << decisions.size();
        }
        for (size_t i = 0; i < decider_tags.size(); ++i) {
            auto decision_tag = obstacle_debug->add_decision_tag();
            decision_tag->set_decider_tag(decider_tags[i]);
            decision_tag->mutable_decision()->CopyFrom(decisions[i]);
        }
    }
}

StageResult LaneFollowStage::Process(const TrajectoryPoint& planning_start_point, Frame* frame) {
    // 检查frame对象中的参考线信息是否为空。如果为空，则返回一个表示阶段已完成的状态结果。
    if (frame->reference_line_info().empty()) {
        return StageResult(StageStatusType::FINISHED);
    }

    bool has_drivable_reference_line = false; // 标记是否存在可行驶的参考线

    ADEBUG << "Number of reference lines:\t" << frame->mutable_reference_line_info()->size();

    unsigned int count = 0;
    StageResult result;

    // 遍历 `frame` 对象中所有可变的参考线信息
    for (auto& reference_line_info : *frame->mutable_reference_line_info()) {
        // TODO(SHU): need refactor

        // count 达到参考线信息的数量，则跳出循环
        if (count++ == frame->mutable_reference_line_info()->size()) {
            break;
        }
        ADEBUG << "No: [" << count << "] Reference Line.";
        ADEBUG << "IsChangeLanePath: " << reference_line_info.IsChangeLanePath();

        // 判断是否存在可行驶的参考线，如果存在，则将当前参考线设置为不可行驶状态
        if (has_drivable_reference_line) {
            reference_line_info.SetDrivable(false);
            break;
        }

        // 根据起始点和参考线生成一条可行的行驶轨迹
        result = PlanOnReferenceLine(planning_start_point, frame, &reference_line_info);

        // 查 result 对象是否没有错误
        if (!result.HasError()) {
            // 调用 IsChangeLanePath() 方法检查是否处于变道状态
            if (!reference_line_info.IsChangeLanePath()) {
                // 如果不是变道状态，则将当前参考线设置为可行驶状态
                ADEBUG << "reference line is NOT lane change ref.";
                has_drivable_reference_line = true; // 表示存在可行驶的参考线
                continue;
            }

            // 检查这条参考线路的代价（Cost）是否小于不进行车道变更的代价（kStraightForwardLineCost）
            if (reference_line_info.Cost() < kStraightForwardLineCost) {
                // If the path and speed optimization succeed on target lane while
                // under smart lane-change or IsClearToChangeLane under older version
                // 如果 小于，则认为路径和速度优化成功，设置该参考线为可行驶状态
                has_drivable_reference_line = true;
                reference_line_info.SetDrivable(true);
            } else {
                // 否则，将当前参考线设置为不可行驶状态
                reference_line_info.SetDrivable(false);
                ADEBUG << "\tlane change failed";
            }
        } else {
            reference_line_info.SetDrivable(false);
        }
    }

    return has_drivable_reference_line ? result.SetStageStatus(StageStatusType::RUNNING) // 将状态设为运行中
                                       : result.SetStageStatus(StageStatusType::ERROR);  // 状态设为错误
}

StageResult LaneFollowStage::PlanOnReferenceLine(
        const TrajectoryPoint& planning_start_point, // 规划起点的轨迹点
        Frame* frame, // 帧数据的指针
        ReferenceLineInfo* reference_line_info) { // 指向参考线信息的指针
            
    // 判断是否为变道路径
    if (!reference_line_info->IsChangeLanePath()) {
        reference_line_info->AddCost(kStraightForwardLineCost); // 不是，添加直行成本
    }
    ADEBUG << "planning start point:" << planning_start_point.DebugString();
    ADEBUG << "Current reference_line_info is IsChangeLanePath: " << reference_line_info->IsChangeLanePath();

    StageResult ret;
    for (auto task : task_list_) {
        const double start_timestamp = Clock::NowInSeconds();
        const auto start_planning_perf_timestamp // 获取系统时钟的时间戳（秒级）。
                = std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();

        ret.SetTaskStatus(task->Execute(frame, reference_line_info)); // 调用task->Execute()执行任务，并将结果状态设置到ret对象中

        const double end_timestamp = Clock::NowInSeconds();
        const double time_diff_ms = (end_timestamp - start_timestamp) * 1000; // 计算时间差，反映任务本身执行效率
        ADEBUG << "after task[" << task->Name() << "]:" << reference_line_info->PathSpeedDebugString();
        ADEBUG << task->Name() << " time spend: " << time_diff_ms << " ms.";
        RecordDebugInfo(reference_line_info, task->Name(), time_diff_ms); // 记录Task任务的执行时间

        const auto end_planning_perf_timestamp
                = std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
        const auto plnning_perf_ms = (end_planning_perf_timestamp - start_planning_perf_timestamp) * 1000; // 记录整个规划阶段的总耗时
        // AINFO << "Planning Perf: task name [" << task->Name() << "], "
        //       << plnning_perf_ms << " ms.";

        // 检查任务执行结果 ret 是否为错误状态
        if (ret.IsTaskError()) {
            AERROR << "Failed to run tasks[" << task->Name()
                   << "], Error message: " << ret.GetTaskStatus().error_message();
            break;
        }

        // TODO(SHU): disable reference line order changes for now
        // updated reference_line_info, because it is changed in
        // lane_change_decider by PrioritizeChangeLane().
        // reference_line_info = &frame->mutable_reference_line_info()->front();
        // ADEBUG << "Current reference_line_info is IsChangeLanePath: "
        //        << reference_line_info->IsChangeLanePath();
    }

    RecordObstacleDebugInfo(reference_line_info);

    // check path and speed results for path or speed fallback
    // 设置轨迹类型 正常模式
    reference_line_info->set_trajectory_type(ADCTrajectory::NORMAL);
    if (ret.IsTaskError()) {
        // 如果是任务错误，则调用 fallback_task_ 的 Execute 方法，传入当前帧 frame 和参考线信息 reference_line_info 进行回退处理
        fallback_task_->Execute(frame, reference_line_info);
    }

    DiscretizedTrajectory trajectory;
    // 调用 CombinePathAndSpeedProfile 方法尝试将规划起点的时间、路径点以及轨迹进行整合。
    if (!reference_line_info->CombinePathAndSpeedProfile(
                planning_start_point.relative_time(), planning_start_point.path_point().s(), &trajectory)) {
        const std::string msg = "Fail to aggregate planning trajectory.";
        AERROR << msg;
        return ret.SetStageStatus(StageStatusType::ERROR, msg);
    }

    // determine if there is a destination on reference line.
    double dest_stop_s = -1.0;

    // 遍历路径决策中的所有障碍物
    for (const auto* obstacle : reference_line_info->path_decision()->obstacles().Items()) {
        // 障碍物有纵向停车决策 停车原因代码为 STOP_REASON_DESTINATION
        if (obstacle->LongitudinalDecision().has_stop()
            && obstacle->LongitudinalDecision().stop().reason_code() == STOP_REASON_DESTINATION) {
            // 调用 GetStopSL 函数计算该障碍物在参考线上的停车位置（SL坐标），并更新 dest_stop_s 为对应的 s 值。
            SLPoint dest_sl = GetStopSL(obstacle->LongitudinalDecision().stop(), reference_line_info->reference_line());
            dest_stop_s = dest_sl.s();
        }
    }

    // 遍历路径上的障碍物
    for (const auto* obstacle : reference_line_info->path_decision()->obstacles().Items()) {
        if (obstacle->IsVirtual()) { // 虚拟
            continue;
        }
        if (!obstacle->IsStatic()) { // 不是静态
            continue;
        }

        // 检查障碍物是否有停车决策（has_stop()）
        if (obstacle->LongitudinalDecision().has_stop()) {
            bool add_stop_obstacle_cost = false;
            if (dest_stop_s < 0.0) {
                // 如果目标停车距离 dest_stop_s 小于 0，则直接设置 add_stop_obstacle_cost 为 true
                add_stop_obstacle_cost = true;
            } else {
                // 通过 GetStopSL 获取障碍物的停车位置 stop_sl，若该位置小于目标停车距离且与自车后边界距离小于 20 米，则也设置 为 true
                SLPoint stop_sl
                        = GetStopSL(obstacle->LongitudinalDecision().stop(), reference_line_info->reference_line());
                if (stop_sl.s() < dest_stop_s && (dest_stop_s - reference_line_info->AdcSlBoundary().end_s()) < 20.0) {
                    add_stop_obstacle_cost = true;
                }
            }
            // 当 add_stop_obstacle_cost 为真时，向 reference_line_info 添加一个静态障碍物代价 kReferenceLineStaticObsCost，用于路径规划中的成本计算。
            if (add_stop_obstacle_cost) {
                static constexpr double kReferenceLineStaticObsCost = 1e3;
                reference_line_info->AddCost(kReferenceLineStaticObsCost);
            }
        }
    }

    // 如果启用了轨迹检查
    if (FLAGS_enable_trajectory_check) {
        // 调用ConstraintChecker::ValidTrajectory验证轨迹是否有效
        if (ConstraintChecker::ValidTrajectory(trajectory) != ConstraintChecker::Result::VALID) {
            const std::string msg = "Current planning trajectory is not valid.";
            AERROR << msg;
            return ret.SetStageStatus(StageStatusType::ERROR, msg);
        }
    }

    reference_line_info->SetTrajectory(trajectory); // 将trajectory赋值给reference_line_info对象，表示参考线的轨迹信息
    reference_line_info->SetDrivable(true); // 将reference_line_info的可行驶状态设为true。
    ret.SetStageStatus(StageStatusType::RUNNING); // 将ret的阶段状态设置为RUNNING（运行中）。
    return ret;
}


// 将停车决策中的停车点坐标转换为SL坐标系下的坐标
SLPoint LaneFollowStage::GetStopSL(const ObjectStop& stop_decision, const ReferenceLine& reference_line) const {
    SLPoint sl_point;
    reference_line.XYToSL(stop_decision.stop_point(), &sl_point);
    return sl_point;
}

}  // namespace planning
}  // namespace apollo
