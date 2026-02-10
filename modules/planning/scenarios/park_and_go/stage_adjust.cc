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

#include "modules/planning/scenarios/park_and_go/stage_adjust.h"

#include "cyber/common/log.h"
#include "modules/planning/planning_base/common/frame.h"
#include "modules/planning/planning_base/common/util/common.h"
#include "modules/planning/scenarios/park_and_go/context.h"
#include "modules/planning/scenarios/park_and_go/util.h"
#include "modules/planning/planning_interface_base/task_base/common/path_generation.h"
#include "modules/planning/planning_interface_base/task_base/common/path_util/path_assessment_decider_util.h"
#include "modules/planning/planning_interface_base/task_base/common/path_util/path_bounds_decider_util.h"
#include "modules/planning/planning_interface_base/task_base/common/path_util/path_optimizer_util.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"

namespace apollo {
namespace planning {

using apollo::common::TrajectoryPoint;

StageResult ParkAndGoStageAdjust::Process(const TrajectoryPoint& planning_init_point, Frame* frame) {
    ADEBUG << "stage: Adjust";
    CHECK_NOTNULL(frame);

    // 施工绕行场景判断
    static bool shigong_judged = false;
    static int shigong_num = 0;
    static double primary_pos_x = 0.0;
    static double primary_pos_y = 0.0;
    static bool initial_position_set = false; // 标记初始位置是否已设置

    // 获取车辆状态
    const auto vehicle_state_provider = injector_->vehicle_state();
    common::VehicleState vehicle_state = vehicle_state_provider->vehicle_state();
    const double current_x = vehicle_state.x();
    const double current_y = vehicle_state.y();

    // 在首次进入时设置初始位置（只设置一次）
    if (!initial_position_set) {
        primary_pos_x = current_x;
        primary_pos_y = current_y;
        initial_position_set = true;
        AINFO << "Initial position set - X: " << primary_pos_x << " Y: " << primary_pos_y;
    }

    // 获取参考线信息和障碍物信息
    if (frame->reference_line_info().empty()) {
        AERROR << "No reference line info available";
        return StageResult(StageStatusType::ERROR);
    }

    const auto& reference_line_info = frame->reference_line_info().front(); // 获取参考线信息
    const auto& path_decision = reference_line_info.path_decision(); // 获取路径决策
    const int obstacle_count = path_decision.obstacles().Items().size(); // 计路径决策中障碍物的数量
    
    if (!shigong_judged && obstacle_count > 30) {
        shigong_judged = true;
        shigong_num = 11;
    }
    // 当障碍物数量足够时进行施工绕行判断
    if (!shigong_judged && obstacle_count > 12) {
        // 收集所有障碍物的x坐标
        std::vector<double> obstacle_x_coords;
        for (const auto* obstacle : path_decision.obstacles().Items()) {
            if (!obstacle)
                continue;
            obstacle_x_coords.push_back(obstacle->Perception().position().x());
        }

        // 排序x坐标
        std::sort(obstacle_x_coords.begin(), obstacle_x_coords.end());

        // 检查是否有足够障碍物
        if (obstacle_x_coords.size() >= 4) {
            // 获取x坐标第四大的障碍物
            const double fourth_largest_x = obstacle_x_coords[obstacle_x_coords.size() - 4];
            constexpr double kEpsilon = 1e-5;  // 浮点数比较容差

            // 查找对应这个x坐标的障碍物
            for (const auto* obstacle : path_decision.obstacles().Items()) { // 遍历路径决策中的障碍物列表
                if (!obstacle)
                    continue;

                const auto& perception = obstacle->Perception(); // 获取感知信息
                if (std::abs(perception.position().x() - fourth_largest_x) < kEpsilon) { // 判断其感知位置的x坐标是否接近第四大x值
                    // 判断y坐标是否小于车辆初始y坐标
                    // 若满足条件，根据障碍物y坐标与车辆初始y坐标（primary_pos_y）的关系，以及x坐标与车辆初始x坐标（primary_pos_x）的距离，设置施工编号（shigong_num）为1、2、3或4。
                    if (perception.position().y() < primary_pos_y) {
                        shigong_num = (std::abs(perception.position().x() - primary_pos_x) < 100) ? 4 : 3;
                    } else {
                        shigong_num = (std::abs(perception.position().x() - primary_pos_x) < 100) ? 2 : 1;
                    }

                    AINFO << "Construction scenario detected - obstacle at (" << perception.position().x() << ", "
                          << perception.position().y() << "), shigong_num=" << shigong_num;
                    shigong_judged = true;
                    break;
                }
            }
        }
    }

    // 如果满足施工绕行条件，提前结束Adjust阶段
    if (shigong_judged && (shigong_num == 2 || shigong_num == 4 || shigong_num == 11)) {
        // AINFO << "current_x " << current_x << " " << primary_pos_x;
        if (current_x > primary_pos_x + 4.0 && current_x < primary_pos_x + 8.5) {
            return FinishStage();
        }
    }

    // 原有逻辑保持不变
    frame->mutable_open_space_info()->set_is_on_open_space_trajectory(true); // 设置frame对象的开放空间轨迹标志
    StageResult result = ExecuteTaskOnOpenSpace(frame); // 执行开放空间任务
    if (result.HasError()) {
        AERROR << "ParkAndGoStageAdjust planning error";
        return result.SetStageStatus(StageStatusType::ERROR);
    }

    // 已准备好进入巡航状态
    const bool is_ready_to_cruise = CheckADCReadyToCruise(
            injector_->vehicle_state(), frame, GetContextAs<ParkAndGoContext>()->scenario_config);


    // 判断当前轨迹是否已结束
    bool is_end_of_trajectory = false;
    const auto& history_frame = injector_->frame_history()->Latest();
    if (history_frame) {
        const auto& trajectory_points = history_frame->current_frame_planned_trajectory().trajectory_point();
        if (!trajectory_points.empty()) {
            is_end_of_trajectory = (trajectory_points.rbegin()->relative_time() < 0.0);
        }
    }

    if (!is_ready_to_cruise && !is_end_of_trajectory) {
        return result.SetStageStatus(StageStatusType::RUNNING);
    }
    return FinishStage();
}

StageResult ParkAndGoStageAdjust::FinishStage() {
    // 完成停车起步场景中的调整阶段（ParkAndGoStageAdjust），并决定下一阶段
    const auto vehicle_status = injector_->vehicle_state();
    ADEBUG << vehicle_status->steering_percentage();
    if (std::fabs(vehicle_status->steering_percentage())
        < GetContextAs<ParkAndGoContext>()->scenario_config.max_steering_percentage_when_cruise()) {
        // 若转向角小于配置的最大巡航转向角，则直接进入巡航阶段
        next_stage_ = "PARK_AND_GO_CRUISE";
    } else {
        // 重置初始位置后进入巡航阶段
        ResetInitPostion();
        next_stage_ = "PARK_AND_GO_CRUISE";
    }
    return StageResult(StageStatusType::FINISHED);
}

void ParkAndGoStageAdjust::ResetInitPostion() {
    auto* park_and_go_status = injector_->planning_context()->mutable_planning_status()->mutable_park_and_go();
    park_and_go_status->mutable_adc_init_position()->set_x(injector_->vehicle_state()->x());
    park_and_go_status->mutable_adc_init_position()->set_y(injector_->vehicle_state()->y());
    park_and_go_status->mutable_adc_init_position()->set_z(0.0);
    park_and_go_status->set_adc_init_heading(injector_->vehicle_state()->heading());
}

}  // namespace planning
}  // namespace apollo
