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

#include "modules/planning/scenarios/park_and_go/park_and_go_scenario.h"

#include "cyber/common/log.h"
#include "modules/common/util/point_factory.h"
#include "modules/map/hdmap/hdmap_util.h"
#include "modules/planning/scenarios/park_and_go/stage_adjust.h"
#include "modules/planning/scenarios/park_and_go/stage_check.h"
#include "modules/planning/scenarios/park_and_go/stage_cruise.h"
#include "modules/planning/scenarios/park_and_go/stage_pre_cruise.h"

namespace apollo {
namespace planning {

using apollo::hdmap::HDMapUtil;

bool ParkAndGoScenario::Init(std::shared_ptr<DependencyInjector> injector, const std::string& name) {
    if (init_) {
        return true;
    }

    if (!Scenario::Init(injector, name)) {
        AERROR << "failed to init scenario" << Name();
        return false;
    }

    if (!Scenario::LoadConfig<apollo::planning::ScenarioParkAndGoConfig>(&context_.scenario_config)) {
        AERROR << "fail to get config of scenario" << Name();
        return false;
    }

    init_ = true;
    return true;
}

bool ParkAndGoScenario::IsTransferable(const Scenario* const other_scenario, const Frame& frame) {
    // if(1) return false;
    // 前置安全检查
    if (!frame.local_view().planning_command->has_lane_follow_command()) {
        return false;
    }
    if (other_scenario == nullptr || frame.reference_line_info().empty()) {
        return false;
    }

    // 检查终点信息
    const auto routing_end = frame.local_view().end_lane_way_point;
    if (nullptr == routing_end) {
        return false;
    }

    // 获取车辆状态
    const auto vehicle_state_provider = injector_->vehicle_state();
    common::VehicleState vehicle_state = vehicle_state_provider->vehicle_state();
    const double current_x = vehicle_state.x();
    const double current_y = vehicle_state.y();

    //wlh
    const auto& dest_pose = routing_end->pose();
    double dest_x = dest_pose.x();
    double dest_y = dest_pose.y();

    double dx = dest_x - current_x;
    double dy = dest_y - current_y;
    double distance = std::sqrt(dx * dx + dy * dy);
    

    // 施工绕行场景判断
    static bool shigong_judged = false;
    static int shigong_num = 0;
    static double primary_pos_x = 0.0;
    static double primary_pos_y = 0.0;
    static bool initial_position_set = false;

    // 在首次进入时设置初始位置（只设置一次）
    if (!initial_position_set) {
        primary_pos_x = current_x;
        primary_pos_y = current_y;
        initial_position_set = true;
        AINFO << "Initial position set - X: " << primary_pos_x << " Y: " << primary_pos_y;
    }

    // 获取参考线信息和障碍物信息
    const auto& reference_line_info = frame.reference_line_info().front();
    const auto& path_decision = reference_line_info.path_decision();
    const int obstacle_count = path_decision.obstacles().Items().size();
    // wlh

    // if (obstacle_count <= 1) {
    //     return false;
    // }
    // 施工绕行-11场景
    if (obstacle_count > 30) {
        if (distance < 100.0) {
            return false;
        }
        return true;
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
            for (const auto* obstacle : path_decision.obstacles().Items()) {
                if (!obstacle)
                    continue;

                const auto& perception = obstacle->Perception();
                if (std::abs(perception.position().x() - fourth_largest_x) < kEpsilon) {
                    // 判断y坐标是否小于车辆初始y坐标
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

    // 如果满足施工绕行条件，直接返回true
    if (shigong_judged && (shigong_num == 2 || shigong_num == 4)) {
        AINFO << "current_x " << current_x << " " << primary_pos_x;
        if (current_x <= primary_pos_x + 3.5) {
            return true;
        }
    }

    // 原有停车再走逻辑保持不变
    const double max_abs_speed_when_stopped = 1.0;
    bool park_and_go = false;
    const auto& scenario_config = context_.scenario_config;
    auto adc_point = common::util::PointFactory::ToPointENU(vehicle_state);
    double s = 0.0;
    double l = 0.0;
    hdmap::LaneInfoConstPtr lane;

    common::SLPoint dest_sl;
    const auto& reference_line = reference_line_info.reference_line();
    reference_line.XYToSL(routing_end->pose(), &dest_sl);
    const double adc_front_edge_s = reference_line_info.AdcSlBoundary().end_s();

    bool is_ego_on_lane = false;
    bool is_lane_type_city_driving = false;
    HDMapUtil::BaseMap().GetNearestLaneWithDistance(adc_point, 5.0, &lane, &s, &l);
    if (lane != nullptr && lane->IsOnLane({adc_point.x(), adc_point.y()})) {
        is_ego_on_lane = true;
        if (lane->lane().type() == hdmap::Lane::CITY_DRIVING) {
            is_lane_type_city_driving = true;
        }
    }

    const double adc_distance_to_dest = dest_sl.s() - adc_front_edge_s;
    ADEBUG << "adc_distance_to_dest:" << adc_distance_to_dest;
    bool is_distance_far_enough = (adc_distance_to_dest > scenario_config.min_dist_to_dest());

    // wlh
    if (std::fabs(vehicle_state_provider->linear_velocity()) < max_abs_speed_when_stopped
        && adc_distance_to_dest > scenario_config.min_dist_to_dest() && (!is_ego_on_lane || !is_lane_type_city_driving)) {
        park_and_go = true;
    }

    return park_and_go;
}

}  // namespace planning
}  // namespace apollo
