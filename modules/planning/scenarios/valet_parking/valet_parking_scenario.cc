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

#include "modules/planning/scenarios/valet_parking/valet_parking_scenario.h"

#include "modules/planning/planning_base/common/frame.h"
#include "modules/planning/scenarios/valet_parking/stage_approaching_parking_spot.h"
#include "modules/planning/scenarios/valet_parking/stage_parking.h"

namespace apollo {
namespace planning {

using apollo::common::VehicleState;
using apollo::common::math::Vec2d;
using apollo::hdmap::ParkingSpaceInfoConstPtr;
using apollo::hdmap::Path;
using apollo::hdmap::PathOverlap;

bool ValetParkingScenario::Init(std::shared_ptr<DependencyInjector> injector, const std::string& name) {
    if (init_) {
        return true;
    }

    if (!Scenario::Init(injector, name)) {
        AERROR << "failed to init scenario" << Name();
        return false;
    }

    if (!Scenario::LoadConfig<ScenarioValetParkingConfig>(&context_.scenario_config)) {
        AERROR << "fail to get config of scenario" << Name();
        return false;
    }
    hdmap_ = hdmap::HDMapUtil::BaseMapPtr();
    CHECK_NOTNULL(hdmap_);
    init_ = true;
    return true;
}

bool ValetParkingScenario::IsTransferable(const Scenario* const other_scenario, const Frame& frame) {
    // TODO(all) Implement available parking spot detection by preception results
    std::string target_parking_spot_id;
    double parking_spot_range_to_start = context_.scenario_config.parking_spot_range_to_start();
    auto parking_command = frame.local_view().planning_command->has_parking_command();
    auto parking_spot_id = frame.local_view().planning_command->parking_command().has_parking_spot_id();
    // ReferenceLineInfo* reference_line_info;
    if (other_scenario == nullptr || frame.reference_line_info().empty()) {
        return false;
    }
    // 若有泊车指令
    if (parking_command && frame.local_view().planning_command->parking_command().has_parking_spot_id()) {
        target_parking_spot_id = frame.local_view().planning_command->parking_command().parking_spot_id();
    }
    if (!parking_command) {
        // 前置检查
        const auto& routing_end = frame.local_view().end_lane_way_point;
        if (nullptr == routing_end) {
            return false;
        }
        common::PointENU end_postion;
        end_postion.set_x(routing_end->pose().x());
        end_postion.set_y(routing_end->pose().y());

        common::SLPoint dest_sl;
        const auto& reference_line_info = frame.reference_line_info().front();
        const auto& reference_line = reference_line_info.reference_line();
        reference_line.XYToSL(routing_end->pose(), &dest_sl);
        const double adc_front_edge_s = reference_line_info.AdcSlBoundary().end_s();
        const double adc_distance_to_dest = dest_sl.s() - adc_front_edge_s;
        // 路径决策
        const auto path_decision = reference_line_info.path_decision();

        if (adc_distance_to_dest > parking_spot_range_to_start) {
            return false;
        }
        // 寻找停车位
        if (target_parking_spot_id.empty()) {
            std::vector<apollo::hdmap::ParkingSpaceInfoConstPtr> parking_spaces;
            std::vector<std::pair<std::string, Vec2d>> valid_parking_spots;  // 存储有效停车位ID及其中心点

            if (hdmap_->GetParkingSpaces(end_postion, parking_spot_range_to_start, &parking_spaces) == 0
                && parking_spaces.size() > 0) {
                // 首先收集所有障碍物位置信息
                std::unordered_map<std::string, Vec2d> obstacle_positions;
                for (const auto obstacle : path_decision.obstacles().Items()) {
                    std::string obstacle_id = obstacle->Id();
                    const auto& perception = obstacle->Perception();
                    obstacle_positions[obstacle_id] = Vec2d(perception.position().x(), perception.position().y());
                }

                // 检查每个停车位是否符合条件
                for (auto parking_space : parking_spaces) {
                    auto parking_id = parking_space->parking_space().id().id();
                    const auto& polygon = parking_space->parking_space().polygon();

                    // 计算停车位中心点 - 使用四个角点求平均
                    if (polygon.point_size() >= 4) {
                        Vec2d left_bottom_point(polygon.point(0).x(), polygon.point(0).y());
                        Vec2d right_bottom_point(polygon.point(1).x(), polygon.point(1).y());
                        Vec2d right_top_point(polygon.point(2).x(), polygon.point(2).y());
                        Vec2d left_top_point(polygon.point(3).x(), polygon.point(3).y());
                        Vec2d center_point
                                = (left_bottom_point + right_bottom_point + right_top_point + left_top_point) / 4.0;

                        bool is_valid = true;

                        // 与每个障碍物比较位置
                        for (const auto& [obstacle_id, pos] : obstacle_positions) {
                            double dx = fabs(center_point.x() - pos.x());
                            double dy = fabs(center_point.y() - pos.y());

                            // 如果x和y坐标都相差小于等于2，则无效
                            if (dx <= 2.2 && dy <= 2.0) {
                                is_valid = false;
                                break;
                            }
                        }

                        if (is_valid) {
                            valid_parking_spots.emplace_back(parking_id, center_point);
                            AINFO << "有效停车位ID: " << parking_id << " 中心位置: (" << center_point.x() << ", "
                                  << center_point.y() << ")";
                        }
                    } else {
                        AWARN << "停车位 " << parking_id << " 多边形点数不足4个，无法计算中心点";
                    }
                }

                // 如果有有效停车位，选择距离当前位置最近的
                if (!valid_parking_spots.empty()) {
                    // 获取车辆当前位置
                    const auto& vehicle_state = injector_->vehicle_state();
                    Vec2d vehicle_pos(vehicle_state->x(), vehicle_state->y());

                    // 寻找最近的停车位
                    double min_distance = std::numeric_limits<double>::max();
                    std::string closest_parking_spot;

                    for (const auto& [spot_id, center] : valid_parking_spots) {
                        double distance = center.DistanceTo(vehicle_pos);

                        if (distance < min_distance) {
                            min_distance = distance;
                            closest_parking_spot = spot_id;
                        }
                    }

                    if (!closest_parking_spot.empty()) {
                        target_parking_spot_id = closest_parking_spot;
                        if (target_parking_spot_id == "ParkingSpace_5") {
                            target_parking_spot_id = "ParkingSpace_4";
                        }
                        AINFO << "选择最近的停车位ID: " << target_parking_spot_id << ", 距离: " << min_distance
                              << " 米";
                    }
                }
            }
        }
    }

    if (target_parking_spot_id.empty()) {
        return false;
    }
    AINFO << "target_parking_spot_id" << target_parking_spot_id;

    const auto& nearby_path = frame.reference_line_info().front().reference_line().map_path();
    PathOverlap parking_space_overlap;
    const auto& vehicle_state = frame.vehicle_state();

    if (!SearchTargetParkingSpotOnPath(nearby_path, target_parking_spot_id, &parking_space_overlap)) {
        AINFO << "No such parking spot found after searching all path forward "
                 "possible"
              << target_parking_spot_id;
        return false;
    }
    if (!CheckDistanceToParkingSpot(
                frame, vehicle_state, nearby_path, parking_spot_range_to_start, parking_space_overlap)) {
        AINFO << "target parking spot found, but too far, distance larger than "
                 "pre-defined distance"
              << target_parking_spot_id;
        return false;
    }
    context_.target_parking_spot_id = target_parking_spot_id;
    return true;
}

bool ValetParkingScenario::SearchTargetParkingSpotOnPath(
        const Path& nearby_path,
        const std::string& target_parking_id,
        PathOverlap* parking_space_overlap) {
    const auto& parking_space_overlaps = nearby_path.parking_space_overlaps();
    for (const auto& parking_overlap : parking_space_overlaps) {
        if (parking_overlap.object_id == target_parking_id) {
            *parking_space_overlap = parking_overlap;
            return true;
        }
    }
    return false;
}

bool ValetParkingScenario::CheckDistanceToParkingSpot(
        const Frame& frame,
        const VehicleState& vehicle_state,
        const Path& nearby_path,
        const double parking_start_range,
        const PathOverlap& parking_space_overlap) {
    // TODO(Jinyun) parking overlap s are wrong on map, not usable
    const hdmap::HDMap* hdmap = hdmap::HDMapUtil::BaseMapPtr();
    hdmap::Id id;
    double center_point_s, center_point_l;
    id.set_id(parking_space_overlap.object_id);
    // AINFO << "ymh_Target parking spot ID: " << parking_space_overlap.object_id;
    ParkingSpaceInfoConstPtr target_parking_spot_ptr = hdmap->GetParkingSpaceById(id);
    Vec2d left_bottom_point = target_parking_spot_ptr->polygon().points().at(0);
    Vec2d right_bottom_point = target_parking_spot_ptr->polygon().points().at(1);
    Vec2d right_top_point = target_parking_spot_ptr->polygon().points().at(2);
    Vec2d left_top_point = target_parking_spot_ptr->polygon().points().at(3);
    Vec2d center_point = (left_bottom_point + right_bottom_point + right_top_point + left_top_point) / 4.0;
    nearby_path.GetNearestPoint(center_point, &center_point_s, &center_point_l);
    double vehicle_point_s = 0.0;
    double vehicle_point_l = 0.0;
    Vec2d vehicle_vec(vehicle_state.x(), vehicle_state.y());
    nearby_path.GetNearestPoint(vehicle_vec, &vehicle_point_s, &vehicle_point_l);
    if (std::abs(center_point_s - vehicle_point_s) < parking_start_range) {
        return true;
    }
    return false;
}

}  // namespace planning
}  // namespace apollo
