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

#include "modules/planning/tasks/open_space_pre_stop_decider/open_space_pre_stop_decider.h"

#include <memory>
#include <string>
#include <vector>

#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/map/pnc_map/path.h"
#include "modules/planning/planning_base/common/planning_context.h"
#include "modules/planning/planning_base/common/util/common.h"

namespace apollo {
namespace planning {

using apollo::common::ErrorCode;
using apollo::common::Status;
using apollo::common::VehicleState;
using apollo::common::math::Vec2d;
using apollo::hdmap::ParkingSpaceInfoConstPtr;

bool OpenSpacePreStopDecider::Init(
        const std::string& config_dir,
        const std::string& name,
        const std::shared_ptr<DependencyInjector>& injector) {
    if (!Decider::Init(config_dir, name, injector)) {
        return false;
    }
    // Load the config this task.
    bool res = Decider::LoadConfig<OpenSpacePreStopDeciderConfig>(&config_);
    AINFO << "Load config:" << config_.DebugString();
    return res;
}

Status OpenSpacePreStopDecider::Process(Frame* frame, ReferenceLineInfo* reference_line_info) {
    CHECK_NOTNULL(frame);
    CHECK_NOTNULL(reference_line_info);
    double target_s = 0.0;
    const auto& stop_type = config_.stop_type();
    switch (stop_type) {
    case OpenSpacePreStopDeciderConfig::PARKING:
        if (!CheckParkingSpotPreStop(frame, reference_line_info, &target_s)) {
            const std::string msg = "Checking parking spot pre stop fails";
            AERROR << msg;
            return Status(ErrorCode::PLANNING_ERROR, msg);
        }
        SetParkingSpotStopFence(target_s, frame, reference_line_info);
        break;
    case OpenSpacePreStopDeciderConfig::PULL_OVER:
        if (!CheckPullOverPreStop(frame, reference_line_info, &target_s)) {
            const std::string msg = "Checking pull over pre stop fails";
            AERROR << msg;
            return Status(ErrorCode::PLANNING_ERROR, msg);
        }
        SetPullOverStopFence(target_s, frame, reference_line_info);
        break;
    default:
        const std::string msg = "This stop type not implemented";
        AERROR << msg;
        return Status(ErrorCode::PLANNING_ERROR, msg);
    }
    return Status::OK();
}

bool OpenSpacePreStopDecider::CheckPullOverPreStop(
        Frame* const frame,
        ReferenceLineInfo* const reference_line_info,
        double* target_s) {
    *target_s = 0.0;
    const auto& pull_over_status = injector_->planning_context()->planning_status().pull_over();
    if (pull_over_status.has_position() && pull_over_status.position().has_x() && pull_over_status.position().has_y()) {
        common::SLPoint pull_over_sl;
        const auto& reference_line = reference_line_info->reference_line();
        reference_line.XYToSL(pull_over_status.position(), &pull_over_sl);
        *target_s = pull_over_sl.s();
    }
    return true;
}

bool OpenSpacePreStopDecider::CheckParkingSpotPreStop(
        Frame* const frame,
        ReferenceLineInfo* const reference_line_info,
        double* target_s) {
    const auto& target_parking_spot_id = frame->open_space_info().target_parking_spot_id();
    const auto& nearby_path = reference_line_info->reference_line().map_path();
    if (target_parking_spot_id.empty()) {
        AERROR << "no target parking spot id found when setting pre stop fence";
        return false;
    }

    double target_area_center_s = 0.0;
    bool target_area_found = false;
    const auto& parking_space_overlaps = nearby_path.parking_space_overlaps();
    ParkingSpaceInfoConstPtr target_parking_spot_ptr;
    const hdmap::HDMap* hdmap = hdmap::HDMapUtil::BaseMapPtr();
    for (const auto& parking_overlap : parking_space_overlaps) {
        if (parking_overlap.object_id == target_parking_spot_id) {
            // TODO(Jinyun) parking overlap s are wrong on map, not usable
            // target_area_center_s =
            //     (parking_overlap.start_s + parking_overlap.end_s) / 2.0;
            hdmap::Id id;
            id.set_id(parking_overlap.object_id);
            target_parking_spot_ptr = hdmap->GetParkingSpaceById(id);
            Vec2d left_bottom_point = target_parking_spot_ptr->polygon().points().at(0);
            Vec2d right_bottom_point = target_parking_spot_ptr->polygon().points().at(1);
            Vec2d right_up_point = target_parking_spot_ptr->polygon().points().at(2);
            Vec2d left_up_point = target_parking_spot_ptr->polygon().points().at(3);
            Vec2d center_point = (left_bottom_point + right_bottom_point + right_up_point + left_up_point) / 4.0;
            double center_l;
            nearby_path.GetNearestPoint(center_point, &target_area_center_s, &center_l);
            target_area_found = true;
        }
    }

    if (!target_area_found) {
        AERROR << "no target parking spot found on reference line";
        return false;
    }
    *target_s = target_area_center_s;
    return true;
}

void OpenSpacePreStopDecider::SetParkingSpotStopFence(
        const double target_s,
        Frame* const frame,
        ReferenceLineInfo* const reference_line_info) {
    double parking_spot_range_to_start = 20.0;
    const auto path_decision = reference_line_info->path_decision();
    const auto& routing_end = frame->local_view().end_lane_way_point;
    int parking_choose_num = 0;
    common::PointENU end_postion;
    end_postion.set_x(routing_end->pose().x());
    end_postion.set_y(routing_end->pose().y());
    const hdmap::HDMap* hdmap = hdmap::HDMapUtil::BaseMapPtr();
    std::vector<apollo::hdmap::ParkingSpaceInfoConstPtr> parking_spaces;
    // 从HDMap获取停车位数据
    if (hdmap->GetParkingSpaces(end_postion, parking_spot_range_to_start, &parking_spaces) != 0
        || parking_spaces.empty()) {
        AERROR << "无法获取停车位信息";
    }

    std::vector<std::pair<std::string, Vec2d>> valid_parking_spots;
    std::string target_parking_spot_id;

    // 障碍物相关数据
    std::unordered_map<std::string, Vec2d> obstacle_positions;
    double distance_with_1541 = 0;  // 专门存储与1541的距离
    double distance_with_5461 = 0;  // 专门存储与5461的距离
    Vec2d obstacle_1541_pos(0, 0);
    Vec2d obstacle_5461_pos(0, 0);
    bool has_1541 = false;
    bool has_5461 = false;

    // 收集障碍物信息
    double min_sum = std::numeric_limits<double>::max();
    double max_sum = std::numeric_limits<double>::lowest();

    for (const auto obstacle : path_decision->obstacles().Items()) {
        std::string obstacle_id = obstacle->Id();
        const auto& perception = obstacle->Perception();
        Vec2d obs_pos(perception.position().x(), perception.position().y());
        obstacle_positions[obstacle_id] = obs_pos;

        double current_sum = obs_pos.x() + obs_pos.y();

        // 更新1541障碍物（x+y最小，最靠近左下）
        if (!has_1541 || current_sum < min_sum) {
            obstacle_1541_pos = obs_pos;
            min_sum = current_sum;
            has_1541 = true;
        }

        // 更新5461障碍物（x+y最大，最靠近右上）
        if (!has_5461 || current_sum > max_sum) {
            obstacle_5461_pos = obs_pos;
            max_sum = current_sum;
            has_5461 = true;
        }
    }

    // 筛选有效停车位
    for (auto parking_space : parking_spaces) {
        auto parking_id = parking_space->parking_space().id().id();
        const auto& polygon = parking_space->parking_space().polygon();

        if (polygon.point_size() >= 4) {
            Vec2d left_bottom_point(polygon.point(0).x(), polygon.point(0).y());
            Vec2d right_bottom_point(polygon.point(1).x(), polygon.point(1).y());
            Vec2d right_top_point(polygon.point(2).x(), polygon.point(2).y());
            Vec2d left_top_point(polygon.point(3).x(), polygon.point(3).y());
            Vec2d center_point = (left_bottom_point + right_bottom_point + right_top_point + left_top_point) / 4.0;

            bool is_valid = true;
            for (const auto& [obstacle_id, pos] : obstacle_positions) {
                double dx = fabs(center_point.x() - pos.x());
                double dy = fabs(center_point.y() - pos.y());
                if (dx <= 2.2 && dy <= 2.0) {
                    is_valid = false;
                    break;
                }
            }

            if (is_valid) {
                valid_parking_spots.emplace_back(parking_id, center_point);
                // AINFO << "有效停车位: " << parking_id << " 中心点: (" << center_point.x() << ", " << center_point.y()
                //       << ")";
            }
        }
    }

    // 选择最近停车位并计算距离
    if (!valid_parking_spots.empty()) {
        const auto& vehicle_state = injector_->vehicle_state();
        Vec2d vehicle_pos(vehicle_state->x(), vehicle_state->y());

        double min_distance = std::numeric_limits<double>::max();
        std::string closest_parking_spot;
        Vec2d closest_parking_center;

        for (const auto& [spot_id, center] : valid_parking_spots) {
            double distance = center.DistanceTo(vehicle_pos);
            if (distance < min_distance) {
                min_distance = distance;
                closest_parking_spot = spot_id;
                closest_parking_center = center;
            }
        }

        if (!closest_parking_spot.empty()) {
            target_parking_spot_id = closest_parking_spot;
            // 特殊处理ParkingSpace_5
            if (target_parking_spot_id == "ParkingSpace_5") {
                target_parking_spot_id = "ParkingSpace_4";
            }
            if (has_1541) {
                distance_with_1541 = closest_parking_center.DistanceTo(obstacle_1541_pos);
                // AINFO << "与障碍物1541的距离: " << distance_with_1541 << " 米";
            }
            if (has_5461) {
                distance_with_5461 = closest_parking_center.DistanceTo(obstacle_5461_pos);
                // AINFO << "与障碍物5461的距离: " << distance_with_5461 << " 米";
            }
            if (distance_with_1541 > 2 && distance_with_1541 < 5) {
                parking_choose_num = 76;
            } else if (distance_with_1541 > 10 && distance_with_1541 < 15) {
                if (distance_with_5461 > 8)
                    parking_choose_num = 64;
                else if (distance_with_5461 < 8)
                    parking_choose_num = 71;
            } else if (distance_with_1541 > 27 && distance_with_1541 < 28.5) {
                parking_choose_num = 85;
            } else
                parking_choose_num = 4;
        }
    } else {
        AWARN << "未找到符合条件的有效停车位";
    }
    const auto& nearby_path = reference_line_info->reference_line().map_path();
    const double adc_front_edge_s = reference_line_info->AdcSlBoundary().end_s();
    const double front_edge_to_center
            = common::VehicleConfigHelper::Instance()->GetConfig().vehicle_param().front_edge_to_center();
    double ego_s = adc_front_edge_s - front_edge_to_center;
    const VehicleState& vehicle_state = frame->vehicle_state();
    double stop_line_s = 0.0;
    double stop_distance_to_target = config_.stop_distance_to_target();
    double static_linear_velocity_epsilon = 1.0e-2;
    static constexpr double kStopBuffer = 0.2;
    CHECK_GE(stop_distance_to_target, 1.0e-8);
    stop_line_s = target_s + front_edge_to_center + config_.stop_buffer_to_target();
    const std::string stop_wall_id = OPEN_SPACE_STOP_ID;
    std::vector<std::string> wait_for_obstacles;
    frame->mutable_open_space_info()->set_open_space_pre_stop_fence_s(stop_line_s);
    if (parking_choose_num == 71) {
        util::BuildStopDecision(
                stop_wall_id,
                stop_line_s,
                -3.5,
                StopReasonCode::STOP_REASON_PRE_OPEN_SPACE_STOP,
                wait_for_obstacles,
                "OpenSpacePreStopDecider",
                frame,
                reference_line_info);
    } else if (parking_choose_num == 4) {
        util::BuildStopDecision(
                stop_wall_id,
                stop_line_s,
                0.0,
                StopReasonCode::STOP_REASON_PRE_OPEN_SPACE_STOP,
                wait_for_obstacles,
                "OpenSpacePreStopDecider",
                frame,
                reference_line_info);
    } else if (parking_choose_num == 64) {
        util::BuildStopDecision(
                stop_wall_id,
                stop_line_s,
                -3.5,
                StopReasonCode::STOP_REASON_PRE_OPEN_SPACE_STOP,
                wait_for_obstacles,
                "OpenSpacePreStopDecider",
                frame,
                reference_line_info);
    } else if (parking_choose_num == 85) {
        util::BuildStopDecision(
                stop_wall_id,
                stop_line_s,
                -2.5,  //-2.5
                StopReasonCode::STOP_REASON_PRE_OPEN_SPACE_STOP,
                wait_for_obstacles,
                "OpenSpacePreStopDecider",
                frame,
                reference_line_info);
    } else
        util::BuildStopDecision(
                stop_wall_id,
                stop_line_s,
                0.0,
                StopReasonCode::STOP_REASON_PRE_OPEN_SPACE_STOP,
                wait_for_obstacles,
                "OpenSpacePreStopDecider",
                frame,
                reference_line_info);
}

void OpenSpacePreStopDecider::SetPullOverStopFence(
        const double target_s,
        Frame* const frame,
        ReferenceLineInfo* const reference_line_info) {
    const auto& nearby_path = reference_line_info->reference_line().map_path();
    const double adc_front_edge_s = reference_line_info->AdcSlBoundary().end_s();
    const VehicleState& vehicle_state = frame->vehicle_state();
    double stop_line_s = 0.0;
    double stop_distance_to_target = config_.stop_distance_to_target();
    double static_linear_velocity_epsilon = 1.0e-2;
    CHECK_GE(stop_distance_to_target, 1.0e-8);
    double target_vehicle_offset = target_s - adc_front_edge_s;
    if (target_vehicle_offset > stop_distance_to_target) {
        stop_line_s = target_s - stop_distance_to_target;
    } else {
        if (!frame->open_space_info().pre_stop_rightaway_flag()) {
            // TODO(Jinyun) Use constant comfortable deacceleration rather than
            // distance by config to set stop fence
            stop_line_s = adc_front_edge_s + config_.rightaway_stop_distance();
            if (std::abs(vehicle_state.linear_velocity()) < static_linear_velocity_epsilon) {
                stop_line_s = adc_front_edge_s;
            }
            *(frame->mutable_open_space_info()->mutable_pre_stop_rightaway_point())
                    = nearby_path.GetSmoothPoint(stop_line_s);
            frame->mutable_open_space_info()->set_pre_stop_rightaway_flag(true);
        } else {
            double stop_point_s = 0.0;
            double stop_point_l = 0.0;
            nearby_path.GetNearestPoint(
                    frame->open_space_info().pre_stop_rightaway_point(), &stop_point_s, &stop_point_l);
            stop_line_s = stop_point_s;
        }
    }

    const std::string stop_wall_id = OPEN_SPACE_STOP_ID;
    std::vector<std::string> wait_for_obstacles;
    frame->mutable_open_space_info()->set_open_space_pre_stop_fence_s(stop_line_s);
    util::BuildStopDecision(
            stop_wall_id,
            stop_line_s,
            0.0,
            StopReasonCode::STOP_REASON_PRE_OPEN_SPACE_STOP,
            wait_for_obstacles,
            "OpenSpacePreStopDecider",
            frame,
            reference_line_info);
}
}  // namespace planning
}  // namespace apollo
