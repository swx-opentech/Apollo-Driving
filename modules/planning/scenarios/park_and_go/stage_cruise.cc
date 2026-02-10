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

#include "modules/planning/scenarios/park_and_go/stage_cruise.h"

#include "cyber/common/log.h"
#include "modules/planning/planning_base/common/frame.h"
#include "modules/planning/planning_base/common/planning_context.h"
#include "modules/planning/planning_base/common/util/common.h"
#include "modules/planning/scenarios/park_and_go/context.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"

namespace apollo {
namespace planning {

using apollo::common::TrajectoryPoint;

StageResult ParkAndGoStageCruise::Process(const TrajectoryPoint& planning_init_point, Frame* frame) {
    ADEBUG << "stage: Cruise";
    CHECK_NOTNULL(frame);

    // 在给定的参考线上处理规划任务
    StageResult result = ExecuteTaskOnReferenceLine(planning_init_point, frame);
    if (result.HasError()) {
        AERROR << "ParkAndGoStageCruise planning error";
    }

    const ReferenceLineInfo& reference_line_info = frame->reference_line_info().front();
    // check ADC status:
    // 1. At routing beginning: stage finished
    // 检查自动驾驶车辆在泊车与巡航场景下的完成状态
    ParkAndGoStatus status = CheckADCParkAndGoCruiseCompleted(reference_line_info);

    // status等于CRUISE_COMPLETE，则调用FinishStage()函数并返回其结果。
    if (status == CRUISE_COMPLETE) {
        return FinishStage();
    }

    // 设置阶段状态为RUNNING，并通过result.SetStageStatus()返回当前运行状态
    return result.SetStageStatus(StageStatusType::RUNNING);
}

// StageResult ParkAndGoStageCruise::FinishStage() {
//     return FinishScenario();
// }
//wlh
StageResult ParkAndGoStageCruise::FinishStage() {
    auto* park_and_go_status = injector_->planning_context()->mutable_planning_status()->mutable_park_and_go(); // 获取规划上下文中的park_and_go状态对象
    park_and_go_status->Clear(); // 清除该状态对象的所有数据
    park_and_go_status->set_in_check_stage(false); // 将in_check_stage标志设置为false，表示不再处于检查阶段
    return FinishScenario(); // 调用FinishScenario()函数，结束当前场景
}

// ParkAndGoStageCruise::ParkAndGoStatus ParkAndGoStageCruise::CheckADCParkAndGoCruiseCompleted(
//         const ReferenceLineInfo& reference_line_info) {
//     const auto& reference_line = reference_line_info.reference_line();

//     // check l delta
//     const common::math::Vec2d adc_position = {injector_->vehicle_state()->x(), injector_->vehicle_state()->y()};
//     common::SLPoint adc_position_sl;
//     reference_line.XYToSL(adc_position, &adc_position_sl);

//     const double kLBuffer = 0.5;
//     if (std::fabs(adc_position_sl.l()) < kLBuffer) {
//         ADEBUG << "cruise completed";
//         return CRUISE_COMPLETE;
//     }
//     // const auto& path_decision = reference_line_info.path_decision();
//     // const int obstacle_count = path_decision.obstacles().Items().size();
//     // if (obstacle_count <= 1) {
//     //     ADEBUG << "cruise completed";
//     //     return CRUISE_COMPLETE;
//     // }
//     /* loose heading check, so that ADC can enter LANE_FOLLOW scenario sooner
//      * which is more sophisticated
//     // heading delta
//     const double adc_heading =
//         common::VehicleStateProvider::Instance()->heading();
//     const auto reference_point =
//         reference_line.GetReferencePoint(adc_position_sl.s());
//     const auto path_point = reference_point.ToPathPoint(adc_position_sl.s());
//     ADEBUG << "adc_position_sl.l():[" << adc_position_sl.l() << "]";
//     ADEBUG << "adc_heading - path_point.theta():[" << adc_heading << "]"
//            << "[" << path_point.theta() << "]";
//     const double kHeadingBuffer = 0.1;
//     if (std::fabs(adc_heading - path_point.theta()) < kHeadingBuffer) {
//       ADEBUG << "cruise completed";
//       return CRUISE_COMPLETE;
//     }
//     */

//     return CRUISING;
// }

ParkAndGoStageCruise::ParkAndGoStatus ParkAndGoStageCruise::CheckADCParkAndGoCruiseCompleted(const ReferenceLineInfo& reference_line_info) {
    int obstacle_flag = 0; // 判断是否有障碍物
    int num = 0; // 障碍物数量
    const auto& reference_line = reference_line_info.reference_line(); // 获取参考线
    const auto& path_decision = reference_line_info.path_decision(); // 获取路径规划结果

    // check l delta
    // 将车辆当前位置从XY坐标系转换为SL坐标系
    const common::math::Vec2d adc_position = {injector_->vehicle_state()->x(), injector_->vehicle_state()->y()};
    common::SLPoint adc_position_sl;
    reference_line.XYToSL(adc_position, &adc_position_sl);

    double adc_s = adc_position_sl.s();

    // 遍历路径决策中的障碍物，判断是否存在前方障碍物
    for (const auto* obstacle : path_decision.obstacles().Items()) {
        // 遍历所有障碍物，跳过空指针
        if (!obstacle) {
            continue;
        }
        // 获取障碍物的感知SL边界（PerceptionSLBoundary）
        const auto& sl_boundary = obstacle->PerceptionSLBoundary();

        // 只判断前方障碍物
        if (sl_boundary.start_s() > adc_s) {
            num++;
            if(num>1){
               obstacle_flag = 1;
               num = 0;
               break;  // 找到一个就够了
            }
            // ADEBUG << "Found obstacle in front: " << obstacle->Id()
            //        << " s=[" << sl_boundary.start_s() << "," << sl_boundary.end_s() << "]"
            //        << " l=[" << sl_boundary.start_l() << "," << sl_boundary.end_l() << "]";
        }
    }

    const double kLBuffer = 0.5;
    // 横向位置相对于参考线的偏差 且 obstacle_flag 为假，并返回 CRUISE_COMPLETE
    if ((std::fabs(adc_position_sl.l()) < kLBuffer)&& !obstacle_flag) {
        ADEBUG << "cruise completed";
        return CRUISE_COMPLETE;
    }


    // const auto& path_decision = reference_line_info.path_decision();
    // const int obstacle_count = path_decision.obstacles().Items().size();
    // if (obstacle_count <= 1) {
    //     ADEBUG << "cruise completed";
    //     return CRUISE_COMPLETE;
    // }

    /* loose heading check, so that ADC can enter LANE_FOLLOW scenario sooner
     * which is more sophisticated
    // heading delta
    const double adc_heading =
        common::VehicleStateProvider::Instance()->heading();
    const auto reference_point =
        reference_line.GetReferencePoint(adc_position_sl.s());
    const auto path_point = reference_point.ToPathPoint(adc_position_sl.s());
    ADEBUG << "adc_position_sl.l():[" << adc_position_sl.l() << "]";
    ADEBUG << "adc_heading - path_point.theta():[" << adc_heading << "]"
           << "[" << path_point.theta() << "]";
    const double kHeadingBuffer = 0.1;
    if (std::fabs(adc_heading - path_point.theta()) < kHeadingBuffer) {
      ADEBUG << "cruise completed";
      return CRUISE_COMPLETE;
    }
    */

    return CRUISING;
}

}  // namespace planning
}  // namespace apollo
