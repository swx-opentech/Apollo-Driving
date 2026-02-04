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

    StageResult result = ExecuteTaskOnReferenceLine(planning_init_point, frame);
    if (result.HasError()) {
        AERROR << "ParkAndGoStageCruise planning error";
    }

    const ReferenceLineInfo& reference_line_info = frame->reference_line_info().front();
    // check ADC status:
    // 1. At routing beginning: stage finished
    ParkAndGoStatus status = CheckADCParkAndGoCruiseCompleted(reference_line_info);

    if (status == CRUISE_COMPLETE) {
        return FinishStage();
    }
    return result.SetStageStatus(StageStatusType::RUNNING);
}

// StageResult ParkAndGoStageCruise::FinishStage() {
//     return FinishScenario();
// }
//wlh
StageResult ParkAndGoStageCruise::FinishStage() {
    auto* park_and_go_status = injector_->planning_context()->mutable_planning_status()->mutable_park_and_go();
    park_and_go_status->Clear();
    park_and_go_status->set_in_check_stage(false);
    return FinishScenario();
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
    int obstacle_flag = 0; 
    int num = 0;
    const auto& reference_line = reference_line_info.reference_line();
    const auto& path_decision = reference_line_info.path_decision();

    // check l delta
    const common::math::Vec2d adc_position = {injector_->vehicle_state()->x(), injector_->vehicle_state()->y()};
    common::SLPoint adc_position_sl;
    reference_line.XYToSL(adc_position, &adc_position_sl);

    double adc_s = adc_position_sl.s();
    for (const auto* obstacle : path_decision.obstacles().Items()) {
        if (!obstacle) {
            continue;
        }
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
