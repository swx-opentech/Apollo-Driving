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

#include "modules/planning/scenarios/park_and_go/stage_check.h"

#include "cyber/common/log.h"
#include "modules/planning/planning_base/common/frame.h"
#include "modules/planning/scenarios/park_and_go/context.h"
#include "modules/planning/scenarios/park_and_go/util.h"

namespace apollo {
namespace planning {

using apollo::common::TrajectoryPoint;

StageResult ParkAndGoStageCheck::Process(const TrajectoryPoint& planning_init_point, Frame* frame) {
    ADEBUG << "stage: Check";
    // 保关键指针非空，避免后续操作出现未定义行为
    CHECK_NOTNULL(frame);
    CHECK_NOTNULL(context_);

    ADCInitStatus(); // 始化自动驾驶车辆（ADC）的状态
    // 当前帧（frame）中的开放空间信息，标记车辆正处于开放空间轨迹规划状态（true）
    frame->mutable_open_space_info()->set_is_on_open_space_trajectory(true); 
    StageResult result = ExecuteTaskOnOpenSpace(frame); // 执行开放空间中的任务
    if (result.HasError()) { // 检查任务是否出错
        AERROR << "ParkAndGoStageAdjust planning error";
        return result.SetStageStatus(StageStatusType::ERROR);
    }

    // 检查车辆是否具备进入巡航状态的条件
    bool ready_to_cruise = CheckADCReadyToCruise(
            injector_->vehicle_state(), frame, GetContextAs<ParkAndGoContext>()->scenario_config);
    return FinishStage(ready_to_cruise);
}

StageResult ParkAndGoStageCheck::FinishStage(const bool success) {
    // 无论成功与否，都将下一阶段设为"PARK_AND_GO_ADJUST"
    if (success) {
        next_stage_ = "PARK_AND_GO_ADJUST";
        // return FinishScenario();
    } else {
        next_stage_ = "PARK_AND_GO_ADJUST";
    }
    // 将规划上下文中的park_and_go状态标记为不在检查阶段
    injector_->planning_context()->mutable_planning_status()->mutable_park_and_go()->set_in_check_stage(false);
    return StageResult(StageStatusType::FINISHED); // 阶段结束
}
// StageResult ParkAndGoStageCheck::FinishStage(const bool success) {
//     auto* park_and_go_status = injector_->planning_context()->mutable_planning_status()->mutable_park_and_go();
//     park_and_go_status->set_in_check_stage(false);

//     if (success) {
//         park_and_go_status->Clear();
//         return FinishScenario();
//     } else {
//         next_stage_ = "PARK_AND_GO_ADJUST";
//         return StageResult(StageStatusType::FINISHED);
//     }
// }

void ParkAndGoStageCheck::ADCInitStatus() {
    auto* park_and_go_status = injector_->planning_context()->mutable_planning_status()->mutable_park_and_go();
    park_and_go_status->Clear();
    park_and_go_status->mutable_adc_init_position()->set_x(injector_->vehicle_state()->x());
    park_and_go_status->mutable_adc_init_position()->set_y(injector_->vehicle_state()->y());
    park_and_go_status->mutable_adc_init_position()->set_z(0.0);
    park_and_go_status->set_adc_init_heading(injector_->vehicle_state()->heading());
    park_and_go_status->set_in_check_stage(true);
}

}  // namespace planning
}  // namespace apollo
