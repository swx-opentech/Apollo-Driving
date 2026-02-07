# 紧急停车
### 由 `swx-opentech` 研究并翻译！

---
## stage_approach.cc
`EmergencyStopStageApproach` : 该阶段用于主车急停前减速，主车速度达到阈值后退出。
### Process 函数
* 获取场景上下文 `scenario_config_.CopyFrom(GetContextAs<EmergencyStopContext>()->scenario_config);`
* 设置车辆的紧急信号灯 `frame->mutable_reference_line_info()->front().SetEmergencyLight();`
* 创建虚拟障碍物
    > 获取参考路径、速度信息
   ```cpp
    const auto& reference_line_info = frame->reference_line_info().front(); // 获取参考路径信息
    const auto& reference_line = reference_line_info.reference_line();      // 路径几何信息
    const double adc_speed = injector_->vehicle_state()->linear_velocity(); // 获取车辆速度
    const double adc_front_edge_s = reference_line_info.AdcSlBoundary().end_s(); // 计算车辆前缘位置
    const double stop_distance = scenario_config_.stop_distance(); // 获取停车距离配置
    ```
    >
    > 设置是否需要紧急停车标志 `stop_fence_exist = false;`
    >
    > 准备紧急停车信息并转换坐标、判断距离 `if (emergency_stop_status.has_stop_fence_point()) {...}`
    ```cpp
    // 若转换后的s值大于自车前缘位置adc_front_edge_s，则标记存在停止线
    if (stop_fence_sl.s() > adc_front_edge_s) {
      stop_fence_exist = true;
      stop_line_s = stop_fence_sl.s();
    }
    ```
    > 计算停车距离与停止线位置 `stop_line_s = adc_front_edge_s + travel_distance + stop_distance + kBuffer;`
    >
    > 设置到紧急停车围栏点 `emergency_stop_fence_point->set_x(stop_fence_point.x());`
    >
* 构建一个紧急停车决策 `planning::util::BuildStopDecision(...)`
* 执行任务 `StageResult result = ExecuteTaskOnReferenceLine(planning_init_point, frame);`
* 判断结果
    ```cpp
    // 若当前车速adc_speed小于等于该阈值，则调用FinishStage()结束当前阶段
    if (adc_speed <= max_adc_stop_speed) {
        return FinishStage();
    }

    // 否则返回运行中状态
    return result.SetStageStatus(StageStatusType::RUNNING);
    ```

---
### FinishStage 函数
该函数用于标记紧急停止接近阶段结束，并指定下一个阶段为待命状态。

---
## stage_standby.cc
`EmergencyStopStageStandby` : 该阶段用于主车保持紧急停车状态。
### Process 函数


* 获取场景上下文 `scenario_config_.CopyFrom(GetContextAs<EmergencyStopContext>()->scenario_config);`
* 停止线计算
    > 获取参考路径、速度信息  `const auto& reference_line_info = frame->reference_line_info().front();` ...
    >
    > 检查停止线等信息 坐标变换 `if (stop_fence_sl.s() > adc_front_edge_s) {...}`
    >
    > 设置紧急停车的停止线位置 `stop_line_s = adc_front_edge_s + stop_distance + kBuffer;`
* 构建一个紧急停车决策
    > 创建虚拟障碍物
    ``` cpp
    const std::string virtual_obstacle_id = "EMERGENCY_STOP";
    planning::util::BuildStopDecision(...)

    ```
* 执行任务 `StageResult result = ExecuteTaskOnReferenceLine(planning_init_point, frame);`
* 判断任务状态
    > 获取`frame`中的驾驶动作消息`pad_msg_driving_action`。若动作不是`"STOP"`，则调用`FinishStage()`结束当前阶段。否则，设置阶段状态为`"RUNNING"`并返回结果。
    
