# 停车起步
### 由 `swx-opentech` 研究并翻译！
用于车辆在远离终点且静止条件下，在非城市车道或匹配不到道路点的位置，通过freespace规划，实现车辆由开放空间驶入道路的功能。

---
## stage_check.cc
### Process 函数
* 检查场景
* 始化自动驾驶车辆（`ADC`）的状态 `ADCInitStatus();`
* 获取帧（frame）中的开放空间信息，标记车辆正处于开放空间轨迹规划状态（true）
    > `frame->mutable_open_space_info()->set_is_on_open_space_planning(true);`
* 执行开放空间中的任务 `StageResult result = ExecuteTaskOnOpenSpace(frame);`
* 检查车辆是否具备进入巡航状态的条件 `bool ready_to_cruise = CheckADCReadyToCruise()`
* 返回结果

### FinishStage 函数
* 都将下一阶段设为 `PARK_AND_GO_ADJUST`
* 将规划上下文中的park_and_go状态标记为不在检查阶段
* 阶段结束 `StageResult(StageStatusType::FINISHED)`

---
## stage_adjust.cc
### Process 函数
* 检查场景不是空
* 初始化变量
    ```cpp
    // 施工绕行场景判断
    static bool shigong_judged = false;
    static int shigong_num = 0;
    static double primary_pos_x = 0.0;
    static double primary_pos_y = 0.0;
    static bool initial_position_set = false; // 标记初始位置是否已设置
    ```
* 在首次进入时设置初始位置（只设置一次）
* 取参考线信息和障碍物信息
* 当障碍物数量足够时进行施工绕行判断
    > 收集所有障碍物的 `x` 坐标，并排序 `x` 坐标
    >
    > 检查是否有足够障碍物 并 获取 `x` 坐标第四大的障碍物
    >
    > 获取感知信息 `perception` 并 判断其感知位置的x坐标是否接近第四大x值
    >
    > 若满足条件，根据障碍物y坐标与车辆初始y坐标（`primary_pos_y`）的关系，以及x坐标与车辆初始x坐标（`primary_pos_x`）的距离，设置施工编号（`shigong_num`）为1、2、3或4。
* 如果满足施工绕行条件，提前结束`Adjust`阶段
* 原有逻辑保持不变
    > 设置 `frame` 对象的开放空间轨迹标志，执行开放空间任务
    >
    > 检查车辆是否具备进入巡航状态的条件
    >
    > 返回结果

### FinishStage 函数
* 完成停车起步场景中的调整阶段（`ParkAndGoStageAdjust`），并决定下一阶段。具体逻辑如下：
    > 获取车辆当前转向角百分比。
    >
    > 若转向角小于配置的最大巡航转向角，则直接进入巡航阶段；否则重置初始位置（`ResetInitPostion();`）后进入巡航阶段。
* 设置下一阶段为 `PARK_AND_GO_CRUISE`

---
## stage_pre_cruise.cc
### Process 函数
* 检查场景
* 设置开放空间轨迹标志位为 `true` : `frame->mutable_open_space_info()->set_is_on_open_space_trajectory(true);`
* 执行开放空间任务
* 判断车辆是否满足进入巡航状态的条件
    > 获取当前车辆状态 `vehicle_status` : `auto vehicle_status = injector_->vehicle_state();`
    >
    > 方向盘转角百分比小于配置的最大值 AND 调用 `CheckADCReadyToCruise` 函数确认车辆已准备好巡航
    >
    > 返回执行结果
    ```cpp
        // 方向盘转角百分比小于配置的最大值 调用 CheckADCReadyToCruise 函数确认车辆已准备好巡航
    if ((std::fabs(vehicle_status->steering_percentage()) <
        scenario_config.max_steering_percentage_when_cruise()) &&
        CheckADCReadyToCruise(injector_->vehicle_state(), frame,
                                scenario_config)) {
        // 调用 FinishStage() 结束当前阶段
        return FinishStage();
    }
    // 返回运行中状态
    return result.SetStageStatus(StageStatusType::RUNNING);
    ```
* 设置下一阶段为 `PARK_AND_GO_CRUISE`

---
## stage_cruise.cc
### Process 函数
* 检查场景
* 在给定的参考线上处理规划任务 `StageResult result = ExecuteTaskOnReferenceLine(planning_init_point, frame);`
* 检查自动驾驶车辆在泊车与巡航场景下的完成状态  `ParkAndGoStatus status = CheckADCParkAndGoCruiseCompleted(reference_line_info);`
* 返回校对结果
    > 如果 `status` 等于 `CRUISE_COMPLETE`，则调用 `FinishStage()` 函数并返回其结果。
    >
    > 否则，设置阶段状态为 `RUNNING`，并通过 `result.SetStageStatus()` 返回当前运行状态

### CheckADCParkAndGoCruiseCompleted 函数
* 获取参考线/获取路径规划结果
* 将车辆当前位置从XY坐标系转换为SL坐标系
    ```cpp
    const common::math::Vec2d adc_position = {injector_->vehicle_state()->x(), injector_->vehicle_state()->y()};
    common::SLPoint adc_position_sl;
    reference_line.XYToSL(adc_position, &adc_position_sl);
    ```
* 遍历路径决策中的障碍物，判断是否存在前方障碍物
    > 遍历所有障碍物，跳过空指针
    >
    > 获取障碍物的感知SL边界（`PerceptionSLBoundary`） 
    >
    > 只判断前方障碍物，找到一个就够了！
* 横向位置相对于参考线的偏差 且 `obstacle_flag` 为假，并返回 `CRUISE_COMPLETE`

---

## parking_and_go_scenario.cc
### Init 函数
确保场景正确初始化并加载相关配置

### IsTransferable 函数
* 前置安全检查、检查终点信息、获取车辆状态
* 计算当前位置 (`current_x, current_y`) 与目标位置之间的距离
* 施工绕行场景判断
    > 获取参考线信息和障碍物信息 `obstacle_count = path_decision.obstacles().Items().size();`
    >
    > 施工绕行-11场景 如果障碍物数量 `obstacle_count` 大于 30 ；距离 `distance` 大于 100.0 -> `true` 满足施工绕行条件
    >
* 当障碍物数量足够时进行施工绕行判断
    > 收集 + 排序所有障碍物的 `x` 坐标
    >
    > 获取x坐标第四大的障碍物
    >
    > 查找对应这个x坐标的障碍物，判断y坐标是否小于车辆初始y坐标     上面同理，略去。
* 如果满足施工绕行条件，直接返回 `true`
* 原有停车再走逻辑保持不变
    > 变量初始化：定义最大停车速度、目标点SL坐标、车辆位置等。
    >
    > 车道检测：检查自车是否在车道上及车道类型是否为城市道路。
    >
    > 距离判断：计算自车到终点的距离，判断是否足够远。
    >
    > 条件综合：若车速低于阈值且距离足够远，同时不在城市道路车道上，则触发park_and_go。
    >
    > 最终返回是否需要停车再启动的布尔值。
